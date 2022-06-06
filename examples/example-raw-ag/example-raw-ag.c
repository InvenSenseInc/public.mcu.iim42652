/*
 * __________________________________________________________________
 *
 * Copyright (C) [2022] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY  AND FITNESS. IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE  FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * __________________________________________________________________
 */

#include "example-raw-ag.h"
 
/* Clock calibration module */
#include "helperClockCalib.h"

#include "Message.h"
//#include "Invn/EmbUtils/RingBuffer.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the ixm42xxx object */
static struct inv_ixm42xxx icm_driver;

/* structure allowing to handle clock calibration */
static clk_calib_t clk_calib;

/* Buffer to keep track of the timestamp when ixm42xxx data ready interrupt fires. */
//extern  RINGBUFFER(timestamp_buffer, 64, uint64_t);


/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

int SetupInvDevice(struct inv_ixm42xxx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;
	
	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Ixm42xxx");
	
	rc = inv_ixm42xxx_init(&icm_driver, icm_serif, HandleInvDeviceFifoPacket);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Ixm42xxx.");
		return rc;
	}	
	
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_INFO, "Check Ixm42xxx whoami value");
	
	rc = inv_ixm42xxx_get_who_am_i(&icm_driver, &who_am_i);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Ixm42xxx whoami value.");
		return rc;
	}
	
	if(who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	//RINGBUFFER_CLEAR(&timestamp_buffer);
	return rc;
}


int ConfigureInvDevice(uint8_t is_low_noise_mode,
                       uint8_t is_high_res_mode,
                       IXM42XXX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       IXM42XXX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       IXM42XXX_ACCEL_CONFIG0_ODR_t acc_freq,
                       IXM42XXX_GYRO_CONFIG0_ODR_t gyr_freq,
						uint8_t is_rtc_mode)
{
	int rc = 0;
	
	if (!is_rtc_mode) {
		/*
		 * Compute the time drift between the MCU and ICM clock
		 */
		rc |= clock_calibration_init(&icm_driver, &clk_calib);
	} else {
		clock_calibration_reset(&icm_driver, &clk_calib);
		clk_calib.coef[INV_IXM42XXX_PLL] = 1.0f;
		clk_calib.coef[INV_IXM42XXX_RC_OSC] = 1.0f;
		clk_calib.coef[INV_IXM42XXX_WU_OSC] = 1.0f;
	}
	
	/* 
	 * Force or prevent CLKIN usage depending on example configuration
	 * Note that CLKIN can't be forced if part is not trimmed accordingly
	 * It can be always disabled however, whatever the part used
	 */
	rc |= inv_ixm42xxx_enable_clkin_rtc(&icm_driver, is_rtc_mode);

	if(is_high_res_mode)
		rc |= inv_ixm42xxx_enable_high_resolution_fifo(&icm_driver);
	else {
		rc |= inv_ixm42xxx_set_accel_fsr(&icm_driver, acc_fsr_g);
		rc |= inv_ixm42xxx_set_gyro_fsr(&icm_driver, gyr_fsr_dps);
	}
	
	rc |= inv_ixm42xxx_set_accel_frequency(&icm_driver, acc_freq);
	rc |= inv_ixm42xxx_set_gyro_frequency(&icm_driver, gyr_freq);
	
	if (is_low_noise_mode)
		rc |= inv_ixm42xxx_enable_accel_low_noise_mode(&icm_driver);
	else
		rc |= inv_ixm42xxx_enable_accel_low_power_mode(&icm_driver);
	
	rc |= inv_ixm42xxx_enable_gyro_low_noise_mode(&icm_driver);

	return rc;
}


int GetDataFromInvDevice(void)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. 
	 * HandleInvDeviceFifoPacket) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	return inv_ixm42xxx_get_data_from_fifo(&icm_driver);
}


void HandleInvDeviceFifoPacket(inv_ixm42xxx_sensor_event_t * event)
{
	uint64_t irq_timestamp = 0, extended_timestamp;
	int32_t accel[3], gyro[3];
	
	/*
	 * Extract the timestamp that was buffered when current packet IRQ fired. See 
	 * ext_interrupt_cb() in main.c for more details.
	 * As timestamp buffer is filled in interrupt handler, we should pop it with
	 * interrupts disabled to avoid any concurrent access.
	 */
	//inv_disable_irq();
	//if (!RINGBUFFER_EMPTY(&timestamp_buffer))
	//	RINGBUFFER_POP(&timestamp_buffer, &irq_timestamp);
	//inv_enable_irq();
	
	/*
	 * Extend the 16-bit timestamp from the Ixm42xxx FIFO to a 64 bits timestamp.
	 */
	inv_helper_extend_timestamp_from_fifo(&icm_driver, &clk_calib, event->timestamp_fsync, irq_timestamp, event->sensor_mask, &extended_timestamp);	
	
	/*
	 * Compute raw data according to the format
	 */
	if(icm_driver.fifo_highres_enabled) {
		accel[0] = (((int32_t)event->accel[0] << 4)) | event->accel_high_res[0];
		accel[1] = (((int32_t)event->accel[1] << 4)) | event->accel_high_res[1];
		accel[2] = (((int32_t)event->accel[2] << 4)) | event->accel_high_res[2];
		
		gyro[0] = (((int32_t)event->gyro[0] << 4)) | event->gyro_high_res[0];
		gyro[1] = (((int32_t)event->gyro[1] << 4)) | event->gyro_high_res[1];
		gyro[2] = (((int32_t)event->gyro[2] << 4)) | event->gyro_high_res[2];
		
	} else {
		accel[0] = event->accel[0];
		accel[1] = event->accel[1];
		accel[2] = event->accel[2];
		
		gyro[0] = event->gyro[0];
		gyro[1] = event->gyro[1];
		gyro[2] = event->gyro[2];
	}
	
	/*
	 * Output data on UART link
	 */
	
	if(event->sensor_mask & (1 << INV_IXM42XXX_SENSOR_ACCEL) && event->sensor_mask & (1 << INV_IXM42XXX_SENSOR_GYRO))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, %d, %d, %d", (uint32_t)extended_timestamp,
		        accel[0], accel[1], accel[2], 
		        event->temperature,
		        gyro[0], gyro[1], gyro[2]);
	else if(event->sensor_mask & (1 << INV_IXM42XXX_SENSOR_GYRO))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: NA, NA, NA, %d, %d, %d, %d", (uint32_t)extended_timestamp,
		        event->temperature,
		        gyro[0], gyro[1], gyro[2]);
	else if (event->sensor_mask & (1 << INV_IXM42XXX_SENSOR_ACCEL))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, NA, NA, NA", (uint32_t)extended_timestamp,
		        accel[0], accel[1], accel[2],
		        event->temperature);

}
