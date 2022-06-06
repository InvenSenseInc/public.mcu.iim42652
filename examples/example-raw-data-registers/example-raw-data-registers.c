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

#include "example-raw-data-registers.h"
 
/* Clock calibration module */
#include "helperClockCalib.h"

#include "Message.h"
//#include "Invn/EmbUtils/RingBuffer.h"


/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the ixm42xxx object */
static struct inv_ixm42xxx icm_driver;

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
	
	rc = inv_ixm42xxx_init(&icm_driver, icm_serif, HandleInvDeviceDataRegisters);
	/* Disable fifo usage, data will be read from sensors registers*/
	rc |= inv_ixm42xxx_configure_fifo(&icm_driver, INV_IXM42XXX_FIFO_DISABLED);
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
                       IXM42XXX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       IXM42XXX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       IXM42XXX_ACCEL_CONFIG0_ODR_t acc_freq,
                       IXM42XXX_GYRO_CONFIG0_ODR_t gyr_freq,
                       uint8_t is_rtc_mode)
{
	int rc = 0;
	
	rc |= inv_ixm42xxx_enable_clkin_rtc(&icm_driver, is_rtc_mode);

	rc |= inv_ixm42xxx_set_accel_fsr(&icm_driver, acc_fsr_g);
	rc |= inv_ixm42xxx_set_gyro_fsr(&icm_driver, gyr_fsr_dps);
	
	rc |= inv_ixm42xxx_set_accel_frequency(&icm_driver, acc_freq);
	rc |= inv_ixm42xxx_set_gyro_frequency(&icm_driver, gyr_freq);
	
	if (is_low_noise_mode)
		rc |= inv_ixm42xxx_enable_accel_low_noise_mode(&icm_driver);
	else
		rc |= inv_ixm42xxx_enable_accel_low_power_mode(&icm_driver);
	
	rc |= inv_ixm42xxx_enable_gyro_low_noise_mode(&icm_driver);

	/* Wait Max of IXM42XXX_GYR_STARTUP_TIME_US and IXM42XXX_ACC_STARTUP_TIME_US*/
	(IXM42XXX_GYR_STARTUP_TIME_US > IXM42XXX_ACC_STARTUP_TIME_US) ? inv_ixm42xxx_sleep_us(IXM42XXX_GYR_STARTUP_TIME_US) : inv_ixm42xxx_sleep_us(IXM42XXX_ACC_STARTUP_TIME_US);
		
	return rc;
}

int GetDataFromInvDevice(void)
{
	/*
	 * Read data from registers. Callback defined at init time (i.e. 
	 * HandleInvDeviceDataRegisters) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	return inv_ixm42xxx_get_data_from_registers(&icm_driver);
}

void HandleInvDeviceDataRegisters(inv_ixm42xxx_sensor_event_t * event)
{
	uint64_t irq_timestamp = 0;	
	
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
	 * Output data on UART link
	 */
	if((event->accel[0] != INVALID_VALUE_FIFO) && (event->gyro[0] != INVALID_VALUE_FIFO))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, %d, %d, %d", (uint32_t)irq_timestamp,
			event->accel[0], event->accel[1], event->accel[2], 
			event->temperature,
			event->gyro[0], event->gyro[1], event->gyro[2]);
	else if(event->gyro[0] != INVALID_VALUE_FIFO)
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: NA, NA, NA, %d, %d, %d, %d", (uint32_t)irq_timestamp, 
			event->temperature,
			event->gyro[0], event->gyro[1], event->gyro[2]);
	else if (event->accel[0] != INVALID_VALUE_FIFO)
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, NA, NA, NA", (uint32_t)irq_timestamp,
			event->accel[0], event->accel[1], event->accel[2], 
			event->temperature);
}