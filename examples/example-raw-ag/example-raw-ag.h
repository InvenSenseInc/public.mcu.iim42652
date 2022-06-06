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

#ifndef _EXAMPLE_RAW_AG_H_
#define _EXAMPLE_RAW_AG_H_

#include <stdint.h>
#include "Ixm42xxxTransport.h"
#include "Ixm42xxxDefs.h"
#include "Ixm42xxxDriver_HL.h"


/**
 * \brief This function is in charge of reseting and initializing Ixm42xxx device. It should
 * be succesfully executed before any access to Ixm42xxx device.
 *
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_ixm42xxx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 *
 * It initialyses clock calibration module (this will allow to extend the 16 bits 
 * timestamp produced by Ixm42xxx to a 64 bits timestamp).
 * Then function sets full scale range and frequency for both accel and gyro and it 
 * starts them in the requested power mode. 
 *
 * \param[in] is_low_noise_mode : if true sensors are started in low-noise mode else in 
 *                                low-power mode.
 * \param[in] is_high_res_mode : if true raw accelerometers and raw gyroscope are used in 20 bits format.
 *                                The fifo will be in 20 byte mode.
 * \param[in] acc_fsr_g :   full scale range for accelerometer. See IXM42XXX_ACCEL_CONFIG0_FS_SEL_t in Ixm42xxxDefs.h
 *                         for possible values.
 * \param[in] gyr_fsr_dps : full scale range for gyroscope. See IXM42XXX_GYRO_CONFIG0_FS_SEL_t in Ixm42xxxDefs.h
 *                         for possible values.
 * \param[in] acc_freq :    accelerometer frequency. See IXM42XXX_ACCEL_CONFIG0_ODR_t in Ixm42xxxDefs.h
 *                         for possible values.
 * \param[in] gyr_freq :    gyroscope frequency. See IXM42XXX_GYRO_CONFIG0_ODR_t in Ixm42xxxDefs.h
 *                         for possible values.
 * \param[in] is_rtc_mode :    requested status for RTC/CLKIN feature
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice(uint8_t is_low_noise_mode,
                       uint8_t is_high_res_mode,
                       IXM42XXX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
					   IXM42XXX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
					   IXM42XXX_ACCEL_CONFIG0_ODR_t acc_freq,
					   IXM42XXX_GYRO_CONFIG0_ODR_t gyr_freq,
					   uint8_t is_rtc_mode);

/**
 * \brief This function extracts data from the Ixm42xxx FIFO.
 *
 * The function just calls Ixm42xxx driver function inv_ixm42xxx_get_data_from_fifo.
 * But note that for each packet extracted from FIFO, a user defined function is called to 
 * allow custom handling of each packet. In this example custom packet handling function
 * is HandleInvDeviceFifoPacket.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromInvDevice(void);

/**
 * \brief This function is the custom handling packet function.
 *
 * It is passed in parameter at driver init time and it is called by 
 * inv_ixm42xxx_get_data_from_fifo function each time a new valid packet is extracted 
 * from FIFO.
 * In this implementation, function extends packet timestamp from 16 to 64 bits and then
 * process data from packet and print them on UART.
 *
 * \param[in] event structure containing sensor data from one packet
 */
void HandleInvDeviceFifoPacket(inv_ixm42xxx_sensor_event_t * event);


#endif /* !_EXAMPLE_RAW_AG_H_ */
