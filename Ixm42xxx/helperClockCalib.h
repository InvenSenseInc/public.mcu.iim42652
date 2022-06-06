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

#ifndef _STANDALONE_HELPER_H_
#define _STANDALONE_HELPER_H_

#include <stdint.h>

#include "Ixm42xxxDefs.h"
#include "Ixm42xxxExtFunc.h"
#include "Ixm42xxxDriver_HL.h"


/*
 * Max value for 20 bits and 16 bits counter
 */
#define ROLLOVER_20BITS 0x100000
#define ROLLOVER_16BITS 0x10000

/*
 * Time between 2 MCU/ICM clock calibrations and for 1 calibration
 */
#define TIME_US_FOR_CLOCK_CALIBRATION 1200000

/*
 * Number of neccessary rollover occured during the TIME_US_FOR_CLOCK_CALIBRATION
 */
#define NB_ROLLOVER    (uint8_t)(TIME_US_FOR_CLOCK_CALIBRATION / TIME_US_FOR_20BITS_TMSP_ROLLOVER)

/*
 * Number of samples needed before computing the clock calibration coefficient 
 * To round up the division is critical when the number of samples is low, i.e when the sensor frequency set is low.
 */
#define NUMBER_OF_SAMPLES_FOR_ODR(odr_us)   TIME_US_FOR_CLOCK_CALIBRATION/(odr_us) + TIME_US_FOR_CLOCK_CALIBRATION%(odr_us);


/* Interrupt enum state for PLL, RC_OSC and WU_OSC */
enum inv_ixm42xxx_clock_source{
	INV_IXM42XXX_PLL,
	INV_IXM42XXX_RC_OSC,
	INV_IXM42XXX_WU_OSC,
	INV_IXM42XXX_CLOCK_SOURCE_MAX
};

/* 
 * Coefficient to interpolate the linear regression between the ICM time and the MCU time
 */
typedef struct clk_calib{
	int initial_recalib_after_n_samples;
	int recalib_after_n_samples;
	uint8_t on_going;
	float    coef[INV_IXM42XXX_CLOCK_SOURCE_MAX];   /**< Calibration coefficient */
	uint64_t last_timestamp_sent[INV_IXM42XXX_SENSOR_MAX]; /**< last timestamp sent when using the extended timestamp API */
	uint32_t last_fifo_timestamp[INV_IXM42XXX_SENSOR_MAX]; /**< last timestamp read from FIFO */
	float    dt_error;                                     /**< keep track of the error on the timestamp due to the usec resolution */
} clk_calib_t;

/** @brief Extend timestamp from FIFO by getting time base from upper layer
 *  @param[in] states     placeholder to inv_ixm42xxx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 *  @param[in] timestamp_fsync according which is selected :
 *         - timestamp counter  absolute time at 16us/1us rate resolution according to settings (16us by default)
 *         - fsync counter 
 * @param[in] irq_timestamp     interrupt data ready timestamp computing by upper layer
 * @param[in] sensor_mask    sensors which have location in the FIFO packet
 * @param[out] timestamp extended timestamp
 */
int inv_helper_extend_timestamp_from_fifo(struct inv_ixm42xxx * s, struct clk_calib *clk_cal, 
		uint16_t timestamp_fsync, uint64_t irq_timestamp, int sensor_mask, uint64_t * timestamp);

/** @brief Intitialisation function to be executed before clock_calibration_update
 *  @param[in] states     placeholder to inv_ixm42xxx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
void clock_calibration_reset(struct inv_ixm42xxx * s, struct clk_calib *clk_cal);

/** @brief Reset calibration data linked to acc and gyr sensors. This is typically used when 
 *  enabling a sensor.
 *
 *  Note: This function should be called when enabling Accel or Gyro
 *
 *  @param[in] states     placeholder to inv_ixm42xxx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 *  @param[in] sensor     sensor to be restarted (INV_IXM42XXX_SENSOR_ACCEL or INV_IXM42XXX_SENSOR_ACCEL)
 */
void clock_calibration_reset_sensors_stats(struct inv_ixm42xxx * s, struct clk_calib *clk_cal, enum inv_ixm42xxx_sensor );

/** @brief First calibration done just after start'up to calculate first time factor
 *  @param[in] states     placeholder to inv_ixm42xxx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
int clock_calibration_init(struct inv_ixm42xxx * s, struct clk_calib *clk_cal);

/** @brief Update time factor for clock calibration
 *  @param[in] states         placeholder to inv_ixm42xxx_t states
 *  @param[in] clk_calib      placeholder to clk_calib_t states
 *  @param[in] irq_timestamp  timestamp from the MCU to be used to compute coefficient (need to be taken at the same time as fifo_timestamp)
 *  @param[in] fifo_timestamp timestamp from the FIFO (ICM) to be used to compute coefficient (need to be taken at the same time as irq_timestamp)
 */
int clock_calibration_update(struct inv_ixm42xxx * s, struct clk_calib *clk_cal, uint64_t irq_timestamp, uint16_t fifo_timestamp);

/** @brief Restart the computation of the time factor.
 * This function finds the smallest ODR of running sensors among ACC and GYR. Then it restarts
 * the time deviation computation based on this ODR.
 *
 *  Note: This function should be called when enabling Accel or Gyro or when changing ODR
 *
 *  @param[in] states     placeholder to inv_ixm42xxx_t states
 *  @param[in] clk_calib  placeholder to clk_calib_t states
 */
int clock_calibration_restart(struct inv_ixm42xxx * s, struct clk_calib *clk_cal);

/*!
 * \brief Converter function from period (in usec) to frequency (in Hz)
 */
uint32_t period_us_to_frequency(const uint32_t period_us);

/** @brief Hook to disable IRQ in low level.
 */
extern void inv_helper_disable_irq(void);

/** @brief Hook to enable IRQ in low level.
 */
extern void inv_helper_enable_irq(void);

#endif /* !_STANDALONE_HELPER_H_ */
