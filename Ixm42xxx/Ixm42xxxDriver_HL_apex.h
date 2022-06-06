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

/** @defgroup DriverIxm42xxxDriver_HL_apex Ixm42xxx driver high level functions related to APEX
 *  @brief High-level function to setup an Ixm42xxx device
 *  @ingroup  DriverIxm42xxx
 *  @{
 */

/** @file Ixm42xxxDriver_HL_apex.h
 * High-level function to setup an Ixm42xxx device
 */

#ifndef _INV_IXM42XXX_DRIVER_HL_APEX_H_
#define _INV_IXM42XXX_DRIVER_HL_APEX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Ixm42xxxDefs.h"

#include "InvError.h"

#include <stdint.h>
#include <string.h>

/* Forward declarations */
struct inv_ixm42xxx;

/** @brief Ixm42xxx TAP inputs parameters definition
 */
typedef struct {
	uint8_t min_jerk_thr;	                                /**< Minimum Jerk Threshold */
	IXM42XXX_APEX_CONFIG7_TAP_MAX_PEAK_TOL_t max_peak_tol;  /**< Maximum peak tolerance */
	IXM42XXX_APEX_CONFIG8_TAP_TMAX_t tmax;                  /**< Tap measurement window */
	IXM42XXX_APEX_CONFIG8_TAP_TAVG_t tavg;                  /**< Energy measumerement window */
	IXM42XXX_APEX_CONFIG8_TAP_TMIN_t tmin;                  /**< Single tap window */
} inv_ixm42xxx_tap_parameters_t;

/** @brief Ixm42xxx APEX inputs parameters definition
 */
typedef struct inv_ixm42xxx_apex_parameters {
	IXM42XXX_APEX_CONFIG2_PEDO_AMP_TH_t pedo_amp_th;                   /**< Peak threshold value to be considered as a valid step (mg) */
	uint8_t pedo_step_cnt_th;                                          /**< Minimum number of steps that must be detected 
	                                                                        before the pedometer step count begins incrementing */
	uint8_t pedo_step_det_th;                                          /**< Minimum number of low latency steps that must be detected 
	                                                                        before the pedometer step count begins incrementing */
	IXM42XXX_APEX_CONFIG3_PEDO_SB_TIMER_TH_t pedo_sb_timer_th;         /**< Duration of non-walk to exit the current walk mode, 
	                                                                        pedo_step_cnt_th number of steps must again be detected 
	                                                                        before step count starts to increase */
	IXM42XXX_APEX_CONFIG3_PEDO_HI_ENRGY_TH_t pedo_hi_enrgy_th;         /**< Threshold to improve run detection if not steps are counted while running */
 
	IXM42XXX_APEX_CONFIG4_TILT_WAIT_TIME_t tilt_wait_time;             /**< Number of accelerometer samples to wait before triggering tilt event */
 
	IXM42XXX_APEX_CONFIG1_DMP_POWER_SAVE_TIME_t power_save_time;       /**< The time after which DMP goes in power save mode according to the DMP ODR configured */
	IXM42XXX_APEX_CONFIG0_DMP_POWER_SAVE_t power_save;                 /**< Power save mode for APEX algorithms. This mode will put APEX features into sleep mode, 
	                                                                        leaving only the WOM running to wake-up the DMP */
	
	IXM42XXX_APEX_CONFIG9_SENSITIVITY_MODE_t sensitivity_mode;         /**< Sensitivity mode Normal(0) or Slow walk(1). The Slow walk mode improve 
	                                                                        the slow walk detection (<1Hz) but in return the number of false detection 
	                                                                        might be increase. */

	IXM42XXX_APEX_CONFIG10_FF_DEBOUNCE_DURATION_t ff_debounce_duration;      /**< Duration(us) during which LowG and HighG events are not taken into account after an HighG event.
	                                                                             The goal is to avoid detecting bounces as free falls */
	IXM42XXX_APEX_CONFIG10_FF_MAX_DURATION_t ff_max_duration_cm;            /**< Distance (cm) max crossed by the device after a LowG event, below which the detection of an HighG event triggers a freefall interrupt */
	IXM42XXX_APEX_CONFIG10_FF_MIN_DURATION_t ff_min_duration_cm;            /**< Distance (cm) min crossed by the device after a LowG event, above which the detection of an HighG event triggers a freefall interrupt */
	
	IXM42XXX_APEX_CONFIG5_LOWG_PEAK_TH_t lowg_peak_th;                     /**< Absolute low peak threshold (mg) to detect when it falls below on any of the accelerometer axis */
	IXM42XXX_APEX_CONFIG4_LOWG_PEAK_TH_HYST_t lowg_peak_hyst;              /**< Hysteresis threshold (mg) added to the threshold after the initial threshold is met */
	IXM42XXX_APEX_CONFIG5_LOWG_TIME_TH_SAMPLES_t lowg_samples_th;          /**< Time in number of samples to stay below the threshold before triggering the event (samples) */
	
	IXM42XXX_APEX_CONFIG6_HIGHG_PEAK_TH_t highg_peak_th;                   /**< Absolute low peak threshold (mg) to detect when it falls below on any of the accelerometer axis */
	IXM42XXX_APEX_CONFIG4_HIGHG_PEAK_TH_HYST_t highg_peak_hyst;            /**< Hysteresis threshold (mg) added to the threshold after the initial threshold is met */
	IXM42XXX_APEX_CONFIG6_HIGHG_TIME_TH_SAMPLES_t highg_samples_th;        /**< Time in number of samples to stay below the threshold before triggering the event (samples) */
	
	IXM42XXX_APEX_CONFIG1_LOW_ENERGY_AMP_TH_t low_energy_amp_th;       /**< Peak threshold value to be considered as a valid step (mg) in Slow walk mode */
} inv_ixm42xxx_apex_parameters_t;

/** @brief APEX pedometer outputs
 */
typedef struct inv_ixm42xxx_apex_step_activity {
	uint16_t step_cnt;      /**< Number of steps taken */
	uint8_t step_cadence;   /**< Walk/run cadency in number of samples. 
	                             Format is u6.2. E.g, At 50Hz and 2Hz walk frequency, if the cadency is 25 samples. The register will output 100. */
	uint8_t activity_class; /**< Detected activity unknown (0), walk (1) or run (2) */
} inv_ixm42xxx_apex_step_activity_t;

/** @brief TAP outputs
 */
typedef struct inv_ixm42xxx_tap_data {
	IXM42XXX_APEX_DATA4_TAP_NUM_t tap_num;     /**< Detects single (1) or double (2) tap*/
	IXM42XXX_APEX_DATA4_TAP_AXIS_t tap_axis;   /**< Axis along which tap has been detected */
	IXM42XXX_APEX_DATA4_TAP_DIR_t tap_dir;     /**< Direction of the tap, either +axis (1) or -axis (0)*/
	uint8_t double_tap_timing;                 /**< Timing between both taps of a double tap expressed in 1/16th of odr in ms (e.g At 500Hz, 2 means 64ms between each tap) */
} inv_ixm42xxx_tap_data_t;


/** @brief  Configure Wake On Motion and SMD thresholds.
 *  @param[in] x_th threshold value for the Wake on Motion Interrupt for X-axis accelerometer.
 *  @param[in] y_th threshold value for the Wake on Motion Interrupt for Y-axis accelerometer.
 *  @param[in] z_th threshold value for the Wake on Motion Interrupt for Z-axis accelerometer.
 *  @param[in] wom_int select which mode between AND/OR is used to generate interrupt.
 *  @param[in] wom_mode select which comparison mode is used for WoM detection.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_configure_smd_wom(struct inv_ixm42xxx * s, const uint8_t x_th, const uint8_t y_th, const uint8_t z_th, IXM42XXX_SMD_CONFIG_WOM_INT_MODE_t wom_int, IXM42XXX_SMD_CONFIG_WOM_MODE_t wom_mode);

/** @brief  Enable Wake On Motion.
 *  note : WoM requests to have the accelerometer enabled to work. 
 *  Enables WoM event generation and configures interrupt to fire on WoM event.
 *  As a consequence Fifo water-mark interrupt is disabled.
 *  To have good performance, it's recommended to set accelerometer ODR (Output Data Rate) to 20ms
 *  and the accelerometer in Low Power Mode.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_enable_wom(struct inv_ixm42xxx * s);

/** @brief  Disable Wake On Motion.
 *  Disables WoM event generation and reconfigures interrupt to fire on Fifo water-mark.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_wom(struct inv_ixm42xxx * s);

/** @brief  Enable Significant Motion Detection.
 *  note : SMD requests to have the accelerometer enabled to work.
 *  Enables SMD event generation and configures interrupt to fire on SMD event. WoM event will also be generated.
 *  To have good performance, it's recommended to set accelerometer ODR (Output Data Rate) to 20ms
 *  and the accelerometer in Low Power Mode.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_enable_smd(struct inv_ixm42xxx * s);

/** @brief  Disable Significant Motion Detection.
 *  Disables SMD event generation and disables SMD interrupt. 
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_smd(struct inv_ixm42xxx * s);

/** @brief Fill the TAP parameters structure with all the default parameters for TAP algorithm
 *  @param[out] tap_inputs Default input parameters. See @sa inv_ixm42xxx_tap_parameters_t 
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_init_tap_parameters_struct(struct inv_ixm42xxx *s, inv_ixm42xxx_tap_parameters_t *tap_inputs);

/** @brief  Configure TAP.
 *  @param[in] tap_inputs The requested input parameters. See @sa inv_ixm42xxx_tap_parameters_t
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_configure_tap_parameters(struct inv_ixm42xxx * s, const inv_ixm42xxx_tap_parameters_t *tap_inputs);

/** @brief Returns current TAP parameters.
 *  @param[out] tap_params The current parameter, fetched from registers. See @sa inv_ixm42xxx_tap_parameters_t
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_get_tap_parameters(struct inv_ixm42xxx *s, inv_ixm42xxx_tap_parameters_t *tap_params);

/** @brief  Enable TAP.
 *  note : TAP requests to have the accelerometer enabled to work. 
 *  To have good performance, it's recommended to set accelerometer ODR (Output Data Rate) to 1ms
 *  and the accelerometer in Low Noise Mode.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_enable_tap(struct inv_ixm42xxx * s);

/** @brief  Disable TAP.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_tap(struct inv_ixm42xxx * s);

/** @brief Fill the APEX parameters structure with all the default parameters for APEX algorithms (pedometer, tilt)
 *  @param[out] apex_inputs Default input parameters. See @sa inv_ixm42xxx_apex_parameters_t 
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_init_apex_parameters_struct(struct inv_ixm42xxx *s, inv_ixm42xxx_apex_parameters_t *apex_inputs);

/** @brief Configures DMP parameters for APEX algorithms (pedometer, tilt). 
 *         This programmable parameters will be decoded and propagate to the SRAM to be executed at DMP start.
 *  @param[in] apex_inputs The requested input parameters. See @sa inv_ixm42xxx_apex_parameters_t
 *  @warning APEX inputs can't change on the fly, this API should be called before enabling any APEX features.
 *  @warning APEX configuration can't be done too frequently, but only once every 10ms. Otherwise it can create unknown behavior.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_configure_apex_parameters(struct inv_ixm42xxx *s, const inv_ixm42xxx_apex_parameters_t *apex_inputs);

/** @brief Returns current DMP parameters for APEX algorithms (pedometer, tilt).
 *  @param[out] apex_params The current parameter, fetched from registers. See @sa inv_ixm42xxx_apex_parameters_t
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_get_apex_parameters(struct inv_ixm42xxx *s, inv_ixm42xxx_apex_parameters_t *apex_params);

/** @brief Configure DMP Output Data Rate for APEX algorithms (pedometer, tilt)
 *  @param[in] frequency The requested frequency.
 *  @sa IXM42XXX_APEX_CONFIG0_DMP_ODR_t
 *  @warning DMP_ODR can change on the fly, and the DMP code will accommodate necessary modifications
 *  @warning The user needs to take care to set Accel frequency >= DMP frequency. This is a hard constraint 
 since HW will not handle incorrect setting.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_set_apex_frequency(struct inv_ixm42xxx * s, const IXM42XXX_APEX_CONFIG0_DMP_ODR_t frequency);

/** @brief Start DMP for APEX algorithms
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_start_dmp(struct inv_ixm42xxx * s);

/** @brief Reset DMP memory
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_reset_dmp(struct inv_ixm42xxx * s);

/** @brief  Enable APEX algorithm Pedometer.
 *  note : Pedometer request to have the accelerometer enabled to works with accelerometer frequency less than dmp frequency. 
 *  @return 0 on success, negative value on error.
 *  @warning Pedometer must be turned OFF to reconfigure it
 */
int inv_ixm42xxx_enable_apex_pedometer(struct inv_ixm42xxx * s);

/** @brief  Disable APEX algorithm Pedometer.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_apex_pedometer(struct inv_ixm42xxx * s);

/** @brief  Enable Free Fall.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_enable_apex_ff(struct inv_ixm42xxx * s);

/** @brief  Disable Free Fall.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_apex_ff(struct inv_ixm42xxx * s);

/** @brief  Disable LowG.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_apex_lowg(struct inv_ixm42xxx * s);

/** @brief  Enable LowG.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_enable_apex_lowg(struct inv_ixm42xxx * s);

/** @brief  Enable APEX algorithm Tilt.
 *  note : Tilt request to have the accelerometer enabled to works with accelerometer frequency less than dmp frequency. 
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_enable_apex_tilt(struct inv_ixm42xxx * s);

/** @brief  Disable APEX algorithm Tilt.
 *  @return 0 on success, negative value on error.
 */
int inv_ixm42xxx_disable_apex_tilt(struct inv_ixm42xxx * s);

/** @brief  Retrieve APEX pedometer outputs and format them
 *  @param[out] apex_activity Apex step and activity data value.
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_ixm42xxx_get_apex_data_activity(struct inv_ixm42xxx * s, inv_ixm42xxx_apex_step_activity_t * apex_activity);

/** @brief  Retrieve tap outputs
 *  @param[out] tap_data Tap axis, direction and type
 *  @return 0 in case of success, negative value on error. See enum inv_error
 */
int inv_ixm42xxx_get_tap_data(struct inv_ixm42xxx * s, inv_ixm42xxx_tap_data_t * tap_data);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IXM42XXX_DRIVER_HL_APEX_H_ */

/** @} */
