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
 
#include "Ixm42xxxSelfTest.h"
#include "Ixm42xxxDefs.h"
#include "Ixm42xxxExtFunc.h"
#include "Ixm42xxxTransport.h"
#include "Ixm42xxxDriver_HL.h"

#include <stdio.h>
#include <math.h>


#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif


/** register configuration for self-test procedure */
#define ST_GYRO_FSR             IXM42XXX_GYRO_CONFIG0_FS_SEL_250dps
#define ST_GYRO_ODR             IXM42XXX_GYRO_CONFIG0_ODR_1_KHZ
#define ST_GYRO_UI_FILT_ORD_IND IXM42XXX_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER
#define ST_GYRO_UI_FILT_BW_IND  IXM42XXX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10

#define ST_ACCEL_FSR             IXM42XXX_ACCEL_CONFIG0_FS_SEL_2g
#define ST_ACCEL_ODR             IXM42XXX_ACCEL_CONFIG0_ODR_1_KHZ
#define ST_ACCEL_UI_FILT_ORD_IND IXM42XXX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER
#define ST_ACCEL_UI_FILT_BW_IND  IXM42XXX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10

/* Formula to get ST_OTP based on FS and ST_code */
#if defined(ICM42686)
#define INV_ST_OTP_EQUATION(FS, ST_code) (uint32_t)((1310/pow(2,3-FS))*pow(1.01, ST_code-1)+0.5)
#else
#define INV_ST_OTP_EQUATION(FS, ST_code) (uint32_t)((2620/pow(2,3-FS))*pow(1.01, ST_code-1)+0.5)
#endif

/* Pass/Fail criteria */
#define MIN_RATIO_GYRO         0.5f /* expected ratio greater than 0.5 */ 
#define MAX_RATIO_GYRO         1.5f /* expected ratio lower than 1.5 */
#define MIN_ST_GYRO_DPS        60   /* expected values greater than 60dps */
#define MAX_ST_GYRO_OFFSET_DPS 20   /* expected offset less than 20 dps */

#define MIN_RATIO_ACCEL        0.5f /* expected ratio greater than 0.5 */ 
#define MAX_RATIO_ACCEL        1.5f /* expected ratio lower than 1.5 */ 
#define MIN_ST_ACCEL_MG        50   /* expected values in [50mgee;1200mgee] */
#define MAX_ST_ACCEL_MG        1200


/** @brief Ixm42xxx HW Base sensor status based upon s->sensor_on_mask
 */
enum inv_ixm42xxx_sensor_on_mask {
	INV_IXM42XXX_SENSOR_ON_MASK_ACCEL = (1L<<INV_IXM42XXX_SENSOR_ACCEL),
	INV_IXM42XXX_SENSOR_ON_MASK_GYRO  = (1L<<INV_IXM42XXX_SENSOR_GYRO),
};

/** @brief Contains the current register values. Used to reapply values after the ST procedure
 */
struct recover_regs {
	/* bank 0 */
	uint8_t intf_config1;       /* REG_INTF_CONFIG1       */
	uint8_t pwr_mgmt_0;         /* REG_PWR_MGMT_0         */
	uint8_t accel_config0;      /* REG_ACCEL_CONFIG0      */
	uint8_t accel_config1;      /* REG_ACCEL_CONFIG1      */
	uint8_t gyro_config0;       /* REG_GYRO_CONFIG0       */
	uint8_t gyro_config1;       /* REG_GYRO_CONFIG1       */
	uint8_t accel_gyro_config0; /* REG_ACCEL_GYRO_CONFIG0 */
	uint8_t fifo_config1;       /* REG_FIFO_CONFIG1       */
	uint8_t self_test_config;   /* REG_SELF_TEST_CONFIG   */
};

/* Static fonctions definition */
static int run_gyro_self_test(struct inv_ixm42xxx * s, int * result);
static int run_accel_self_test(struct inv_ixm42xxx * s, int * result);
static int average_sensor_output(struct inv_ixm42xxx * s, int sensor, int self_test_config, int32_t sensor_result[3]);
static int save_settings(struct inv_ixm42xxx * s, struct recover_regs * saved_regs);
static int recover_settings(struct inv_ixm42xxx * s, const struct recover_regs * saved_regs);
static int set_user_offset_regs(struct inv_ixm42xxx * s, uint8_t sensor);
static int reg_to_accel_fsr(IXM42XXX_ACCEL_CONFIG0_FS_SEL_t reg);
static int reg_to_gyro_fsr(IXM42XXX_GYRO_CONFIG0_FS_SEL_t reg);


int inv_ixm42xxx_run_selftest(struct inv_ixm42xxx * s, int * result)
{
	int status = 0;
	int gyro_result = 0, accel_result = 0;
	struct recover_regs saved_regs;

	*result = 0;

	/* Run self-test only once */
	if (s->st_result == 0) {

		/* Save current settings to restore them at the end of the routine */
		status |= save_settings(s, &saved_regs);

		status |= run_gyro_self_test(s, &gyro_result);
		if ((status == 0) && (gyro_result == 1))
			status |= set_user_offset_regs(s, INV_IXM42XXX_SENSOR_ON_MASK_GYRO);
		
		status |= run_accel_self_test(s, &accel_result);
		if ((status == 0) && (accel_result == 1))
			status |= set_user_offset_regs(s, INV_IXM42XXX_SENSOR_ON_MASK_ACCEL);

		/* Restore settings previously saved */
		status |= recover_settings(s, &saved_regs);
		
		/* Store acc and gyr results */
		s->st_result = (accel_result << 1) | gyro_result;
	}

	*result = s->st_result;
	return status;
}

int inv_ixm42xxx_get_st_bias(struct inv_ixm42xxx * s, int st_bias[6])
{
	int status = 0;
	int i;
	
	/* if ST didn't run, return null biases */
	if (s->st_result == 0) {
		for (i = 0; i < 6; i++)
			st_bias[i] = 0;
		return status;
	}

	/* Gyro bias LN: first 3 elements */
	for (i = 0; i < 3; i++) /* convert bias to 1 dps Q16 */
		st_bias[i] = s->gyro_st_bias[i] * 2 * reg_to_gyro_fsr(ST_GYRO_FSR);

	/* Accel bias LN: last 3 elements */
	for (i = 0; i < 3; i++) /* convert bias to 1 gee Q16 */
		st_bias[i+3] = s->accel_st_bias[i] * 2 * reg_to_accel_fsr(ST_ACCEL_FSR);

	return status;
}

int inv_ixm42xxx_set_st_bias(struct inv_ixm42xxx * s, const int st_bias[6])
{
	int status = 0;
	int i;
	
	/* Gyro */
	for (i = 0; i < 3; i++)
		s->gyro_st_bias[i] = st_bias[i] / (2 * reg_to_gyro_fsr(ST_GYRO_FSR));

	status |= set_user_offset_regs(s, INV_IXM42XXX_SENSOR_ON_MASK_GYRO);
	
	/* Accel */
	for (i = 0; i < 3; i++)
		s->accel_st_bias[i] = st_bias[i+3] / (2 * reg_to_accel_fsr(ST_ACCEL_FSR));

	status |= set_user_offset_regs(s, INV_IXM42XXX_SENSOR_ON_MASK_ACCEL);

	return status;
}

/*
 * Params:
 *   - result: 1 if success, 0 if failure
 * Returns 0 if success, error code if failure
 */
static int run_gyro_self_test(struct inv_ixm42xxx * s, int * result)
{
	int status = 0;
	uint8_t data;
	uint8_t bank;

	int32_t STG_OFF[3], STG_ON[3];
	uint32_t STG_response[3];

	uint8_t STG_code[3];
	uint32_t STG_OTP[3];

	int i = 0;
	
	uint32_t gyro_sensitivity_1dps = 32768 / reg_to_gyro_fsr(ST_GYRO_FSR);;
	
	*result = 1;

	/* Set gyro configuration */
	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &data);
	data &= ~BIT_GYRO_CONFIG0_FS_SEL_MASK;
	data &= ~BIT_GYRO_CONFIG0_ODR_MASK;
	data |= ST_GYRO_FSR;
	data |= ST_GYRO_ODR;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &data);

	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_CONFIG1, 1, &data);
	data &= ~BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK;
	data |= ST_GYRO_UI_FILT_ORD_IND;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_CONFIG1, 1, &data);
	
	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &data);
	data &= ~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK; 
	data |= ST_GYRO_UI_FILT_BW_IND;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &data);

	/* Read average gyro digital output for each axis and store them as STG_OFF_{x,y,z} in lsb */
	status |= average_sensor_output(s, INV_IXM42XXX_SENSOR_ON_MASK_GYRO, 0, STG_OFF);

	/* Enable self-test for each axis and read average gyro digital output 
	 * for each axis and store them as STG_ON_{x,y,z} in lsb */
	status |= average_sensor_output(s, INV_IXM42XXX_SENSOR_ON_MASK_GYRO, 
		(BIT_GYRO_X_ST_EN + BIT_GYRO_Y_ST_EN + BIT_GYRO_Z_ST_EN), STG_ON);

	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	for(i = 0; i < 3; i++)
		STG_response[i] = INV_ABS(STG_ON[i] - STG_OFF[i]);

	/* Read ST_code */
	bank = 1;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_XG_ST_DATA_B1, 3, STG_code);
	bank = 0;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	/* If ST_Code=0 for any axis */
	if (STG_code[0] == 0 || STG_code[1] == 0 || STG_code[2] == 0) {
		/* compare the Self-Test response to the ST absolute limits */
		for (i = 0; i < 3; i++) {
			if (STG_response[i] < (MIN_ST_GYRO_DPS * gyro_sensitivity_1dps))
				*result = 0; /* fail */
		}
	/* If ST_Code!=0 for all axis */
	} else { 
		/* compare the Self-Test response to the factory OTP values */
		int fs_sel = ST_GYRO_FSR >> BIT_GYRO_CONFIG0_FS_SEL_POS;
		for (i = 0; i < 3; i++) {
			STG_OTP[i] = INV_ST_OTP_EQUATION(fs_sel, STG_code[i]);
			if (STG_OTP[i] == 0) {
				*result = 0; /* fail */
			} else {
				float ratio = ((float)STG_response[i]) / ((float)STG_OTP[i]);
				if ((ratio >= MAX_RATIO_GYRO) || (ratio <= MIN_RATIO_GYRO))
					*result = 0; /* fail */
			}
		}
	}

	/* stored the computed bias (checking GST and GOFFSET values) */
	for (i = 0; i < 3; i++) {
		if((INV_ABS(STG_OFF[i]) > (int32_t)(MAX_ST_GYRO_OFFSET_DPS * gyro_sensitivity_1dps)))
			*result = 0; /* fail */
		s->gyro_st_bias[i] = STG_OFF[i];
	}

	return status;
}

/*
 * Params: 
 *   - result: 1 if success, 0 if failure
 * Returns 0 if success, error code if failure
 */
static int run_accel_self_test(struct inv_ixm42xxx * s, int * result)
{
	int status = 0;
	uint8_t data;
	uint8_t bank;

	int32_t STA_OFF[3], STA_ON[3];
	uint32_t STA_response[3];

	uint8_t STA_code[3];
	uint32_t STA_OTP[3];

	int i = 0;
	int axis, axis_sign;
	
	uint32_t accel_sensitivity_1g = 32768 / reg_to_accel_fsr(ST_ACCEL_FSR);
	uint32_t gravity; 

	*result = 1;
	
	/* Set accel configuration */
	status |= inv_ixm42xxx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &data);
	data &= ~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
	data &= ~BIT_ACCEL_CONFIG0_ODR_MASK;
	data |= ST_ACCEL_FSR;
	data |= ST_ACCEL_ODR;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &data);
	
	status |= inv_ixm42xxx_read_reg(s, MPUREG_ACCEL_CONFIG1, 1, &data);
	data &= ~BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK;
	data |= ST_ACCEL_UI_FILT_ORD_IND;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_ACCEL_CONFIG1, 1, &data);
	
	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &data);
	data &= ~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= ST_ACCEL_UI_FILT_BW_IND;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &data);

	/* read average accel digital output for each axis and store them as ST_OFF_{x,y,z} in lsb x 1000 */
	status |= average_sensor_output(s, INV_IXM42XXX_SENSOR_ON_MASK_ACCEL, 0, STA_OFF);

	/* Enable self-test for each axis and read average gyro digital output 
	 * for each axis and store them as ST_ON_{x,y,z} in lsb x 1000 */
	status |= average_sensor_output(s, INV_IXM42XXX_SENSOR_ON_MASK_ACCEL, 
		(BIT_ACCEL_X_ST_EN + BIT_ACCEL_Y_ST_EN + BIT_ACCEL_Z_ST_EN + BIT_ST_REGULATOR_EN), STA_ON);

	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	/* outputs from this routine are in units of lsb and hence are dependent on the full-scale used on the DUT */
	for(i = 0; i < 3; i++)
		STA_response[i] = INV_ABS(STA_ON[i] - STA_OFF[i]);

	/* Read ST_code */
	bank = 2;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_XA_ST_DATA_B2, 3, STA_code);
	bank = 0;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);

	/* If ST_Code=0 for any axis */
	if (STA_code[0] == 0 || STA_code[1] == 0 || STA_code[2] == 0) {
		/* compare the Self-Test response to the ST absolute limits */
		for (i = 0; i < 3; i++) {
			if (   (STA_response[i] < ((MIN_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))
				|| (STA_response[i] > ((MAX_ST_ACCEL_MG * accel_sensitivity_1g) / 1000)))
				*result = 0; /* fail */
		}
	/* If ST_Code!=0 for all axis */
	} else { 
		int fs_sel = ST_ACCEL_FSR >> BIT_ACCEL_CONFIG0_FS_SEL_POS;
		for (i = 0; i < 3; i++) {
			STA_OTP[i] = INV_ST_OTP_EQUATION(fs_sel, STA_code[i]);
			if (STA_OTP[i] == 0) {
				*result = 0; /* fail */
			} else {
				float ratio = ((float)STA_response[i]) / ((float)STA_OTP[i]);
				if ((ratio >= MAX_RATIO_ACCEL) || (ratio <= MIN_RATIO_ACCEL))
					*result = 0; /* fail */
			}
		}
	}

	/* stored the computed offset */
	for(i = 0; i < 3; i++) {
		s->accel_st_bias[i] = STA_OFF[i];
	}

	/* assume the largest data axis shows +1 or -1 gee for gravity */
	axis = 0;
	axis_sign = 1;
	if (INV_ABS(s->accel_st_bias[1]) > INV_ABS(s->accel_st_bias[0]))
		axis = 1;
	if (INV_ABS(s->accel_st_bias[2]) > INV_ABS(s->accel_st_bias[axis]))
		axis = 2;
	if (s->accel_st_bias[axis] < 0)
		axis_sign = -1;

	gravity = accel_sensitivity_1g * axis_sign;
	s->accel_st_bias[axis] -= gravity;

	return status;
}

static int average_sensor_output(struct inv_ixm42xxx * s, int sensor, int self_test_config, int32_t average[3])
{
	int status = 0;
	int it = 0; /* Number of sample read */
	int sample_discarded = 0; /* Number of sample discarded */ 
	int timeout = 300; /* us */
	uint8_t data_reg; /* address of the register where to read the data */
	uint8_t self_test_config_reg; /* SELF_TEST_CONFIG register content */
	uint8_t pwr_mgmt_reg; /* PWR_MGMT register content */
	int32_t sum[3] = {0}; /* sum of all data read */

	if(sensor == INV_IXM42XXX_SENSOR_ON_MASK_GYRO) {
		data_reg = MPUREG_GYRO_DATA_X1_UI;

		/* Enable Gyro */
		status |= inv_ixm42xxx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt_reg);
		pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
		pwr_mgmt_reg |= (uint8_t)IXM42XXX_PWR_MGMT_0_GYRO_MODE_LN;
		status |= inv_ixm42xxx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt_reg);

		/* wait for 60ms to allow output to settle */
		inv_ixm42xxx_sleep_us(60*1000);
	
	} else if(sensor == INV_IXM42XXX_SENSOR_ON_MASK_ACCEL) {
		data_reg = MPUREG_ACCEL_DATA_X1_UI;
		
		/* Enable Accel */
		status |= inv_ixm42xxx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt_reg);
		pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
		pwr_mgmt_reg |= (uint8_t)IXM42XXX_PWR_MGMT_0_ACCEL_MODE_LN;
		status |= inv_ixm42xxx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt_reg);

		/* wait for 25ms to allow output to settle */
		inv_ixm42xxx_sleep_us(25*1000);
	}
	else
		return INV_ERROR_BAD_ARG; /* Invalid sensor provided */

	/* Apply ST config if required */
	if(self_test_config) {
		status |= inv_ixm42xxx_read_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &self_test_config_reg);
		self_test_config_reg |= self_test_config; 
		status |= inv_ixm42xxx_write_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &self_test_config_reg);

		if(sensor == INV_IXM42XXX_SENSOR_ON_MASK_GYRO)
			/* wait 200ms for the oscillation to stabilize */
			inv_ixm42xxx_sleep_us(200*1000);
		else 
			/* wait for 25ms to allow output to settle */
			inv_ixm42xxx_sleep_us(25*1000);
	}

	do {
		uint8_t int_status;
		status |= inv_ixm42xxx_read_reg(s, MPUREG_INT_STATUS, 1, &int_status);
		
		if (int_status & BIT_INT_STATUS_DRDY) {
			int16_t sensor_data[3] = {0}; 
			uint8_t sensor_data_reg[6]; /* sensor data registers content */

			/* Read data */
			status |= inv_ixm42xxx_read_reg(s, data_reg, 6, sensor_data_reg);
			
			if (s->endianess_data == IXM42XXX_INTF_CONFIG0_DATA_BIG_ENDIAN) {
				sensor_data[0] = (sensor_data_reg[0] << 8) | sensor_data_reg[1];
				sensor_data[1] = (sensor_data_reg[2] << 8) | sensor_data_reg[3];
				sensor_data[2] = (sensor_data_reg[4] << 8) | sensor_data_reg[5];
			} else { // LITTLE ENDIAN
				sensor_data[0] = (sensor_data_reg[1] << 8) | sensor_data_reg[0];
				sensor_data[1] = (sensor_data_reg[3] << 8) | sensor_data_reg[2];
				sensor_data[2] = (sensor_data_reg[5] << 8) | sensor_data_reg[4];
			}
			if ((sensor_data[0] != -32768) && (sensor_data[1] != -32768) && (sensor_data[2] != -32768)) {
				sum[0] += sensor_data[0];
				sum[1] += sensor_data[1];
				sum[2] += sensor_data[2];
			} else {
				sample_discarded++;
			}
			it++;
		}
		inv_ixm42xxx_sleep_us(1000);
		timeout--;
	} while((it < 200) && (timeout > 0));

	/* Disable Accel and Gyro */
	status |= inv_ixm42xxx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt_reg);
	pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
	pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	pwr_mgmt_reg |= (uint8_t)IXM42XXX_PWR_MGMT_0_GYRO_MODE_OFF;
	pwr_mgmt_reg |= (uint8_t)IXM42XXX_PWR_MGMT_0_ACCEL_MODE_OFF;
	status |= inv_ixm42xxx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt_reg);

	/* Disable self-test config if necessary */
	if(self_test_config) {
		self_test_config_reg &= ~self_test_config;
		status |= inv_ixm42xxx_write_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &self_test_config_reg);
	}

	/* Compute average value */
	it -= sample_discarded;
	average[0] = (sum[0] / it);
	average[1] = (sum[1] / it);
	average[2] = (sum[2] / it);
	
	return status;
}

static int save_settings(struct inv_ixm42xxx * s, struct recover_regs * saved_regs)
{
	int status = 0;

	status |= inv_ixm42xxx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &saved_regs->intf_config1);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &saved_regs->pwr_mgmt_0);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &saved_regs->accel_config0);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_ACCEL_CONFIG1, 1, &saved_regs->accel_config1);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &saved_regs->gyro_config0);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_CONFIG1, 1, &saved_regs->gyro_config1);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &saved_regs->accel_gyro_config0);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &saved_regs->fifo_config1);
	status |= inv_ixm42xxx_read_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &saved_regs->self_test_config);

	return status;
}

static int recover_settings(struct inv_ixm42xxx * s, const struct recover_regs * saved_regs)
{
	int status = 0;

	/* Set en_g{x/y/z}_st_d2a to 0 disable self-test for each axis */
	status |= inv_ixm42xxx_write_reg(s, MPUREG_SELF_TEST_CONFIG, 1, &saved_regs->self_test_config);
	/*Restore gyro_dec2_m2_ord, gyro_ui_filt_ord_ind and gyro_ui_filt_bw_ind to previous values.*/
	status |= inv_ixm42xxx_write_reg(s, MPUREG_INTF_CONFIG1, 1, &saved_regs->intf_config1);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &saved_regs->pwr_mgmt_0);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &saved_regs->accel_config0);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_ACCEL_CONFIG1, 1, &saved_regs->accel_config1);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &saved_regs->gyro_config0);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_CONFIG1, 1, &saved_regs->gyro_config1);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &saved_regs->fifo_config1);
	status |= inv_ixm42xxx_write_reg(s, MPUREG_GYRO_ACCEL_CONFIG0, 1, &saved_regs->accel_gyro_config0);
	/* wait 200ms for gyro output to settle */
	inv_ixm42xxx_sleep_us(200*1000);

	status |= inv_ixm42xxx_reset_fifo(s);

	return status;
}

static int set_user_offset_regs(struct inv_ixm42xxx * s, uint8_t sensor)
{
	uint8_t data[5];
	int16_t cur_bias;
	int status = 0;

	// Set memory bank 4
	status |= inv_ixm42xxx_set_reg_bank(s, 4);

	/* Set offset registers sensor */ 
	if (sensor == INV_IXM42XXX_SENSOR_ON_MASK_ACCEL) {
		/* 
		 * Invert sign for OFFSET and
		 * accel_st_bias is 2g coded 16 
		 * OFFUSER is 1g coded 12 (or 2g coded 12 for High FSR parts)
		 */
		status |= inv_ixm42xxx_read_reg(s, MPUREG_OFFSET_USER_4_B4, 1, &data[0]); // Fetch gyro_z_offuser[8-11]
		data[0] &= BIT_GYRO_Z_OFFUSER_MASK_HI;
		
		cur_bias = (int16_t)(-s->accel_st_bias[0] >> 3);
		cur_bias /= ACCEL_OFFUSER_MAX_MG/1000;
		data[0] |= (((cur_bias & 0x0F00) >> 8) << BIT_ACCEL_X_OFFUSER_POS_HI);
		data[1] = ((cur_bias & 0x00FF) << BIT_ACCEL_X_OFFUSER_POS_LO);
		cur_bias = (int16_t)(-s->accel_st_bias[1] >> 3);
		cur_bias /= ACCEL_OFFUSER_MAX_MG/1000;
		data[2] = ((cur_bias & 0x00FF) << BIT_ACCEL_Y_OFFUSER_POS_LO);
		data[3] = (((cur_bias & 0x0F00) >> 8) << BIT_ACCEL_Y_OFFUSER_POS_HI);
		cur_bias = (int16_t)(-s->accel_st_bias[2] >> 3);
		cur_bias /= ACCEL_OFFUSER_MAX_MG/1000;
		data[3] |= (((cur_bias & 0x0F00) >> 8) << BIT_ACCEL_Z_OFFUSER_POS_HI);
		data[4] = ((cur_bias & 0x00FF) << BIT_ACCEL_Z_OFFUSER_POS_LO);
		
		status |= inv_ixm42xxx_write_reg(s, MPUREG_OFFSET_USER_4_B4, 5, &data[0]);
		
	} else if (sensor == INV_IXM42XXX_SENSOR_ON_MASK_GYRO) {
		/* 
		 * Invert sign for OFFSET and
		 * gyro_st_bias is 250dps coded 16 
		 * OFFUSER is 64dps coded 12 (or 128dps coded 12 for High FSR parts)
		 */
		status |= inv_ixm42xxx_read_reg(s, MPUREG_OFFSET_USER_4_B4, 1, &data[4]); // Fetch accel_x_offuser[8-11]
		data[4] &= BIT_ACCEL_X_OFFUSER_MASK_HI;
		
		cur_bias = (int16_t)(-(s->gyro_st_bias[0]*250/GYRO_OFFUSER_MAX_DPS) >> 4);
		data[0] = ((cur_bias & 0x00FF) << BIT_GYRO_X_OFFUSER_POS_LO);
		data[1] = (((cur_bias & 0x0F00) >> 8) << BIT_GYRO_X_OFFUSER_POS_HI);
		cur_bias = (int16_t)(-(s->gyro_st_bias[1]*250/GYRO_OFFUSER_MAX_DPS) >> 4);
		data[1] |= (((cur_bias & 0x0F00) >> 8) << BIT_GYRO_Y_OFFUSER_POS_HI);
		data[2] = ((cur_bias & 0x00FF) << BIT_GYRO_Y_OFFUSER_POS_LO);
		cur_bias = (int16_t)(-(s->gyro_st_bias[2]*250/GYRO_OFFUSER_MAX_DPS) >> 4);
		data[3] = ((cur_bias & 0x00FF) << BIT_GYRO_Z_OFFUSER_POS_LO);
		data[4] |= (((cur_bias & 0x0F00) >> 8) << BIT_GYRO_Z_OFFUSER_POS_HI);
		
		status |= inv_ixm42xxx_write_reg(s, MPUREG_OFFSET_USER_0_B4, 5, &data[0]);
		
	}

	// Set memory bank 0
	status |= inv_ixm42xxx_set_reg_bank(s, 0);

	return status;
}

static int reg_to_accel_fsr(IXM42XXX_ACCEL_CONFIG0_FS_SEL_t reg)
{
	switch(reg) {
	case IXM42XXX_ACCEL_CONFIG0_FS_SEL_2g:   return 2;
	case IXM42XXX_ACCEL_CONFIG0_FS_SEL_4g:   return 4;
	case IXM42XXX_ACCEL_CONFIG0_FS_SEL_8g:   return 8;
	case IXM42XXX_ACCEL_CONFIG0_FS_SEL_16g:  return 16;
#if defined(ICM42686)
	case IXM42XXX_ACCEL_CONFIG0_FS_SEL_32g:  return 32;
#endif
	default:                                 return -1;
	}
}
static int reg_to_gyro_fsr(IXM42XXX_GYRO_CONFIG0_FS_SEL_t reg)
{
	switch(reg) {
#if !defined(ICM42686)
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_16dps:   return 16;
#endif
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_31dps:   return 31;
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_62dps:   return 62;
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_125dps:  return 125;
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_250dps:  return 250;
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_500dps:  return 500;
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_1000dps: return 1000;
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_2000dps: return 2000;
#if defined(ICM42686)
	case IXM42XXX_GYRO_CONFIG0_FS_SEL_4000dps: return 4000;
#endif
	default:                                   return -1;
	}
}