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

#include "example-selftest.h"

/* InvenSense drivers and utils */
#include "Message.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the ixm42xxx object */
static struct inv_ixm42xxx icm_driver;


/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */
 
int SetupInvDevice(struct inv_ixm42xxx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Ixm42xxx");
	
	rc = inv_ixm42xxx_init(&icm_driver, icm_serif, NULL);
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

	return rc;
}

void RunSelfTest(void)
{
	int rc = 0, st_result = 0;

	rc = inv_ixm42xxx_run_selftest(&icm_driver, &st_result);

	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while running selftest");
	} else {
		/* Check for GYR success (1 << 0) and ACC success (1 << 1) */
		if (st_result & 0x1)
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest PASS");
		else
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest FAIL");
		
		if (st_result & 0x2)
			INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest PASS");
		else
			INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest FAIL");
	} 
}

void GetBias(void)
{
	int rc = 0;
	int raw_bias[6];

	/* Get Low Noise / Low Power bias computed by self-tests scaled by 2^16 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting ST bias");
	rc |= inv_ixm42xxx_get_st_bias(&icm_driver, raw_bias);
	if (rc < 0)
		INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while getting ST bias");
	
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (dps): x=%f, y=%f, z=%f",
			(float)(raw_bias[0]) / (float)(1 << 16), (float)(raw_bias[1]) / (float)(1 << 16), (float)(raw_bias[2]) / (float)(1 << 16));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (g): x=%f, y=%f, z=%f",
			(float)(raw_bias[0 + 3] / (float)(1 << 16)), (float)(raw_bias[1 + 3] / (float)(1 << 16)), (float)(raw_bias[2 + 3] / (float)(1 << 16)));
}
