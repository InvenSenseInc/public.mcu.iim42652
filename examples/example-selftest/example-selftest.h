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

#ifndef _EXAMPLE_SELFTEST_H_
#define _EXAMPLE_SELFTEST_H_

#include <stdint.h>
#include "Ixm42xxxTransport.h"
#include "Ixm42xxxDefs.h"
#include "Ixm42xxxDriver_HL.h"
#include "Ixm42xxxSelfTest.h"

/**
 * \brief This function is in charge of reseting and initializing Ixm42xxx device. It should
 * be succesfully executed before any access to Ixm42xxx device.
 *
 */
int SetupInvDevice(struct inv_ixm42xxx_serif * icm_serif);

/*!
 * \brief Run Self Test on Invensense device
 */
void RunSelfTest(void);

/*!
 * \brief Get Bias values calculated from selftest
 */
void GetBias(void);


#endif /* !_EXAMPLE_SELFTEST_H_ */
