/*
 * __________________________________________________________________
 *
 * Copyright (C) [2021] by InvenSense, Inc.
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

/** @defgroup ErrorHelper Error Helper
 *	@brief    Helper functions realted to error code
 *  @ingroup  EmbUtils
 *	@{
 */

#ifndef _INV_ERROR_HELPER_H_
#define _INV_ERROR_HELPER_H_

#//include "Invn/InvExport.h"
#include "InvError.h"
#define INV_EXPORT

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Returns string describing error number
 *  @sa enum inv_error
 */
const char INV_EXPORT * inv_error_str(int error);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ERROR_HELPER_H_ */

/** @} */
