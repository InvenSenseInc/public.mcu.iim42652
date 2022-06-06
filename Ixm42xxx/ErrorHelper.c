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
 
#include "ErrorHelper.h"

const char * inv_error_str(int error)
{
	switch(error) {
	case INV_ERROR_SUCCESS:      return "Success";
	case INV_ERROR:              return "Unspecified error";
	case INV_ERROR_NIMPL:        return "Not implemented";
	case INV_ERROR_TRANSPORT:    return "Transport error";
	case INV_ERROR_TIMEOUT:      return "Timeout, action did not complete in time";
	case INV_ERROR_SIZE:         return "Wrong size error";
	case INV_ERROR_OS:           return "Operating system failure";
	case INV_ERROR_IO:           return "Input/Output error";
	case INV_ERROR_MEM: 		 return "Bad allocation";
	case INV_ERROR_HW:           return "Hardware error";
	case INV_ERROR_BAD_ARG:      return "Invalid arguments";
	case INV_ERROR_UNEXPECTED:   return "Unexpected error";
	case INV_ERROR_FILE:         return "Invalid file format";
	case INV_ERROR_PATH:         return "Invalid file path";
	case INV_ERROR_IMAGE_TYPE:   return "Unknown image type";
	case INV_ERROR_WATCHDOG:     return "Watchdog error";
	default:                     return "Unknown error";
	}
}