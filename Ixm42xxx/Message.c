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

#include "Message.h"

#include <stdio.h>
#include <stdlib.h>

static int               msg_level;
static inv_msg_printer_t msg_printer;

void inv_msg_printer_default(int level, const char * str, va_list ap)
{
#if !defined(__ICCARM__)
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		" [E] ", // INV_MSG_LEVEL_ERROR
		" [W] ", // INV_MSG_LEVEL_WARNING
		" [I] ", // INV_MSG_LEVEL_INFO
		" [V] ", // INV_MSG_LEVEL_VERBOSE
		" [D] ", // INV_MSG_LEVEL_DEBUG
	};

	fprintf(stderr, "%s", s[level]);	
	vfprintf(stderr, str, ap);
	fprintf(stderr, "\n");
#else
	(void)level, (void)str, (void)ap;
#endif
}

void inv_msg_setup(int level, inv_msg_printer_t printer)
{
	msg_level   = level;
	if (level < INV_MSG_LEVEL_OFF)
		msg_level = INV_MSG_LEVEL_OFF;
	else if (level > INV_MSG_LEVEL_MAX)
		msg_level = INV_MSG_LEVEL_MAX;
	msg_printer = printer;
}

void inv_msg(int level, const char * str, ...)
{
	if(level && level <= msg_level && msg_printer) {
		va_list ap;
		va_start(ap, str);
		msg_printer(level, str, ap);
		va_end(ap);
	}
}

int inv_msg_get_level(void)
{
	return msg_level;
}
