/*
 * Emulated Display Handler -- Definitions
 *
 * BSD 2-Clause License
 * 
 * Copyright (C) 2013-2020 Chuck McManis <cmcmanis@mcmanis.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <stdint.h>
#include "slave_i2c.h"

struct emd_state_t {
	uint8_t		*display;		/* Display contents */
	uint32_t	send_errors;	/* Errors sending to master */
	uint32_t	recv_errors;	/* Errors receiving from master */
	uint32_t	ops_count;		/* Total number of operations start/finish */
	uint32_t	ops_errors;		/* Operations that errored out */
	uint8_t		state_changed;	/* Indication of a state change */
	uint8_t		cmd;			/* Command register (Write Only) */
	uint8_t		row;			/* Current row (Read / Write) */
	uint8_t		row_max;		/* Maximum row (Read Only) */
	uint8_t		col;			/* Current column (Read / Write) */
	uint8_t		col_max;		/* Maximum column (Read Only) */
	uint8_t		fg;				/* Display Foreground color (Read / Write) */
	uint8_t		bg;				/* Display Background Color (Read / Write) */
	uint8_t		cur_mode;		/* Internal (don't change) */
	uint8_t		cur_reg;		/* Internal (don't change) */
	uint8_t		new_reg_value;	/* Internal (don't change) */
};

enum emd_reg_t {                     
	EMD_REG_CMD = 0,		/* 0 */
	EMD_REG_ROW,			/* 1 */
	EMD_REG_MAX_ROW,		/* 2 */
	EMD_REG_COL,			/* 3 */
	EMD_REG_MAX_COL,		/* 4 */
	EMD_REG_FG,				/* 5 */
	EMD_REG_BG,				/* 6 */
    EMD_REG_DISPLAY			/* 7 	(Always last) */
}; 

enum emd_cmd_t {
	EMD_CMD_NOP = 0,		/* do nothing command */
	EMD_CMD_CLEAR = 1,		/* Clear the display */
	EMD_CMD_CLEAR_LINE = 2,	/* Clear the current line in ROW register */
	EMD_CMD_INIT = 3,		/* Re-initialize display */
	EMD_CMD_HOME = 4		/* Cursor to home (1, 1) position */
};

#define EMD_COLOR_BLACK		0x0
#define EMD_COLOR_RED		0x1
#define EMD_COLOR_GREEN		0x2
#define EMD_COLOR_CYAN		0x3
#define EMD_COLOR_BLUE		0x4
#define EMD_COLOR_YELLOW	0x5
#define EMD_COLOR_MAGENTA	0x6
#define EMD_COLOR_WHITE		0x7

/* Prototypes */

extern const i2c_handler_t emd_handler;

/* Allocate and initialize the handler's state structure */
struct emd_state_t *emd_init(void);

/* Return the string representing the register */
char *emd_reg_name(enum emd_reg_t reg);
