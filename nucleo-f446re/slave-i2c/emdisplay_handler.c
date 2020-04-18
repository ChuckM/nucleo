/*
 * Emulated Display Handler -- Emulate a 16 x 2 Character display
 *
 * This I2C handler emulates a simple 16 x 2 character display.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "slave_i2c.h"
#include "emdisplay_handler.h"

/* private command state machine */
enum emd_cmd_state_t {
	EMD_STATE_IDLE = 0,
	EMD_STATE_SEND_REG,
	EMD_STATE_RECV_REG,
	EMD_STATE_RECV_REG_VALUE,
	EMD_STATE_INVALID_REG,
};

static void emd_start(void *state, uint8_t rw);
static uint8_t emd_send(void *state);
static void emd_recv(void *state, uint8_t db);
static void emd_finish(void *state, uint8_t err);

static const char *__emd_init_msg =
/*	0123456789ABCEF */
   "Emulated       "
   "   Display V1.0";

/*
 * Prototype handler structure for the emulated display device.
 */
const i2c_handler_t emd_handler = {
	.send = emd_send,
	.recv = emd_recv,
	.start = emd_start,
	.stop = emd_finish,
	.addr = 0,
	.mode = HANDLE_MODE_IDLE,
	.state = NULL
};

#define EMD_MAX_ROWS	2
#define EMD_MAX_COLS	16
#define EMD_DISPLAY_SIZE	(EMD_MAX_ROWS * EMD_MAX_COLS)

/*
 * Allocate and initialize the state structure for our emulated
 * display.
 */
struct emd_state_t *
emd_init(void)
{
	struct emd_state_t	*state;

	state = calloc(1, sizeof(struct emd_state_t));
	if (state == NULL) {
		return NULL;
	}
	state->display = calloc(1, EMD_MAX_ROWS * EMD_MAX_COLS);
	if (state->display == NULL) {
		free(state);
		return NULL;
	}
	snprintf((char *)state->display, EMD_DISPLAY_SIZE, "%s", 
								__emd_init_msg);
	state->row = 0;
	state->row_max = EMD_MAX_ROWS;
	state->col = 0;
	state->col_max = EMD_MAX_COLS;
	state->fg = EMD_COLOR_YELLOW;	/* for fun */
	state->bg = EMD_COLOR_BLUE;
	state->cmd = 0;
	state->cur_mode = EMD_STATE_IDLE;
	state->cur_reg = EMD_REG_CMD;
	state->state_changed = 1;
	state->new_reg_value = 0;
	return state;
}

char *
emd_reg_name(enum emd_reg_t reg)
{
	switch (reg) {
		case EMD_REG_CMD:
			return "Cmd";
		case EMD_REG_ROW:
			return "Row";
		case EMD_REG_MAX_ROW:
			return "RowMax";
		case EMD_REG_COL:
			return "Col";
		case EMD_REG_MAX_COL:
			return "ColMax";
		case EMD_REG_FG:
			return "FgCol";
		case EMD_REG_BG:
			return "BgCol";
		case EMD_REG_DISPLAY:
			return "Display";
		default:
			return "XXX";
	}
}

/*
 * Start a transaction
 */
static void
emd_start(void *s, uint8_t rw)
{
	struct emd_state_t *state = (struct emd_state_t *) s;

	if (rw) {
		state->cur_mode = EMD_STATE_SEND_REG;
	} else {
		state->cur_mode = EMD_STATE_RECV_REG;
	}
}

/*
 * Receiving bytes
 *
 * If we were idle, this is the register to read or write.
 * If we had a register already, we remember the value sent
 * Once we finish we'll actually change the register.
 *
 * The exception is the display. We immediately write into the
 * display buffer bytes as they come in.
 *
 * We record errors if we try to write read only or invalid registers.
 */
static void
emd_recv(void *s, uint8_t db)
{
	struct emd_state_t *state = (struct emd_state_t *) s;

	switch (state->cur_mode) {
		case EMD_STATE_RECV_REG:
			if (db <= (uint8_t)(EMD_REG_DISPLAY)) {
				state->cur_reg = (enum emd_reg_t) db;	/* set the register */
				state->cur_mode = EMD_STATE_RECV_REG_VALUE;
			} else {
				state->recv_errors++;
				state->cur_mode = EMD_STATE_INVALID_REG;
			}
			return;
		case EMD_STATE_INVALID_REG:
			state->recv_errors++;
			return;
		case EMD_STATE_RECV_REG_VALUE:
			switch (state->cur_reg) {
				case EMD_REG_MAX_ROW:
				case EMD_REG_MAX_COL:
					state-> recv_errors++;
					break;
				case EMD_REG_DISPLAY:
					*(state->display + state->row * 16 + state->col) = db;
					state->col = (state->col + 1) % state->col_max;
					if (state->col == 0) {
						state->row = (state->row + 1) % state->row_max;
					}
					break;
				default:
					state-> new_reg_value = db;
					break;
			}
			return;
	}
}

/*
 * Sending bytes
 *
 * We send the contents of the current register, if there is a valid register
 * set, or we send 0xff and record an error.
 *
 * If we are reading from the display it automatically updates the row and
 * column registers.
 */
static uint8_t
emd_send(void *s)
{
	struct emd_state_t *state = (struct emd_state_t *) s;
	uint8_t db;

	switch(state->cur_mode) {
		default:
		case EMD_STATE_INVALID_REG:
			state->send_errors++;
			return 0xff;
		case EMD_STATE_SEND_REG:
			switch (state->cur_reg) {
				case EMD_REG_DISPLAY:
					db = *(state->display + state->row * state->row_max +
											state->col);
					state->col = (state->col + 1) % state->col_max;
					if (state->col == 0) {
						state->row = (state->row + 1) % state->row_max;
					}
					return db;
				case EMD_REG_CMD:
					return 0x00;
				case EMD_REG_ROW:
					return state->row;
				case EMD_REG_MAX_ROW:
					return state->row_max;
				case EMD_REG_COL:
					return state->col;
				case EMD_REG_MAX_COL:
					return state->col_max;
				case EMD_REG_FG:
					return state->fg;
				case EMD_REG_BG:
					return state->bg;
				default:
					state->send_errors++;
					return 0xff;
			}
	}
	state->send_errors++;
	return 0xff;
}

/*
 * This function called once the transaction is finished.
 *
 * Check for errors, update state, and return.
 */
static void
emd_finish(void *s, uint8_t err)
{
	struct emd_state_t *state = (struct emd_state_t *) s;

	state->ops_count++;

	/*
 	 * First, deal with i2c errors.
 	 *
 	 * If we ended on a sour note (err != 0), we go back to idle, reset the
 	 * current register to the command register, and update there ops error
 	 * count.
 	 */
	if (err) {
		state->cur_mode = EMD_STATE_IDLE;
		state->cur_reg = EMD_REG_CMD;
		state->ops_errors++;
		return;
	}

	/*
	 * Okay, it was a valid i2c transaction from start to finish.
	 *
	 * Check to see if we just sent the current value of a register.
	 * If it was any register other than the DISPLAY register then 
	 * we're done. But if we were sending data from the emulated display,
	 * row and column will be updated. So indicate a state change.
	 *
	 * Leave the current register alone.
	 */
	if (state->cur_mode == EMD_STATE_SEND_REG) {
		if (state->cur_reg == EMD_REG_DISPLAY) {
			state->state_changed++;
		}
		state->cur_mode = EMD_STATE_IDLE;
		return;
	}

	/* 
	 * Next, deal emulator usage errors, specifically did the master
	 * try to read or write an invalid register? Did the handler get into
	 * some weird state? The only valid state that it could be in at  this
	 * point is EMD_STATE_RECV_REG, we'll handle that last.
	 */
	if (state->cur_mode != EMD_STATE_RECV_REG) {
		state->ops_errors++;
		state->cur_mode = EMD_STATE_IDLE;
		return;
	}

	/*
 	 * Finally, the last thing it can be is that we've received a new
 	 * register value.
 	 */
	switch (state->cur_reg) {
		/* these don't change state */
		case EMD_REG_MAX_ROW:
		case EMD_REG_MAX_COL:
		default:
			return;
		/* These updates happened while receiving data for the display */
		case EMD_REG_DISPLAY:
			break;
		case EMD_REG_FG:
			state->fg = state->new_reg_value & 0x7;
			break;
		case EMD_REG_BG:
			state->bg = state->new_reg_value & 0x7;
			break;
		case EMD_REG_ROW:
			state->row = state->new_reg_value % state->row_max;
			break;
		case EMD_REG_COL:
			state->col = state->new_reg_value % state->col_max;
			break;
		case EMD_REG_CMD:
			switch ((enum emd_cmd_state_t) state->new_reg_value) {
				default:
					state->ops_errors++;
					break;
				case EMD_CMD_CLEAR:
					memset(state->display, 
							' ', state->row_max * state->col_max);
					break;
				case EMD_CMD_CLEAR_LINE:
					memset(state->display + state->row * state->col_max, ' ',
						state->col_max);
					break;
				case EMD_CMD_HOME:
					state->row = state->col = 0;
					break;
				case EMD_CMD_INIT:
					memset(state->display, 
							' ', state->row_max * state->col_max);
					snprintf((char *)state->display, EMD_DISPLAY_SIZE, "%s", 
															__emd_init_msg);
					break;
			}
	}
	state->cmd = 0;
	state->state_changed++;
}
