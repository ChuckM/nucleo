/*
 * Echo handler
 *
 * This simple handler stores bytes that are written to the device
 * and returns them when the device is read.
 *
 * Note, this is dynamically allocated because that makes it "easy"
 * to have two echo responders, but mostly it also just makes the
 * code more flexible.
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

struct echo_state_t {
	ringbuffer_t	*rb;
	uint32_t		bytes_sent, bytes_recv;
	uint32_t		send_errs, recv_errs;
	uint32_t		send_count, receive_count, err_count;
	uint32_t		total;
	uint32_t		processed;
};

/* Prototypes */
struct echo_state_t *echo_init(uint16_t buf_size);
extern const i2c_handler_t echo_handler;

