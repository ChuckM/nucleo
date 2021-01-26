/*
 * Ring - A simple ring buffer implementation.
 *
 * As its name implies a ring buffer "appears" to be an endless
 * buffer but really it just wraps around on itself. The two key APIs
 * here are _get and _put. Get returns the next item from the buffer
 * or an error value if the buffer is "empty". Put adds an item to the
 * list, potentially discarding the oldest unseen item in the list if
 * the buffer is "full."
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

#include "ring.h"


/*
 * Add bytes to the buffer.
 *
 * If the ringbuffer is "full", move both the next and current
 * pointers (results in dropping the oldest byte in the buffer)
 * and note the error.
 */
void
ringbuffer_put(ringbuffer_t *rb, uint8_t data)
{
	/* store the byte */
	*(rb->buffer + rb->next) = data;

	/* point to where the next byte will go */
	rb->next = (rb->next + 1) % rb->size;
	/* if we're 'full' move the current pointer (loses oldest byte) */
	if (rb->next == rb->cur) {
		rb->cur = (rb->cur + 1) % rb->size;
		rb->err++;	/* note the error */
	}
}

/*
 * Return the next available byte from the ringbuffer.
 * If there are no bytes in the ring buffer to return,
 * return the error byte and increment the error
 * count.
 */
uint8_t
ringbuffer_get(ringbuffer_t *rb)
{
	uint8_t	result;

	if (rb->cur == rb->next) {
		rb->err++;
		return RINGBUFFER_EMPTY_BYTE;
	}
	result = *(rb->buffer + rb->cur);
	rb->cur = (rb->cur + 1) % rb->size;
	return result;
}

/*
 * Return the number of bytes currently held in the
 * ring buffer.
 */
uint16_t
ringbuffer_holding(ringbuffer_t *rb)
{
	return ((rb->next < rb->cur) ? (rb->next + rb->size) - rb->cur :
									rb->next - rb->cur);
}

/*
 * Return the number of spaces available in the ring buffer 
 */
uint16_t
ringbuffer_available(ringbuffer_t *rb)
{
	return ( rb->size - 1 -
				((rb->next < rb->cur) ? (rb->next + rb->size) - rb->cur :
									rb->next - rb->cur));
}

/*
 * Return the total bytes held by the ring buffer.
 */
uint16_t
ringbuffer_size(ringbuffer_t *rb)
{
	return (rb->size);
}

/*
 * Flush (or empty) a ring buffer
 */
void
ringbuffer_flush(ringbuffer_t *rb)
{
	rb->next = rb->cur = rb->err = 0;
}
