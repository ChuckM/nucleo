#include "ring.h"


/*
 * Write bytes from the interface.
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
