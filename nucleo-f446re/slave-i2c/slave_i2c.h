/*
 * slave i2c handler defines
 */

#pragma once
#include <stdint.h>

typedef struct {
	void	*	(*init)(void);
	uint8_t		(*send)(void *);
	void		(*recv)(void *, uint8_t db);
	void		(*start)(void *, uint8_t rw);
	void		(*stop)(void *, uint8_t err);
	uint16_t	addr;
	uint8_t		mode; 	/* IDLE, READING, WRITING */
	void		*state;
} i2c_handler_t;

#define HANDLE_MODE_IDLE		0
#define HANDLE_MODE_SENDING		1
#define HANDLE_MODE_RECEIVING	2
