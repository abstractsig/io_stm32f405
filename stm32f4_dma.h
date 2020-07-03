/*
 *
 * stm32f4 dma
 *
 * NB: included by io_cpu.h
 *
 */
#ifndef stm32f4_dma_H_
#define stm32f4_dma_H_

//
//
typedef struct stm32f4_io_dma_controller {
	IO_DMA_CONTROLLER_STRUCT_MEMBERS

	io_cpu_clock_pointer_t peripheral_bus_clock;
	DMA_TypeDef *cpu_registers;
	
} stm32f4_io_dma_controller_t;

extern EVENT_DATA io_dma_controller_implementation_t stm32f4_io_dma_controller_implementation;

typedef struct PACK_STRUCTURE stm32f4_io_dma_channel {
	IO_DMA_CHANNEL_STRUCT_MEMBERS

	io_dma_controller_t *controller;	// need like a clock pointer
	DMA_Stream_TypeDef *stream_registers;
	uint32_t channel_number;
	int32_t interrupt_number;
	uint32_t peripheral_register;
	
} stm32f4_io_dma_channel_t;

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------

//
// controller
//
bool
stm32f4_io_dma_controller_start (io_dma_controller_t *dmac) {
	stm32f4_io_dma_controller_t *this = (stm32f4_io_dma_controller_t*) dmac;
	if (io_cpu_clock_start (this->io,this->peripheral_bus_clock)) {
		return true;
	} else {
		return false;
	}
}

bool
stm32f4_io_dma_controller_start_transfer (io_dma_controller_t *dmac,io_dma_channel_t *channel) {
	return true;
}

EVENT_DATA io_dma_controller_implementation_t
stm32f4_io_dma_controller_implementation = {
	SPECIALISE_IO_DMA_CONTROLLER_IMPLEMENTATION(NULL)
	.start_controller = stm32f4_io_dma_controller_start,
	.start_transfer = stm32f4_io_dma_controller_start_transfer,
};

uint32_t
stm32f4_io_dma_channel_get_interrupt_status (stm32f4_io_dma_channel_t *this) {
	DMA_TypeDef *dmac = ((stm32f4_io_dma_controller_t*) this->controller)->cpu_registers;
	uint32_t status = 0;
	
	switch (this->channel_number) {
		case 0:
			status = dmac->LISR & 0x3d;
		break;
		
		case 1:
			status = (dmac->LISR >> 6) & 0x3d;
		break;
		
		case 2:
			status = (dmac->LISR >> 16) & 0x3d;
		break;
		
		case 3:
			status = (dmac->LISR >> 22) & 0x3d;
		break;

		case 4:
			status = dmac->HISR & 0x3d;
		break;
		
		case 5:
			status = (dmac->HISR >> 6) & 0x3d;
		break;
		
		case 6:
			status = (dmac->HISR >> 16) & 0x3d;
		break;
		
		case 7:
			status = (dmac->HISR >> 22) & 0x3d;
		break;

	}
	
	return status;
}

void
stm32f4_io_dma_channel_clear_interrupt_status (stm32f4_io_dma_channel_t *this,uint32_t status) {
	DMA_TypeDef *dmac = ((stm32f4_io_dma_controller_t*) this->controller)->cpu_registers;
	status &= 0x3d;
	
	switch (this->channel_number) {
		case 0:
			dmac->LISR |= status;
		break;
		
		case 1:
			dmac->LISR |= (status << 6);
		break;
		
		case 2:
			dmac->LISR |= (status << 16);
		break;
		
		case 3:
			dmac->LISR |= (status << 22);
		break;

		case 4:
			dmac->HISR |= status;
		break;
		
		case 5:
			dmac->HISR |= (status << 6);
		break;
		
		case 6:
			dmac->HISR |= (status << 16);
		break;
		
		case 7:
			dmac->HISR |= (status << 22);
		break;
	}
}

//
// channel
//
void
stm32f4_io_dma_channel_interrupt (void *user_value) {
	stm32f4_io_dma_channel_t *this = user_value;
	uint32_t status;
	
	status = stm32f4_io_dma_channel_get_interrupt_status (this);
	stm32f4_io_dma_channel_clear_interrupt_status (this,status);
	
	if (status & 0x0d) {
		io_enqueue_event (this->controller->io,&this->error);
	} else if (status & 0x20) {
		io_enqueue_event (this->controller->io,&this->complete);
	}
}

void
stm32f4_io_dma_channel_initialise (io_dma_channel_t *channel) {
	stm32f4_io_dma_channel_t *this = (stm32f4_io_dma_channel_t*) channel;

	register_io_interrupt_handler (
		this->controller->io,this->interrupt_number,stm32f4_io_dma_channel_interrupt,this
	);
}

static void
stm32f4_io_dma_channel_transfer_to_peripheral (
	io_dma_channel_t *channel,void const *src,uint32_t size
) {
	stm32f4_io_dma_channel_t *this = (stm32f4_io_dma_channel_t*) channel;
	DMA_Stream_TypeDef *reg = this->stream_registers;
	
	if ((reg->CR * 0x1) == 0) {
		uint32_t config = reg->CR & 0xf0100000;
		
		reg->NDTR = size;
		reg->PAR = this->peripheral_register;
		reg->M0AR = (uint32_t) src;
		config = (
				(this->channel_number << 25)
			|	(0 << 23)	// single transfer memory burst
			|	(0 << 21)	// single transfer peripheral burst
			|	(0 << 19)	// current target not used
			|	(0 << 18)	// single buffer
			|	(0 << 16)	// priority
			|	(0 << 15)	// peripheral increment no meaning, PINC = 0
			|	(0 << 13)	// memory inc size = 1 byte
			|	(0 << 11)	// peripheral inc size = 1 byte
			|	(1 << 10)	// increment memory address
			|	(0 <<  9)	// fixed peripheral address
			|	(0 <<  8)	// no circular
			|	(2 <<  6)	// memory to peripheral
			|	(1 <<  5)	// peripheral is flow controller
			|	(1 <<  4)	// TCIE
			|	(0 <<  3)	// HTIE
			|	(1 <<  2)	// TEIE
			|	(1 <<  1)	// DMEIE
			|	(1 <<  0)	// enable
		);
		
		reg->CR |= config;
	} else {
		// something bad?
	}
}

static void
stm32f4_io_dma_channel_transfer_from_peripheral (
	io_dma_channel_t *channel,void *dest,uint32_t length
) {

}

static void
stm32f4_io_dma_channel_transfer_complete (io_t *io,io_dma_channel_t *channel) {
/*
	cc2652_io_dma_channel_t *cc_channel = (cc2652_io_dma_channel_t*) channel;
	volatile uint32_t mask = (1 << cc2652_io_dma_channel_number(cc_channel));
	uDMAIntClear(UDMA0_BASE,mask);
*/
	io_enqueue_event (io,&channel->complete);
}

EVENT_DATA io_dma_channel_implementation_t 
stm32f4_dma_channel_implementation = {
	SPECIALISE_IO_DMA_CHANNEL_IMPLEMENTATION (
		&dma_channel_implementation
	)
	.initialise = stm32f4_io_dma_channel_initialise,
	.transfer_from_peripheral = stm32f4_io_dma_channel_transfer_from_peripheral,
	.transfer_to_peripheral = stm32f4_io_dma_channel_transfer_to_peripheral,
	.transfer_complete = stm32f4_io_dma_channel_transfer_complete,
};

#endif /* IMPLEMENT_IO_CPU */
#endif
/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2020 Gregor Bruce
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/

