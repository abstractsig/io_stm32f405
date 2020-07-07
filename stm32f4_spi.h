/*
 *
 * stm32f4 spi
 *
 * NB: included by io_cpu.h
 *
 */
#ifndef stm32f4_spi_H_
#define stm32f4_spi_H_

#define IO_SINGLE_BIND_UNCOUNTED_SOCKET_STRUCT_MEMBERS \
	IO_SOCKET_STRUCT_MEMBERS \
	io_encoding_pipe_t *transmit_pipe; \
	io_event_t *signal_transmit_available; \
	io_event_t *signal_receive_data_available; \
	/**/

//
// operated in raw binary mode, only supports one
// inner binding
//
typedef struct PACK_STRUCTURE {
	IO_SINGLE_BIND_UNCOUNTED_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;

	stm32f4_io_dma_channel_t tx_dma_channel;
	stm32f4_io_dma_channel_t rx_dma_channel;

	io_cpu_clock_pointer_t peripheral_bus_clock;
	SPI_TypeDef *spi_registers;
	
	stm32f4_io_pin_t mosi_pin;
	stm32f4_io_pin_t miso_pin;
	stm32f4_io_pin_t sck_pin;
	stm32f4_io_pin_t ss_pin;
	
	uint32_t maximum_sck_frequency;
	
} stm32f4_spi_socket_t;

extern EVENT_DATA io_socket_implementation_t stm32f4_dma_spi_socket_implementation;


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------

bool
stm32f4_spi_socket_output_next_message (stm32f4_spi_socket_t *this) {
	io_encoding_t *next;
	if (
			io_socket_is_open ((io_socket_t*) this)
		&& io_encoding_pipe_peek (this->transmit_pipe,&next)
	) {
		const uint8_t *begin,*end;
		io_encoding_get_content (next,&begin,&end);		
		io_dma_transfer_to_peripheral (
			(io_dma_channel_t*) &this->tx_dma_channel,begin,end - begin
		);
		return true;
	} else {
		return false;
	}
}

void
stm32f4_spi_tx_dma_complete (io_event_t *ev) {
	
}

void
stm32f4_spi_tx_dma_error (io_event_t *ev) {
	
}

void
stm32f4_spi_rx_dma_complete (io_event_t *ev) {
	
}

void
stm32f4_spi_rx_dma_error (io_event_t *ev) {
	
}

static io_socket_t*
stm32f4_spi_initialise (
	io_socket_t *socket,io_t *io,io_settings_t const *C
) {
	stm32f4_spi_socket_t *this = (stm32f4_spi_socket_t*) socket;

	initialise_io_socket (socket,io);
	this->encoding = C->encoding;
	
	initialise_io_event (
		&this->tx_dma_channel.complete,stm32f4_spi_tx_dma_complete,this
	);

	initialise_io_event (
		&this->tx_dma_channel.error,stm32f4_spi_tx_dma_error,this
	);

	initialise_io_event (
		&this->rx_dma_channel.complete,stm32f4_spi_rx_dma_complete,this
	);

	initialise_io_event (
		&this->rx_dma_channel.error,stm32f4_spi_rx_dma_error,this
	);


	return socket;
}

static bool
stm32f4_spi_open (io_socket_t *socket,io_socket_open_flag_t flag) {
	stm32f4_spi_socket_t *this = (stm32f4_spi_socket_t*) socket;

	if (io_cpu_clock_start (this->io,this->peripheral_bus_clock)) {
		if ((this->spi_registers->CR1 & SPI_CR1_SPE) == 0) {
			float64_t freq = io_cpu_clock_get_current_frequency (
				this->peripheral_bus_clock
			);
			uint32_t pclock = (freq);
			uint32_t div = 0;
			
			while ((pclock >> (div + 1)) > this->maximum_sck_frequency) {
				div ++;
				if (div > 7) {
					return false;
				}
			}

			io_set_pin_to_output (io_socket_io (this),this->ss_pin.io);
			io_set_pin_to_alternate (io_socket_io (this),this->miso_pin.io);
			io_set_pin_to_alternate (io_socket_io (this),this->mosi_pin.io);
			io_set_pin_to_alternate (io_socket_io (this),this->sck_pin.io);
			
			GPIOB->BSRRL |= GPIO_Pin_6;

			io_dma_controller_start_controller (this->tx_dma_channel.controller);
			io_dma_controller_start_controller (this->rx_dma_channel.controller);
			
			this->spi_registers->CR1 = (
					SPI_CR1_SPE
				|	(div << 3)
			);
		}
	}
	
	return true;
}

bool
stm32f4_spi_is_closed (io_socket_t const *socket) {
	stm32f4_spi_socket_t *this = (stm32f4_spi_socket_t*) socket;
	return ((this->spi_registers->CR1 & SPI_CR1_SPE) == 0);
}

static io_encoding_t*
stm32f4_spi_new_message (io_socket_t *socket) {
	stm32f4_spi_socket_t *this = (stm32f4_spi_socket_t*) socket;
	return reference_io_encoding (
		new_io_encoding (
			this->encoding,io_get_byte_memory(this->io)
		)
	);
}

EVENT_DATA io_socket_implementation_t
stm32f4_dma_spi_socket_implementation = {
	SPECIALISE_IO_SOCKET_IMPLEMENTATION (
		&io_physical_socket_implementation
	)
	.initialise = stm32f4_spi_initialise,
	.open = stm32f4_spi_open,
	.is_closed = stm32f4_spi_is_closed,
	.new_message = stm32f4_spi_new_message,
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

