/*
 *
 * stm32f4 uart
 *
 * NB: included by io_cpu.h
 *
 */
#ifndef stm32f4_uart_H_
#define stm32f4_uart_H_

// also see pipi_protocol_endpoint in projects/old/ntt_old1/working/cpu/microchip/saml21/things

typedef struct PACK_STRUCTURE stm32f4_uart {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;
	
	io_encoding_pipe_t *tx_pipe;
	io_event_t output_complete_event;
	io_byte_pipe_t *rx_pipe;

	// for single bind
	io_event_t *signal_transmit_available;
	io_event_t *signal_receive_data_available;
	
	io_cpu_clock_pointer_t peripheral_bus_clock;
	USART_TypeDef* uart_registers;
	uint32_t baud_rate;
	IRQn_Type interrupt_number;
	stm32f4_io_pin_t tx_pin;
	stm32f4_io_pin_t rx_pin;

} stm32f4_uart_t;

extern EVENT_DATA io_socket_implementation_t stm32f4_uart_implementation;

#define USART_CR1_CLEAR_MASK ((uint16_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE))


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------

static void
stm32f4_uart_interrupt_handler (void *user_value) {
	stm32f4_uart_t *this = user_value;
	volatile uint16_t data = this->uart_registers->DR;

	if (io_byte_pipe_put_byte (this->rx_pipe,data)) {
		// line mode?
		if (this->signal_receive_data_available) {
			io_enqueue_event (this->io,this->signal_receive_data_available);
		}
	}
}

static void stm32f4_uart_output_event_handler (io_event_t *ev);

static io_socket_t*
stm32f4_uart_initialise (
	io_socket_t *socket,io_t *io,io_settings_t const *C
) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;
	initialise_io_socket (socket,io);

	this->signal_transmit_available = NULL;
	this->signal_receive_data_available = NULL;
	
	this->rx_pipe = mk_io_byte_pipe (
		io_get_byte_memory(io),C->receive_pipe_length
	);

	this->tx_pipe = mk_io_encoding_pipe (
		io_get_byte_memory(io),C->transmit_pipe_length
	);

	initialise_io_event (
		&this->output_complete_event,stm32f4_uart_output_event_handler,this
	);

	register_io_interrupt_handler (
		io,this->interrupt_number,stm32f4_uart_interrupt_handler,this
	);
	
	this->State = &io_socket_default_state_closed;
	
	return socket;
}

bool
stm32f4_uart_is_closed (io_socket_t const *socket) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;
	return ((this->uart_registers->CR1 & USART_CR1_UE) == 0);
}

static bool
stm32f4_uart_open (io_socket_t *socket,io_socket_open_flag_t flag) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;

	if (io_cpu_clock_start (this->io,this->peripheral_bus_clock)) {
		if ((this->uart_registers->CR1 & USART_CR1_UE) == 0) {
			float64_t freq = io_cpu_clock_get_current_frequency (
				this->peripheral_bus_clock
			);
			uint32_t baud,temp_reg;

			if ((this->uart_registers->CR1 & USART_CR1_OVER8) != 0) {
				freq /= (float64_t) (8 * 1 * (this->baud_rate));
			} else {
				freq /= (float64_t) (8 * 2 * (this->baud_rate));
			}

			baud = (uint32_t) (floor (freq));
			baud <<= 4;
			if ((this->uart_registers->CR1 & USART_CR1_OVER8) != 0) {
				baud += (uint32_t) ((freq - (floor (freq))) * 8.0);
			} else {
				baud += (uint32_t) ((freq - (floor (freq))) * 16.0);
			}

			this->uart_registers->BRR = (uint16_t) baud;

			io_set_pin_to_alternate (io_socket_io (this),this->tx_pin.io);
			io_set_pin_to_alternate (io_socket_io (this),this->rx_pin.io);

			temp_reg = this->uart_registers->CR2;
			temp_reg &= (uint32_t) ~((uint32_t)USART_CR2_STOP);
			temp_reg |= (uint32_t) USART_StopBits_1;
			this->uart_registers->CR2 = (uint16_t)temp_reg;

			temp_reg = this->uart_registers->CR1;
			temp_reg &= (uint32_t)~((uint32_t) USART_CR1_CLEAR_MASK);
			temp_reg |= (
					8
				|	USART_Parity_No
				|	USART_Mode_Rx
				|	USART_Mode_Tx
			);
			this->uart_registers->CR1 = (uint16_t)temp_reg;
			this->uart_registers->CR3 = 0;

			USART_ITConfig(this->uart_registers,USART_IT_RXNE, ENABLE);
			USART_Cmd(this->uart_registers, ENABLE);

			NVIC_SetPriority (this->interrupt_number,0);
			NVIC_EnableIRQ (this->interrupt_number);
		}
	}

	return true;
}

static void
stm32f4_uart_close (io_socket_t *socket) {
	// and then ....
}

static io_encoding_t*
stm32f4_uart_new_message (io_socket_t *socket) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;
	return reference_io_encoding (
		new_io_encoding (
			this->encoding,io_get_byte_memory(this->io)
		)
	);
}

static bool
stm32f4_uart_send_message_blocking (
	io_socket_t *socket,io_encoding_t *encoding
) {
	bool sent = false;
	
	if (
			io_socket_is_open (socket)
		&&	is_io_binary_encoding (encoding)
	) {
		stm32f4_uart_t *this = (stm32f4_uart_t*) socket;
		const uint8_t *b,*e;

		io_encoding_get_content (encoding,&b,&e);
		while (b < e) {
			while (!(this->uart_registers->SR & USART_SR_TXE));
			this->uart_registers->DR = *b++;
		}

		sent = true;
	}
	
	unreference_io_encoding (encoding);
	return sent;
}

static void
stm32f4_uart_output_event_handler (io_event_t *ev) {
	stm32f4_uart_t *this = ev->user_value;
	io_encoding_t *next;
	while (io_encoding_pipe_peek (this->tx_pipe,&next)) {
		stm32f4_uart_send_message_blocking (ev->user_value,next);
		io_encoding_pipe_pop_encoding (this->tx_pipe);
	}
	
	if (this->signal_transmit_available) {
		io_enqueue_event (this->io,this->signal_transmit_available);
	}
}

static size_t
stm32f4_uart_mtu (io_socket_t const *socket) {
	return 1024;
}

static bool
stm32f4_uart_bind_to_outer (io_socket_t *socket,io_socket_t *outer) {
	return false;
}

static bool
stm32f4_uart_bind (io_socket_t *socket,io_address_t a,io_event_t *tx,io_event_t *rx) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;

	this->signal_transmit_available = tx;
	this->signal_receive_data_available = rx;

	return true;
}

io_pipe_t*
stm32f4_uart_get_receive_pipe (io_socket_t *socket,io_address_t address) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;
	return (io_pipe_t*) this->rx_pipe;
}

void
stm32f4_uart_flush (io_socket_t *socket) {
	stm32f4_uart_t *this = (stm32f4_uart_t*) socket;
	while (io_encoding_pipe_count_occupied_slots (this->tx_pipe) > 0);
}

EVENT_DATA io_socket_implementation_t stm32f4_uart_implementation = {
	SPECIALISE_IO_SOCKET_IMPLEMENTATION (
		&io_physical_socket_implementation
	)
	.initialise = stm32f4_uart_initialise,
	.open = stm32f4_uart_open,
	.close = stm32f4_uart_close,
	.is_closed = stm32f4_uart_is_closed,
	.bind_to_outer_socket = stm32f4_uart_bind_to_outer,
	.bind_inner = stm32f4_uart_bind,
	.get_receive_pipe = stm32f4_uart_get_receive_pipe,
	.new_message = stm32f4_uart_new_message,
	.send_message = stm32f4_uart_send_message_blocking,
	.flush = stm32f4_uart_flush,
	.mtu = stm32f4_uart_mtu,
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

