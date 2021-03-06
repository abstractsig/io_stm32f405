/*
 *
 * stm32f4 pins
 *
 * NB: included by io_cpu.h
 *
 */
#ifndef stm32f4_pins_H_
#define stm32f4_pins_H_
//
// pins
//
typedef union PACK_STRUCTURE {
	io_pin_t io;
	uint32_t u32;
	struct PACK_STRUCTURE {
		uint32_t port_id:4;
		uint32_t number:4;
		uint32_t alternate:4;
		uint32_t active_level:1;
		uint32_t initial_state:1;
		uint32_t mode:2;
		uint32_t pull_mode:2;
		uint32_t speed:2;
		uint32_t output_type:1;	// push-pull or open-drain
		uint32_t pull:2;
		uint32_t :9;
	} stm;
} stm32f4_io_pin_t;

struct stm_gpio_port {
	GPIO_TypeDef *registers;
	void (*enable_clock) (void);
};

extern const struct stm_gpio_port stm_gpio_ports[];

#define stm_pin_enable_port_clock(pin)		stm_gpio_ports[(pin).stm.port_id].enable_clock()
#define stm_pin_port(pin)						stm_gpio_ports[(pin).stm.port_id].registers
#define stm_pin_number(pin)					(pin).stm.number
#define stm_pin_active_level(pin)			(pin).stm.active_level
#define stm_pin_initial_state(pin)			(pin).stm.initial_state
#define stm_pin_speed(pin)						(pin).stm.speed
#define stm_pin_alternate(pin)				(pin).stm.alternate
#define stm_pin_pull_mode(pin)				(pin).stm.pull_mode
#define stm_pin_output_type(pin)				(pin).stm.output_type
#define stm_pin_pull(pin)						(pin).stm.pull
#define stm_pin_mode(pin)						(pin).stm.mode

#define IO_PIN_ACTIVE_LOW			0
#define IO_PIN_ACTIVE_HIGH			1
#define IO_PIN_INACTIVE				0
#define IO_PIN_ACTIVE				1

enum {
	STM_GPIO_PORT_A = 0,
	STM_GPIO_PORT_B,
	STM_GPIO_PORT_C,
	STM_GPIO_PORT_D,
	STM_GPIO_PORT_E,
};

void configure_stm32f4_io_pin_alternate (stm32f4_io_pin_t);
void configure_stm32f4_io_pin (stm32f4_io_pin_t);


INLINE_FUNCTION void
stm32f4_pin_set_low (stm32f4_io_pin_t pin) {
	stm_pin_port(pin)->BSRRH = (1 << stm_pin_number(pin));
}

INLINE_FUNCTION void
stm32f4_pin_set_high (stm32f4_io_pin_t pin) {
	stm_pin_port(pin)->BSRRL = (1 << stm_pin_number(pin));
}

typedef enum {
  ANALOG_CHANNEL_0 = 0,
  ANALOG_CHANNEL_1,
  ANALOG_CHANNEL_2,
  ANALOG_CHANNEL_3,
  ANALOG_CHANNEL_4,
  ANALOG_CHANNEL_5,
  ANALOG_CHANNEL_6,
  ANALOG_CHANNEL_7,
  ANALOG_CHANNEL_8,
  ANALOG_CHANNEL_9,
  ANALOG_CHANNEL_10,
  ANALOG_CHANNEL_11,
  ANALOG_CHANNEL_12,
  ANALOG_CHANNEL_13,
  ANALOG_CHANNEL_14,
  ANALOG_CHANNEL_15,
  ANALOG_CHANNEL_16,
  ANALOG_CHANNEL_17,
} analogue_channel_number_t;

#define def_stm32f4_function_pin(Port,Pin,Function,Speed) {\
			.port_id = Port,\
			.number = Pin,\
			.mode = GPIO_Mode_AF,\
			.alternate = Function,\
			.speed = Speed, \
			.pull = GPIO_PuPd_NOPULL,\
		}\
		/**/

#define def_stm32f4_output_pin(Port,Pin,ActiveLevel,InitialLevel,Speed) {\
			.port_id = Port,\
			.number = Pin,\
			.active_level = ActiveLevel,\
			.initial_state = InitialLevel,\
			.mode = GPIO_Mode_OUT,\
			.speed = Speed,\
			.output_type = GPIO_OType_PP,\
			.pull = GPIO_PuPd_NOPULL,\
		}


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------
//
// pins
//

static void
enable_gpio_port_a_clock (void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

static void
enable_gpio_port_b_clock (void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
}

static void
enable_gpio_port_c_clock (void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
}

static void
enable_gpio_port_d_clock (void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}

static void
enable_gpio_port_e_clock (void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}

const struct stm_gpio_port stm_gpio_ports[] = {
	{GPIOA,enable_gpio_port_a_clock},
	{GPIOB,enable_gpio_port_b_clock},
	{GPIOC,enable_gpio_port_c_clock},
	{GPIOD,enable_gpio_port_d_clock},
	{GPIOE,enable_gpio_port_e_clock},
	{NULL}
};

void
configure_stm32f4_io_pin_alternate (stm32f4_io_pin_t pin) {
	GPIO_InitTypeDef GPIO_InitStructure;

	stm_pin_enable_port_clock(pin);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = stm_pin_output_type(pin);
	GPIO_InitStructure.GPIO_PuPd = stm_pin_pull (pin);
	GPIO_InitStructure.GPIO_Speed = stm_pin_speed (pin);
	GPIO_InitStructure.GPIO_Pin = (1 << stm_pin_number (pin));
	GPIO_InitStructure.GPIO_PuPd = stm_pin_pull(pin);
	GPIO_Init(stm_pin_port (pin), &GPIO_InitStructure);

	GPIO_PinAFConfig (
		stm_pin_port (pin),
		stm_pin_number (pin),
		stm_pin_alternate(pin)
	);
}

void
configure_stm32f4_io_pin (stm32f4_io_pin_t pin) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = stm_pin_mode(pin);
	GPIO_InitStructure.GPIO_Pin = (1 << stm_pin_number (pin));
	GPIO_InitStructure.GPIO_OType = stm_pin_output_type(pin);
	GPIO_InitStructure.GPIO_Speed = stm_pin_speed (pin);
	GPIO_InitStructure.GPIO_PuPd = stm_pin_pull(pin);
	
	GPIO_Init(stm_pin_port (pin), &GPIO_InitStructure);
}
	
void
stm32f4_write_to_io_pin (io_t *io,io_pin_t rpin,int32_t state) {
	stm32f4_io_pin_t pin = {rpin};
	
	// if analogue ...
	
	if (state ^ stm_pin_active_level (pin)) {
		stm32f4_pin_set_low (pin);
	} else {
		stm32f4_pin_set_high(pin);
	}
}

void
stm32f4_set_io_pin_output (io_t *io,io_pin_t p) {
	stm32f4_io_pin_t pin = {p};

	stm_pin_enable_port_clock(pin);
	stm32f4_write_to_io_pin (io,p,stm_pin_initial_state(pin));
	configure_stm32f4_io_pin (pin);
}

void
stm32f4_set_io_pin_alternate (io_t *io,io_pin_t p) {
	stm32f4_io_pin_t pin = {p};
	configure_stm32f4_io_pin_alternate (pin);
}

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

