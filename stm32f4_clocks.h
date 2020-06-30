/*
 *
 * power domains and clocks for stm32f4 cpus
 *
 * NB: included by io_cpu.h
 *
 */
#ifndef stm32f4_clocks_H_
#define stm32f4_clocks_H_

typedef struct PACK_STRUCTURE stm32f4_crystal_oscillator {
	IO_CPU_CLOCK_SOURCE_STRUCT_MEMBERS
	float64_t crystal_frequency;
} stm32f4_crystal_oscillator_t;

typedef struct PACK_STRUCTURE stm32f4_rc_oscillator {
	IO_CPU_CLOCK_SOURCE_STRUCT_MEMBERS
	float64_t frequency;
} stm32f4_rc_oscillator_t;

typedef struct PACK_STRUCTURE stm32f4_apb_clock {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS
	uint32_t divider;
} stm32f4_ahb_clock_t, stm32f4_apb_clock_t;

typedef struct PACK_STRUCTURE stm32f4_pll {
	IO_CPU_CLOCK_FUNCTION_STRUCT_MEMBERS

	uint32_t M;
	uint32_t N;
	uint32_t P;
	uint32_t Q;	// for USB

} stm32f4_pll_t;

typedef struct stm32f4_core_clock {
	IO_CPU_DEPENDANT_CLOCK_STRUCT_MEMBERS
} stm32f4_core_clock_t;

typedef struct stm32f4_peripheral_clock {
	IO_CPU_DEPENDANT_CLOCK_STRUCT_MEMBERS
	void (*enable_apb_clock) (void);
} stm32f4_peripheral_clock_t;

extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_hsi_oscillator_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_hse_oscillator_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_pll_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_core_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_ahb_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_apb1_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_apb2_clock_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t stm32f4_apb_peripheral_clock_implementation;


#ifdef IMPLEMENT_STM32F4_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------


static float64_t
stm32f4_hsi_oscillator_get_current_frequency (io_cpu_clock_pointer_t this) {
	stm32f4_rc_oscillator_t const *c = (stm32f4_rc_oscillator_t const*) (
		io_cpu_clock_ro_pointer (this)
	);
	return c->frequency;
}

static bool
stm32f4_hsi_oscillator_start (io_t* io,io_cpu_clock_pointer_t this) {
	if (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == 0) {
		// HSION
		RCC->CR |= (uint32_t)0x00000001;
		while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == 0);
		return true;
	} else {
		return true;
	}
}

EVENT_DATA io_cpu_clock_implementation_t stm32f4_hsi_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_current_frequency = stm32f4_hsi_oscillator_get_current_frequency,
	.start = stm32f4_hsi_oscillator_start,
	.stop = NULL,
};

bool
stm32f4_clock_is_hsi (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&stm32f4_hsi_oscillator_implementation);
}

static float64_t
stm32f4_hse_oscillator_get_current_frequency (io_cpu_clock_pointer_t this) {
	stm32f4_crystal_oscillator_t const *c = (stm32f4_crystal_oscillator_t const*) (
		io_cpu_clock_ro_pointer (this)
	);
	return c->crystal_frequency;
}

static bool
stm32f4_hse_oscillator_start (io_t *io,io_cpu_clock_pointer_t this) {
	if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == 0) {
		RCC_HSEConfig(RCC_HSE_ON);
		if (RCC_WaitForHSEStartUp () == SUCCESS) {
			return true;
		} else {
			return false;
		}
	} else {
		return true;
	}
}

EVENT_DATA io_cpu_clock_implementation_t stm32f4_hse_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_current_frequency = stm32f4_hse_oscillator_get_current_frequency,
	.start = stm32f4_hse_oscillator_start,
	.stop = NULL,
};

bool
stm32f4_clock_is_hse (io_cpu_clock_pointer_t clock) {
	return io_cpu_clock_has_implementation (clock,&stm32f4_hse_oscillator_implementation);
}

static float64_t
stm32f4_pll_oscillator_get_current_frequency (io_cpu_clock_pointer_t clock) {
	stm32f4_pll_t const *this = (
		(stm32f4_pll_t const*) io_cpu_clock_ro_pointer (clock)
	);
	float64_t f = io_cpu_clock_get_current_frequency (this->input);

	f = ((f / (float64_t) this->M) * (float64_t) this->N) / (float64_t) (this->P);

	return f;
}

static bool
stm32f4_pll_oscillator_start (io_t *io,io_cpu_clock_pointer_t clock) {
	stm32f4_pll_t const *this = (
		(stm32f4_pll_t const*) io_cpu_clock_ro_pointer (clock)
	);
	if (io_cpu_dependant_clock_start_input (io,clock)) {
		uint32_t source = 0;

		if (stm32f4_clock_is_hse (this->input)) {
			source = RCC_PLLCFGR_PLLSRC_HSE;
		} else if (stm32f4_clock_is_hsi (this->input)) {
			source = RCC_PLLCFGR_PLLSRC_HSI;
		} else {
			return false;
		}

		RCC->PLLCFGR = (
				this->M
			|	(this->N << 6)
			|	(((this->P >> 1) -1) << 16)
			| 	(source)
			|	(this->Q << 24)
		);
		RCC->CR |= RCC_CR_PLLON;

		while((RCC->CR & RCC_CR_PLLRDY) == 0);

		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t stm32f4_pll_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_current_frequency = stm32f4_pll_oscillator_get_current_frequency,
	.start = stm32f4_pll_oscillator_start,
	.stop = NULL,
};

static float64_t
stm32f4_apb_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	stm32f4_apb_clock_t const *this = (
		(stm32f4_apb_clock_t const*) io_cpu_clock_ro_pointer (clock)
	);
	float64_t f = io_cpu_clock_get_current_frequency (this->input);
	return f / (float64_t) this->divider;
}

#define SPECIALISE_STM32F4_APB_CLOCK_IMPLEMENTATION(S) \
	SPECIALISE_IO_CPU_CLOCK_IMPLEMENTATION (S) \
	.get_current_frequency = stm32f4_apb_clock_get_current_frequency,\
	/**/
	
EVENT_DATA io_cpu_clock_implementation_t 
stm32f4_apb_clock_implementation = {
	SPECIALISE_STM32F4_APB_CLOCK_IMPLEMENTATION (
		&io_cpu_clock_implementation
	)
};

void
stm32f4_enable_spi1_apb_clock (void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
}

void
stm32f4_enable_uart1_apb_clock (void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

static bool
stm32f4_ahb_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	if (io_cpu_dependant_clock_start_input (io,clock)) {
		stm32f4_ahb_clock_t const *this = (
			(stm32f4_ahb_clock_t const*) io_cpu_clock_ro_pointer (clock)
		);
		bool ok = true;

		switch (this->divider) {
			case 1:
				RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
			break;

			default:
				ok = false;
			break;
		}

		return ok;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t stm32f4_ahb_clock_implementation = {
	SPECIALISE_STM32F4_APB_CLOCK_IMPLEMENTATION (
		&stm32f4_apb_clock_implementation
	)
	.start = stm32f4_ahb_clock_start,
};

static bool
stm32f4_apb1_clock_start (io_t* io,io_cpu_clock_pointer_t clock) {
	if (io_cpu_dependant_clock_start_input (io,clock)) {
		stm32f4_apb_clock_t const *this = (
			(stm32f4_apb_clock_t const*) io_cpu_clock_ro_pointer (clock)
		);
		bool ok = true;

		// HCLK divider
		switch (this->divider) {
			case 1:
				RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
			break;
			case 4:
				RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
			break;

			default:
				ok = false;
			break;
		}

		return ok;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t 
stm32f4_apb1_clock_implementation = {
	SPECIALISE_STM32F4_APB_CLOCK_IMPLEMENTATION (
		&stm32f4_apb_clock_implementation
	)
	.start = stm32f4_apb1_clock_start,
};

static bool
stm32f4_apb2_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	if (io_cpu_dependant_clock_start_input (io,clock)) {
		stm32f4_apb_clock_t const *this = (
			(stm32f4_apb_clock_t const*) io_cpu_clock_ro_pointer (clock)
		);
		bool ok = true;

		// HCLK divider
		switch (this->divider) {
			case 1:
				RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
			break;
			case 2:
				RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
			break;
			case 4:
				RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
			break;

			default:
				ok = false;
			break;
		}

		return ok;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t 
stm32f4_apb2_clock_implementation = {
	SPECIALISE_STM32F4_APB_CLOCK_IMPLEMENTATION (
		&stm32f4_apb_clock_implementation
	)
	.start = stm32f4_apb2_clock_start,
};

static float64_t
stm32f4_core_clock_get_current_frequency (io_cpu_clock_pointer_t clock) {
	stm32f4_core_clock_t const *this = (stm32f4_core_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_current_frequency (this->input);
}

static bool
stm32f4_core_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	if (io_cpu_dependant_clock_start_input (io,clock)) {
		stm32f4_core_clock_t const *this = (stm32f4_core_clock_t const*) (
			io_cpu_clock_ro_pointer (clock)
		);
		float64_t new_frequency = io_cpu_clock_get_current_frequency (this->input);

		if (io_math_compare_float64_ge (new_frequency,168000000.0)) {
			// based on frequency, set WS
			// wait states for 168MHz @ 3.3V
			FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
		} else {
			// need to find the wait state table
		}

		/* Select the main PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		/* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL) {
		}

		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t 
stm32f4_core_clock_implementation = {
	SPECIALISE_IO_CPU_CLOCK_IMPLEMENTATION (
		&io_cpu_clock_implementation
	)
	.get_current_frequency = stm32f4_core_clock_get_current_frequency,
	.start = stm32f4_core_clock_start,
};

static bool
stm32f4_apb_peripheral_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	if (io_dependant_cpu_clock_start (io,clock)) {
		stm32f4_peripheral_clock_t const *this = (
			(stm32f4_peripheral_clock_t const*) io_cpu_clock_ro_pointer (clock)
		);
		this->enable_apb_clock();
		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t 
stm32f4_apb_peripheral_clock_implementation = {
	SPECIALISE_DEPENDANT_IO_CPU_CLOCK_IMPLEMENTATION (
		&io_dependent_clock_implementation
	)
	.start = stm32f4_apb_peripheral_clock_start,
};

#endif /* IMPLEMENT_STM32F4_IO_CPU */
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
