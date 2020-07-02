/*
 *
 * io representation of a stm32f4xx cpu
 *
 */
#ifndef io_cpu_H_
#define io_cpu_H_
#include <io_core.h>
#include <stm32f4xx_peripherals.h>

io_byte_memory_t* stm32f4_io_get_byte_memory (io_t*);
io_value_memory_t* stm32f4_io_get_short_term_value_memory (io_t*);
void stm32f4_do_gc (io_t*,int32_t);
void stm32f4_signal_event_pending (io_t*);
void stm32f4_wait_for_event (io_t*);
void stm32f4_wait_for_all_events (io_t*);
bool stm32f4_is_in_event_thread (io_t*);
bool stm32f4_enter_critical_section (io_t*);
void stm32f4_exit_critical_section (io_t*,bool);
void stm32f4_register_interrupt_handler (io_t*,int32_t,io_interrupt_action_t,void*);
void stm32f4_signal_task_pending (io_t*);
uint32_t stm32f4_random_uint32 (io_t*);
uint32_t stm32f4_get_prbs_random_u32 (io_t*);
void stm32f4_panic (io_t*,int);
bool stm32f4_is_first_run (io_t*);
bool stm32f4_clear_first_run (io_t*);
void stm32f4_set_io_pin_alternate (io_t*,io_pin_t);

#define SPECIALISE_IO_CPU_IMPLEMENTATION(S) \
	SPECIALISE_IO_IMPLEMENTATION(S) \
	.is_first_run = stm32f4_is_first_run, \
	.clear_first_run = stm32f4_clear_first_run,\
	.get_byte_memory = stm32f4_io_get_byte_memory, \
	.get_short_term_value_memory = stm32f4_io_get_short_term_value_memory, \
	.do_gc = stm32f4_do_gc, \
	.get_random_u32 = stm32f4_random_uint32, \
	.get_next_prbs_u32 = stm32f4_get_prbs_random_u32, \
	.wait_for_event = stm32f4_wait_for_event, \
	.wait_for_all_events = stm32f4_wait_for_all_events, \
	.signal_event_pending = stm32f4_signal_event_pending, \
	.in_event_thread = stm32f4_is_in_event_thread, \
	.signal_task_pending = stm32f4_signal_task_pending, \
	.enter_critical_section = stm32f4_enter_critical_section, \
	.exit_critical_section = stm32f4_exit_critical_section, \
	.register_interrupt_handler = stm32f4_register_interrupt_handler, \
	.panic = stm32f4_panic, \
	.set_io_pin_alternate = stm32f4_set_io_pin_alternate,\
	/**/
	
/*
	.unregister_interrupt_handler = NULL;
	.set_io_pin_output = NULL,
	.set_io_pin_input = NULL,
	.set_io_pin_interrupt = NULL,
	.read_from_io_pin = NULL,
	.write_to_io_pin = NULL,
	.toggle_io_pin = NULL,
	.valid_pin = NULL,
	.release_io_pin = NULL,
*/

#include "stm32f4_clocks.h"
#include "stm32f4_pins.h"
#include "stm32f4_uart.h"
#include "stm32f4_adc.h"
#include "stm32f4_twi.h"

#define STM32F4_IO_CPU_STRUCT_MEMBERS \
	IO_STRUCT_MEMBERS				\
	io_value_memory_t *vm;\
	io_byte_memory_t *bm;\
	uint32_t in_event_thread;\
	io_value_pipe_t *tasks;\
	uint32_t prbs_state[4]; \
	/**/

typedef struct PACK_STRUCTURE stm32f4xx_io {
	STM32F4_IO_CPU_STRUCT_MEMBERS
} stm32f4xx_io_t;

io_t*	initialise_cpu_io (io_t*);
void	microsecond_delay (uint32_t us);

typedef struct {
	uint8_t data[64];
	uint32_t datalen;
	uint64_t bitlen;
	uint32_t state[8];
} sha256_context_t;

//
// STM32F4 interrupts
//

#define ENABLE_INTERRUPTS			do {__DSB(); __enable_irq();} while (0);
#define DISABLE_INTERRUPTS			do {__disable_irq(); __DSB();} while (0);
#define EVENT_PRIORITY				((1 << __NVIC_PRIO_BITS) - 1)
#define EVENT_THREAD_INTERRUPT	PendSV_IRQn

#define NUMBER_OF_ARM_INTERRUPT_VECTORS	16L
#define NUMBER_OF_INTERRUPT_VECTORS (NUMBER_OF_ARM_INTERRUPT_VECTORS + NUMBER_OF_STM32F4_INTERRUPT_VECTORS)

typedef void (*io_cpu_interrupt_handler_t) (void*);

typedef struct PACK_STRUCTURE interrupt_handler {
	io_cpu_interrupt_handler_t handler;
	void *user_value;
} io_cpu_interrupt_t;

void	unhandled_cpu_interrupt (void*);

#ifndef nmi_cpu_interrupt
# define nmi_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef hard_fault_cpu_interrupt
# define hard_fault_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef memory_manager_cpu_interrupt
# define memory_manager_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef bus_fault_cpu_interrupt
# define bus_fault_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef usage_fault_cpu_interrupt
# define usage_fault_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef service_cpu_interrupt
# define service_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef debug_monitor_cpu_interrupt
# define debug_monitor_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef pending_service_cpu_interrupt
# define pending_service_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef system_tick_cpu_interrupt
# define system_tick_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef uart1_io_cpu_interrupt
# define uart1_io_cpu_interrupt	unhandled_cpu_interrupt
#endif
#ifndef uart3_io_cpu_interrupt
# define uart3_io_cpu_interrupt	unhandled_cpu_interrupt
#endif

#ifndef timer3_io_cpu_interrupt
# define timer3_io_cpu_interrupt	unhandled_cpu_interrupt
#endif

//
// implementation
//
#ifdef IMPLEMENT_STM32F4_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------

#define IO_CONFIG_MEMORY_SECTION __attribute__ ((section(".io_config")))

static IO_CONFIG_MEMORY_SECTION io_persistant_state_t stm32f4_cpu_config = {
	.first_run_flag = IO_FIRST_RUN_SET,
	.power_cycles = 0,
	.uid = {.bytes = {STATIC_UID}},
	.secret = {{0}},
	.shared = {{0}},
};

#define FLASH_WAIT_LIMIT 		(30000000*20)
#define SECTOR_SIZE	 			0x20000

static inline void
unlock_flash_control_register (void) {
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

bool
flash_wait_for_flash_ready(void) {
	uint32_t count = 0;
	while(FLASH->SR & FLASH_SR_BSY && count < FLASH_WAIT_LIMIT) count++;
	return count < FLASH_WAIT_LIMIT;
}

const uint32_t stm32f4_flash_sector_map[13] = {
	0x08000000,
	0x08004000,
	0x08008000,
	0x0800C000,
	0x08010000,
	0x08020000,	// first 128k sector
	0x08040000,
	0x08060000,
	0x08080000,
	0x080A0000,
	0x080C0000,
	0x080E0000,
	0x08100000,
};

void const*
stm32f4_get_flash_sector_base_address (uint32_t sector) {
	return (void const*) stm32f4_flash_sector_map[sector];
}

uint32_t
stm32f4_internal_flash_base_address_to_sector (uint32_t base) {
	uint32_t s = 0;
	while(s < SIZEOF(stm32f4_flash_sector_map) - 1) {
		if (
				(stm32f4_flash_sector_map[s] >= base)
			&&	(stm32f4_flash_sector_map[s + 1] > base)
		) {
			break;
		}
		s++;
	}
	return s;
}

//
// 128k sectors, numbers 5 through 11
//
bool
stm32f4_flash_erase_sector (uint32_t sector) {
	// can this fail?
	flash_wait_for_flash_ready();

	unlock_flash_control_register();

	FLASH->CR |= (2 << 8);	//set PSIZE to x32
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR = (FLASH->CR & ~(0x0f << 3)) + ((0x0f & sector) << 3);
	FLASH->CR |= FLASH_CR_STRT;

	flash_wait_for_flash_ready();
	FLASH->CR &= ~FLASH_CR_SER;
	FLASH->CR &= ~(3 << 8);	//set PSIZE to x8
	FLASH->CR |= FLASH_CR_LOCK;

	return 1;
}

//
// write to flash
//
bool
stm32f4_flash_write_with_callback (
	uint32_t sector,void (*cb) (uint32_t*,uint32_t*),uint32_t *user_value
) {
	if (!(FLASH->CR & FLASH_CR_PG)) {
		uint32_t *flash = (uint32_t*) (stm32f4_flash_sector_map[sector]);

		stm32f4_flash_erase_sector (sector);

		// wait until flash is not busy
		while (FLASH->SR & FLASH_SR_BSY);

		unlock_flash_control_register();
		FLASH->CR |= FLASH_CR_PG;	// enable write

		cb(flash,user_value);

		while (FLASH->SR & FLASH_SR_BSY);
		FLASH->CR &= ~FLASH_CR_PG;
		FLASH->CR |= FLASH_CR_LOCK;

		return true;
	} else {
		return false;
	}
}

bool
stm32f4_is_first_run (io_t *io) {
	return (stm32f4_cpu_config.first_run_flag == IO_FIRST_RUN_SET);
}

void
stm32f4_write_cpu_config (uint32_t *dest,uint32_t *config) {
}

bool
stm32f4_clear_first_run (io_t *io) {
	if (stm32f4_cpu_config.first_run_flag == IO_FIRST_RUN_SET) {
		io_persistant_state_t new_config = stm32f4_cpu_config;

		new_config.first_run_flag = IO_FIRST_RUN_CLEAR;

		DISABLE_INTERRUPTS;

		stm32f4_flash_write_with_callback (
			stm32f4_internal_flash_base_address_to_sector(
				(uint32_t) &stm32f4_cpu_config
			),
			stm32f4_write_cpu_config,(uint32_t*) &new_config
		);
		
		ENABLE_INTERRUPTS;

		return memcmp (&new_config,&stm32f4_cpu_config,sizeof(io_persistant_state_t)) == 0;
	} else {
		return true;
	}
}

//
// should be able to use the RNG
//
uint32_t
stm32f4_random_uint32 (io_t *io) {
	ADC_TempSensorVrefintCmd(ENABLE);
	// don't wait here. We want to be reading as the voltage reference
	// tries to start up, so make things as random as we can!
	int s;
	uint32_t v = 0;
	for (s = 0; s < 32; s++) {
		v = (v<<3) ^ (v>>29) ^ (unsigned int)stm32f4_analog_read(ADC1,ANALOG_CHANNEL_17, false);
	}
	ADC_TempSensorVrefintCmd(DISABLE);
	return v;
}

io_byte_memory_t*
stm32f4_io_get_byte_memory (io_t *io) {
	stm32f4xx_io_t *this = (stm32f4xx_io_t*) io;
	return this->bm;
}

io_value_memory_t*
stm32f4_io_get_short_term_value_memory (io_t *io) {
	stm32f4xx_io_t *this = (stm32f4xx_io_t*) io;
	return this->vm;
}

void
stm32f4_panic (io_t *io,int code) {
	DISABLE_INTERRUPTS;
	while (1);
}

void
stm32f4_wait_for_event (io_t *io) {
	__WFI();
}

void
stm32f4_wait_for_all_events (io_t *io) {
	io_event_t *event;
	io_alarm_t *alarm;
	do {
		ENTER_CRITICAL_SECTION(io);
		event = io->events;
		alarm = io->alarms;
		EXIT_CRITICAL_SECTION(io);
	} while (
			event != &s_null_io_event
		||	alarm != &s_null_io_alarm
	);
}

bool
stm32f4_is_in_event_thread (io_t *io) {
	return ((stm32f4xx_io_t*) io)->in_event_thread;
}

bool
stm32f4_enter_critical_section (io_t *io) {
	uint32_t interrupts_are_enabled = !(__get_PRIMASK() & 0x1);
	DISABLE_INTERRUPTS;
	return interrupts_are_enabled;
}

void
stm32f4_exit_critical_section (io_t *io,bool were_enabled) {
	if (were_enabled) {
		ENABLE_INTERRUPTS;
	}
}

void
stm32f4_signal_event_pending (io_t *io) {
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void
stm32f4_do_gc (io_t *io,int32_t count) {
	io_value_memory_do_gc (io_get_short_term_value_memory (io),count);
}

void
initialise_core_clock (io_t *io) {
	io_cpu_clock_pointer_t core_clock = io_get_core_clock(io);
	uint32_t cf = io_cpu_clock_get_current_frequency (core_clock);

	io_cpu_clock_start (io,core_clock);

	//
	// requires core clock frequency to be a multiple of 1MHz
	//
	SysTick->LOAD = cf/1000000UL;
	SysTick->LOAD -= 1UL;
	SysTick->VAL = 0UL;
	SysTick->CTRL = (
			SysTick_CTRL_CLKSOURCE_Msk
		|	SysTick_CTRL_ENABLE_Msk
	);
}


static void
tune_stm32f4_cpu (void) {
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	// set CP10 and CP11 to full access
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
	#endif

	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Disable all interrupts
	RCC->CIR = 0x00000000;

	// enable instruction and data cache
	FLASH->ACR = (
			FLASH_ACR_ICEN
		|	FLASH_ACR_DCEN
	);

    // regulator voltage output Scale 1 for core clock up to 168 MHz
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

}

static void
null_interrupt_handler (void *w) {
	while(1);
}

static io_interrupt_handler_t cpu_interrupts[NUMBER_OF_INTERRUPT_VECTORS];

static void
handle_io_cpu_interrupt (void) {
	io_interrupt_handler_t const *interrupt = &cpu_interrupts[
		SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk
	];
	interrupt->action(interrupt->user_value);
}

void	
stm32f4_register_interrupt_handler (
	io_t *io,int32_t number,io_interrupt_action_t handler,void *user_value
) {
	io_interrupt_handler_t *i = (
		cpu_interrupts + number + NUMBER_OF_ARM_INTERRUPT_VECTORS
	);
	i->action = handler;
	i->user_value = user_value;
}

static void
event_thread (void *io) {
	while (next_io_event (io));
}

void
stm32f4_signal_task_pending (io_t *io) {
	// no action required
}

INLINE_FUNCTION uint32_t prbs_rotl(const uint32_t x, int k) {
	return (x << k) | (x >> (32 - k));
}

uint32_t
stm32f4_get_prbs_random_u32 (io_t *io) {
	stm32f4xx_io_t *this = (stm32f4xx_io_t*) io;
	uint32_t *s = this->prbs_state;
	bool h = enter_io_critical_section (io);
	const uint32_t result = prbs_rotl (s[0] + s[3], 7) + s[0];

	const uint32_t t = s[1] << 9;

	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];

	s[2] ^= t;

	s[3] = prbs_rotl (s[3], 11);

	exit_io_critical_section (io,h);
	return result;
}

/*
 *-----------------------------------------------------------------------------
 *
 * get the io resources for the stm32f4
 *
 *-----------------------------------------------------------------------------
 */
io_t*
initialise_cpu_io (io_t *io) {
	stm32f4xx_io_t *cpu = (stm32f4xx_io_t*) io;
	
	// tune cpu ...
	NVIC_SetPriorityGrouping(0);

//	io_device_add_io_methods (&io_i);
	
//	cpu->implementation = &io_i;
	cpu->in_event_thread = 0;

//	initialise_io (io,&io_i);
	initialise_core_clock (io);

	NVIC_SetPriority (EVENT_THREAD_INTERRUPT,EVENT_PRIORITY);
	register_io_interrupt_handler (
		io,EVENT_THREAD_INTERRUPT,event_thread,io
	);

	cpu->prbs_state[0] = io_get_random_u32(io);
	cpu->prbs_state[1] = 0xf542d2d3;
	cpu->prbs_state[2] = 0x6fa035c3;
	cpu->prbs_state[3] = 0x77f2db5b;
	
	return io;
}

static void
make_ram_interrupt_vectors (void) {
	io_interrupt_handler_t *i = cpu_interrupts;
	io_interrupt_handler_t *e = i + NUMBER_OF_INTERRUPT_VECTORS;
	while (i < e) {
		i->action = null_interrupt_handler;
		i->user_value = NULL;
		i++;
	}
}

static void
initialise_c_runtime (void) {
	extern uint32_t ld_start_of_sdata_in_flash,ld_start_of_sdata_in_ram,ld_end_of_sdata_in_ram;
	extern uint32_t ld_start_of_bss,ld_end_of_bss;

	uint32_t *src = &ld_start_of_sdata_in_flash;
	uint32_t *dest = &ld_start_of_sdata_in_ram;

	while(dest < &ld_end_of_sdata_in_ram) *dest++ = *src++;
	dest = &ld_start_of_bss;
	while(dest < &ld_end_of_bss) *dest++ = 0;

	// fill stack/heap region of RAM with a pattern
	extern uint32_t ld_end_of_static_ram_allocations;
	uint32_t *end = (uint32_t*) __get_MSP();
	dest = &ld_end_of_static_ram_allocations;
	while (dest < end) *dest++ = 0xdeadc0de;
	
	SCB->VTOR = FLASH_BASE;
	make_ram_interrupt_vectors ();
}

//
// entry point on core reset (CPU starts on HSI @ 16MHz)
//
int main(void);
void
stm32f4_core_reset (void) {
	initialise_c_runtime();
	tune_stm32f4_cpu ();
	main ();
	while (1);
}

void
unhandled_cpu_interrupt (void *user_value) {
	while (1);
}

static uint32_t g_us_delay;
static void
SysTick_Handler (void) {
	if (g_us_delay) {
		g_us_delay--;
	} else {
		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	}
}

//  will block for at least us microseconds
void
microsecond_delay (uint32_t us) {
	g_us_delay = us + 1;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	while (SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) {
	}
}

extern uint32_t ld_top_of_c_stack;
__attribute__ ((section(".isr_vector")))
const void* s_flash_vector_table[NUMBER_OF_INTERRUPT_VECTORS] = {
	&ld_top_of_c_stack,
	stm32f4_core_reset,	//
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	SysTick_Handler,

	handle_io_cpu_interrupt,				// Window Watchdog interrupt
	handle_io_cpu_interrupt,				// PVD through EXTI line detection interrupt
	handle_io_cpu_interrupt,				// Tamper and TimeStamp interrupts through the EXTI line
	handle_io_cpu_interrupt,				// RTC Wakeup interrupt through the EXTI line
	handle_io_cpu_interrupt,				// Reserved
	handle_io_cpu_interrupt,				// RCC global interrupt
	handle_io_cpu_interrupt,				// EXTI Line0 interrupt
	handle_io_cpu_interrupt,				// EXTI Line1 interrupt
	handle_io_cpu_interrupt,				// EXTI Line2 interrupt
	handle_io_cpu_interrupt,				// EXTI Line3 interrupt
	handle_io_cpu_interrupt,				// EXTI Line4 interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream0 global interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream1 global interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream2 global interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream3 global interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream4 global interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream5 global interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream6 global interrupt
	handle_io_cpu_interrupt,				// ADC3 global interrupts
	handle_io_cpu_interrupt,				// CAN1 TX interrupts
	handle_io_cpu_interrupt,				// CAN1 RX0 interrupts
	handle_io_cpu_interrupt,				// CAN1 RX1 interrupts
	handle_io_cpu_interrupt,				// CAN1 SCE interrupt
	handle_io_cpu_interrupt,				// EXTI Line[9:5] interrupts
	handle_io_cpu_interrupt,				// TIM1 Break interrupt and TIM9 global interrupt
	handle_io_cpu_interrupt,				// TIM1 Update interrupt and TIM10 global interrupt
	handle_io_cpu_interrupt,				// TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
	handle_io_cpu_interrupt,				// TIM1 Capture Compare interrupt
	handle_io_cpu_interrupt,				// TIM2 global interrupt
	handle_io_cpu_interrupt,				// TIM3 global interrupt
	handle_io_cpu_interrupt,				// TIM4 global interrupt
	handle_io_cpu_interrupt,				// I2C1 event interrupt
	handle_io_cpu_interrupt,				// I2C1 error interrupt
	handle_io_cpu_interrupt,				// I2C2 event interrupt
	handle_io_cpu_interrupt,				// I2C2 error interrupt
	handle_io_cpu_interrupt,				// SPI1 global interrupt
	handle_io_cpu_interrupt,				// SPI2 global interrupt
	handle_io_cpu_interrupt,				// USART1 global interrupt
	handle_io_cpu_interrupt,				// USART2 global interrupt
	handle_io_cpu_interrupt,				// USART3 global interrupt
	handle_io_cpu_interrupt,				// EXTI Line[15:10] interrupts
	handle_io_cpu_interrupt,				// RTC Alarms (A and B) through EXTI line interrupt
	handle_io_cpu_interrupt,				// USB On-The-Go FS Wakeup through EXTI line interrupt
	handle_io_cpu_interrupt,				// TIM8 Break interrupt and TIM12 global interrupt
	handle_io_cpu_interrupt,				// TIM8 Update interrupt and TIM13 global interrupt
	handle_io_cpu_interrupt,				// TIM8 Trigger and Commutation interrupts and TIM14 global interrupt
	handle_io_cpu_interrupt,				// TIM8 Capture Compare interrupt
	handle_io_cpu_interrupt,				// DMA1 Stream7 global interrupt
	handle_io_cpu_interrupt,				// FSMC global interrupt
	handle_io_cpu_interrupt,				// SDIO global interrupt
	handle_io_cpu_interrupt,				// TIM5 global interrupt
	handle_io_cpu_interrupt,				// SPI3 global interrupt
	handle_io_cpu_interrupt,				// UART4 global interrupt
	handle_io_cpu_interrupt,				// UART5 global interrupt
	handle_io_cpu_interrupt,				// TIM6 global interrupt, DAC1 and DAC2 underrun error interrupt
	handle_io_cpu_interrupt,				// TIM7 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream0 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream1 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream2 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream3 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream4 global interrupt
	handle_io_cpu_interrupt,				// Ethernet global interrupt
	handle_io_cpu_interrupt,				// Ethernet Wakeup through EXTI line interrupt
	handle_io_cpu_interrupt,				// CAN2 TX interrupts
	handle_io_cpu_interrupt,				// CAN2 RX0 interrupts
	handle_io_cpu_interrupt,				// CAN2 RX1 interrupts
	handle_io_cpu_interrupt,				// CAN2 SCE interrupt
	handle_io_cpu_interrupt,				// USB On The Go FS global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream5 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream6 global interrupt
	handle_io_cpu_interrupt,				// DMA2 Stream7 global interrupt
	handle_io_cpu_interrupt,				// USART6 global interrupt
	handle_io_cpu_interrupt,				// I2C3 event interrupt
	handle_io_cpu_interrupt,				// I2C3 error interrupt
	handle_io_cpu_interrupt,				// USB On The Go HS End Point 1 Out global interrupt
	handle_io_cpu_interrupt,				// USB On The Go HS End Point 1 In global interrupt
	handle_io_cpu_interrupt,				// USB On The Go HS Wakeup through EXTI interrupt
	handle_io_cpu_interrupt,				// USB On The Go HS global interrupt
	handle_io_cpu_interrupt,				// DCMI global interrupt
	handle_io_cpu_interrupt,				// CRYP crypto global interrupt
	handle_io_cpu_interrupt,				// Hash and Rng global interrupt
	handle_io_cpu_interrupt,				// Floating point interrupt
};


#endif /* IMPLEMENT_STM32F4_IO_CPU */
#endif /* io_cpu_H_ */
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
