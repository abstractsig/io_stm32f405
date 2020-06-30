/*
 *
 * stm32f4 adc
 *
 * NB: included by io_cpu.h
 *
 */
#ifndef stm32f4_adc_H_
#define stm32f4_adc_H_

int stm32f4_analog_read (ADC_TypeDef*,uint32_t,bool);

#ifdef IMPLEMENT_STM32F4_IO_CPU
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------

int
stm32f4_analog_read (ADC_TypeDef *ADCx,uint32_t channel_number, bool fastConversion) {
	static bool is_initialised = false;
	bool needs_init = false;

	if (!fastConversion) {
		if (ADCx == ADC1) {
	      if (!is_initialised) {
	        is_initialised = true;
	        needs_init = true;
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	      }
		}
	}
	if (needs_init) {
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef ADC_InitStructure;

		ADC_CommonStructInit(&ADC_CommonInitStructure);
		ADC_CommonInit(&ADC_CommonInitStructure);

		ADC_StructInit(&ADC_InitStructure);
		// Preinit
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_Init(ADCx, &ADC_InitStructure);

		// Enable the ADC
		ADC_Cmd(ADCx, ENABLE);
	}

	uint8_t sampleTime = fastConversion ? ADC_SampleTime_3Cycles : ADC_SampleTime_480Cycles;
	ADC_RegularChannelConfig(ADCx,channel_number, 1, sampleTime);
	ADC_SoftwareStartConv(ADCx);

	while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);

	return (int)ADC_GetConversionValue(ADCx);
}

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

