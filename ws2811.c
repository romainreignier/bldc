/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <math.h>
#include "ws2811.h"
#include "stm32f4xx_conf.h"
#include "ch.h"
#include "hal.h"

// Settings
#define TIM_PERIOD			(((168000000 / 2 / WS2811_CLK_HZ) - 1))
#define LED_BUFFER_LEN		(WS2811_LED_NUM + 1)
#define BITBUFFER_PAD		50
#define BITBUFFER_LEN		(24 * LED_BUFFER_LEN + BITBUFFER_PAD)
#define WS2811_ZERO			(TIM_PERIOD * 0.2)
#define WS2811_ONE			(TIM_PERIOD * 0.8)

// Private variables
static uint16_t bitbuffer[BITBUFFER_LEN];
static uint32_t RGBdata[LED_BUFFER_LEN];
static uint8_t gamma_table[256];
static uint32_t brightness;

// Private function prototypes
static uint32_t rgb_to_local(uint32_t color);

void ws2811_init(void) {
	LL_TIM_InitTypeDef  TIM_TimeBaseStructure;
	LL_TIM_OC_InitTypeDef  TIM_OCInitStructure;
	LL_DMA_InitTypeDef DMA_InitStructure;

	brightness = 100;

	// Default LED values
	int i, bit;

	for (i = 0;i < LED_BUFFER_LEN;i++) {
		RGBdata[i] = 0;
	}

	for (i = 0;i < LED_BUFFER_LEN;i++) {
		uint32_t tmp_color = rgb_to_local(RGBdata[i]);

		for (bit = 0;bit < 24;bit++) {
			if(tmp_color & (1 << 23)) {
				bitbuffer[bit + i * 24] = WS2811_ONE;
			} else {
				bitbuffer[bit + i * 24] = WS2811_ZERO;
			}
			tmp_color <<= 1;
		}
	}

	// Fill the rest of the buffer with zeros to give the LEDs a chance to update
	// after sending all bits
	for (i = 0;i < BITBUFFER_PAD;i++) {
		bitbuffer[BITBUFFER_LEN - BITBUFFER_PAD - 1 + i] = 0;
	}

	// Generate gamma correction table
	for (i = 0;i < 256;i++) {
		gamma_table[i] = (int)roundf(powf((float)i / 255.0, 1.0 / 0.45) * 255.0);
	}

#if WS2811_USE_CH2
	palSetPadMode(GPIOB, 7,
			PAL_MODE_ALTERNATE(LL_GPIO_AF_2) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);
#else
	palSetPadMode(GPIOB, 6,
			PAL_MODE_ALTERNATE(LL_GPIO_AF_2) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);
#endif

	// DMA clock enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

#if WS2811_USE_CH2
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_3);
	DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM4->CCR2;
#else
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_0);
	DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&TIM4->CCR1;
#endif
	DMA_InitStructure.Channel = LL_DMA_CHANNEL_2;
	DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)bitbuffer;
	DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	DMA_InitStructure.NbData = BITBUFFER_LEN;
	DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
	DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
	DMA_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
	DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
	DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
	DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL;
	DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
	DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;

#if WS2811_USE_CH2
	LL_DMA_Init(DMA1, LL_DMA_STREAM_3, &DMA_InitStructure);
#else
	LL_DMA_Init(DMA1, LL_DMA_STREAM_0, &DMA_InitStructure);
#endif

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

	// Time Base configuration
	TIM_TimeBaseStructure.Prescaler = 0;
	TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_TimeBaseStructure.Autoreload = TIM_PERIOD;
	TIM_TimeBaseStructure.ClockDivision = 0;
	TIM_TimeBaseStructure.RepetitionCounter = 0;

	LL_TIM_Init(TIM4, &TIM_TimeBaseStructure);

	// Channel 1 Configuration in PWM mode
	TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OCInitStructure.CompareValue = bitbuffer[0];
	TIM_OCInitStructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;

#if WS2811_USE_CH2
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OCInitStructure);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
#else
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
#endif

	// TIM4 counter enable
	LL_TIM_EnableCounter(TIM4);

	// DMA enable
#if WS2811_USE_CH2
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
#else
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
#endif

	// TIM4 Update DMA Request enable
#if WS2811_USE_CH2
	LL_TIM_EnableDMAReq_CC2(TIM4);
#else
	LL_TIM_EnableDMAReq_CC1(TIM4);
#endif

	// Main Output Enable
	LL_TIM_EnableAllOutputs(TIM4);
}

void ws2811_set_led_color(int led, uint32_t color) {
	if (led >= 0 && led < WS2811_LED_NUM) {
		RGBdata[led] = color;

		color = rgb_to_local(color);

		int bit;
		for (bit = 0;bit < 24;bit++) {
			if(color & (1 << 23)) {
				bitbuffer[bit + led * 24] = WS2811_ONE;
			} else {
				bitbuffer[bit + led * 24] = WS2811_ZERO;
			}
			color <<= 1;
		}
	}
}

uint32_t ws2811_get_led_color(int led) {
	if (led >= 0 && led < WS2811_LED_NUM) {
		return RGBdata[led];
	}

	return 0;
}

void ws2811_all_off(void) {
	int i;

	for (i = 0;i < WS2811_LED_NUM;i++) {
		RGBdata[i] = 0;
	}

	for (i = 0;i < (WS2811_LED_NUM * 24);i++) {
		bitbuffer[i] = WS2811_ZERO;
	}
}

void ws2811_set_all(uint32_t color) {
	int i, bit;

	for (i = 0;i < WS2811_LED_NUM;i++) {
		RGBdata[i] = color;

		uint32_t tmp_color = rgb_to_local(color);

		for (bit = 0;bit < 24;bit++) {
			if(tmp_color & (1 << 23)) {
				bitbuffer[bit + i * 24] = WS2811_ONE;
			} else {
				bitbuffer[bit + i * 24] = WS2811_ZERO;
			}
			tmp_color <<= 1;
		}
	}
}

void ws2811_set_brightness(uint32_t br) {
	brightness = br;

	for (int i = 0;i < WS2811_LED_NUM;i++) {
		ws2811_set_led_color(i, ws2811_get_led_color(i));
	}
}

uint32_t ws2811_get_brightness(void) {
	return brightness;
}

static uint32_t rgb_to_local(uint32_t color) {
	uint32_t r = (color >> 16) & 0xFF;
	uint32_t g = (color >> 8) & 0xFF;
	uint32_t b = color & 0xFF;

	r = (r * brightness) / 100;
	g = (g * brightness) / 100;
	b = (b * brightness) / 100;

	r = gamma_table[r];
	g = gamma_table[g];
	b = gamma_table[b];

	return (g << 16) | (r << 8) | b;
}
