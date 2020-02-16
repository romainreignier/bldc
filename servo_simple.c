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

#include "servo_simple.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "utils.h"

// Settings
#define TIM_CLOCK			1000000 // Hz

#if SERVO_OUT_ENABLE

void servo_simple_init(void) {
	LL_TIM_InitTypeDef  TIM_TimeBaseStructure;
	LL_TIM_OC_InitTypeDef  TIM_OCInitStructure;

	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_ALTERNATE(HW_ICU_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_FLOATING);

	HW_ICU_TIM_CLK_EN();

	TIM_TimeBaseStructure.Autoreload = (uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_OUT_RATE_HZ);
	TIM_TimeBaseStructure.Prescaler = (uint16_t)((168000000 / 2) / TIM_CLOCK) - 1;
	TIM_TimeBaseStructure.ClockDivision = 0;
	TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;

	LL_TIM_Init(HW_ICU_TIMER, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OCInitStructure.CompareValue = 0;
	TIM_OCInitStructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;

	if (HW_ICU_CHANNEL == ICU_CHANNEL_1) {
		LL_TIM_OC_Init(HW_ICU_TIMER, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
		LL_TIM_OC_EnablePreload(HW_ICU_TIMER, LL_TIM_CHANNEL_CH1);
	} else if (HW_ICU_CHANNEL == ICU_CHANNEL_2) {
		LL_TIM_OC_Init(HW_ICU_TIMER, LL_TIM_CHANNEL_CH2, &TIM_OCInitStructure);
		LL_TIM_OC_EnablePreload(HW_ICU_TIMER, LL_TIM_CHANNEL_CH2);
	}

	LL_TIM_EnableARRPreload(HW_ICU_TIMER);

	servo_simple_set_output(0.5);

	LL_TIM_EnableCounter(HW_ICU_TIMER);
}

void servo_simple_set_output(float out) {
	utils_truncate_number(&out, 0.0, 1.0);

	float us = (float)SERVO_OUT_PULSE_MIN_US + out *
			(float)(SERVO_OUT_PULSE_MAX_US - SERVO_OUT_PULSE_MIN_US);
	us *= (float)TIM_CLOCK / 1000000.0;

	if (HW_ICU_CHANNEL == ICU_CHANNEL_1) {
		HW_ICU_TIMER->CCR1 = (uint32_t)us;
	} else if (HW_ICU_CHANNEL == ICU_CHANNEL_2) {
		HW_ICU_TIMER->CCR2 = (uint32_t)us;
	}
}

#endif
