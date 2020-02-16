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

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "hw.h"
#include "encoder.h"
#include "stm32f4xx_ll_tim.h"

#define USE_ST_LL

CH_IRQ_HANDLER(ADC1_2_3_IRQHandler) {
	CH_IRQ_PROLOGUE();
	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
	mc_interface_adc_inj_int_handler();
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(TIM8_CC_IRQHandler) {
#ifdef USE_OLD
	if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) {
		mcpwm_foc_tim_sample_int_handler();

		// Clear the IT pending bit
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
	}
#elif defined(USE_ST_LL)
	if(LL_TIM_IsActiveFlag_CC1(TIM8) == 1) {
		mcpwm_foc_tim_sample_int_handler();

		// Clear the update interrupt flag
		LL_TIM_ClearFlag_CC1(TIM8);
	}
#endif
}

CH_IRQ_HANDLER(PVD_IRQHandler) {
	if (EXTI_GetITStatus(EXTI_Line16) != RESET) {
		// Log the fault. Supply voltage dropped below 2.9V,
		// could corrupt an ongoing flash programming
		mc_interface_fault_stop(FAULT_CODE_MCU_UNDER_VOLTAGE);

		// Clear the PVD pending bit
		EXTI_ClearITPendingBit(EXTI_Line16);
		EXTI_ClearFlag(EXTI_Line16);
	}
}
