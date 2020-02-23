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

CH_IRQ_HANDLER(ADC1_2_3_IRQHandler) {
	CH_IRQ_PROLOGUE();
	LL_ADC_ClearFlag_JEOS(ADC1);
	mc_interface_adc_inj_int_handler();
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(DMA2_Stream4_IRQHandler) {
	CH_IRQ_PROLOGUE();
	/* Check whether DMA transfer complete caused the DMA interruption */
	if(LL_DMA_IsActiveFlag_TC4(DMA2) == 1)
	{
		/* Call interruption treatment function */
		mc_interface_adc_complete_int_handler();

		/* Clear flag DMA transfer complete */
		LL_DMA_ClearFlag_TC4(DMA2);
	}

  CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(HW_ENC_EXTI_ISR_VEC) {
	if (LL_EXTI_IsActiveFlag_0_31(HW_ENC_EXTI_LINE) != RESET) {
		encoder_reset();

		// Clear the EXTI line pending bit
		LL_EXTI_ClearFlag_0_31(HW_ENC_EXTI_LINE);
	}
}

CH_IRQ_HANDLER(HW_ENC_TIM_ISR_VEC) {
	if ((LL_TIM_IsActiveFlag_UPDATE(HW_ENC_TIM) && LL_TIM_IsEnabledIT_UPDATE(HW_ENC_TIM)) != RESET) {
		encoder_tim_isr();

		// Clear the IT pending bit
		LL_TIM_ClearFlag_UPDATE(HW_ENC_TIM);
	}
}

CH_IRQ_HANDLER(TIM8_CC_IRQHandler) {
	if ((LL_TIM_IsActiveFlag_CC1(TIM8) && LL_TIM_IsEnabledIT_CC1(TIM8)) != RESET) {
		mcpwm_foc_tim_sample_int_handler();

		// Clear the IT pending bit
		LL_TIM_ClearFlag_CC1(TIM8);
	}
}

CH_IRQ_HANDLER(PVD_IRQHandler) {
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_16) != RESET) {
		// Log the fault. Supply voltage dropped below 2.9V,
		// could corrupt an ongoing flash programming
		mc_interface_fault_stop(FAULT_CODE_MCU_UNDER_VOLTAGE);

		// Clear the PVD pending bit
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_16);
	}
}
