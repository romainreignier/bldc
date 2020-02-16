/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * hw_victor_r1a.c
 *
 *  Created on: 14 apr 2014
 *      Author: benjamin
 */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	// LEDs
	palSetPadMode(GPIOC, 4,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOA, 7,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// GPIOC (ENABLE_GATE)
	palSetPadMode(GPIOC, 10,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	DISABLE_GATE();

	// GPIOB (DCCAL)
	palSetPadMode(GPIOB, 12,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(LL_GPIO_AF_1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(LL_GPIO_AF_1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(LL_GPIO_AF_1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(LL_GPIO_AF_1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(LL_GPIO_AF_1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(LL_GPIO_AF_1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUPDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Fault pin
	palSetPadMode(GPIOC, 12, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_5);
	LL_ADC_SetChannelSamplingTime(ADC1, ADC_Channel_Vrefint, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, ADC_Channel_Vrefint);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_13, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_13);

	// ADC2 regular channels
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_6);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_11);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_15);

	// ADC3 regular channels
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_3);
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_12);
	LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_10, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_10);

	// Injected channels
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_5);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_6);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_6);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_15CYCLES);
	LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_5);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUPDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}
