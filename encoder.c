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

#include "encoder.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>

// Defines
#define AS5047P_READ_ANGLECOM		(0x3FFF | 0x4000 | 0x8000) // This is just ones
#define AS5047_SAMPLE_RATE_HZ		20000

#if (AS5047_USE_HW_SPI_PINS || AD2S1205_USE_HW_SPI_PINS)
#ifdef HW_SPI_DEV
#define SPI_SW_MISO_GPIO			HW_SPI_PORT_MISO
#define SPI_SW_MISO_PIN				HW_SPI_PIN_MISO
#define SPI_SW_MOSI_GPIO			HW_SPI_PORT_MOSI
#define SPI_SW_MOSI_PIN				HW_SPI_PIN_MOSI
#define SPI_SW_SCK_GPIO				HW_SPI_PORT_SCK
#define SPI_SW_SCK_PIN				HW_SPI_PIN_SCK
#define SPI_SW_CS_GPIO				HW_SPI_PORT_NSS
#define SPI_SW_CS_PIN				HW_SPI_PIN_NSS
#else
// Note: These values are hardcoded.
#define SPI_SW_MISO_GPIO			GPIOB
#define SPI_SW_MISO_PIN				4
#define SPI_SW_MOSI_GPIO			GPIOB
#define SPI_SW_MOSI_PIN				5
#define SPI_SW_SCK_GPIO				GPIOB
#define SPI_SW_SCK_PIN				3
#define SPI_SW_CS_GPIO				GPIOB
#define SPI_SW_CS_PIN				0
#endif
#else
#define SPI_SW_MISO_GPIO			HW_HALL_ENC_GPIO2
#define SPI_SW_MISO_PIN				HW_HALL_ENC_PIN2
#define SPI_SW_SCK_GPIO				HW_HALL_ENC_GPIO1
#define SPI_SW_SCK_PIN				HW_HALL_ENC_PIN1
#define SPI_SW_CS_GPIO				HW_HALL_ENC_GPIO3
#define SPI_SW_CS_PIN				HW_HALL_ENC_PIN3
#endif

// Private types
typedef enum {
	ENCODER_MODE_NONE = 0,
	ENCODER_MODE_ABI,
	ENCODER_MODE_AS5047P_SPI
} encoder_mode;

// Private variables
static bool index_found = false;
static uint32_t enc_counts = 10000;
static encoder_mode mode = ENCODER_MODE_NONE;
static float last_enc_angle = 0.0;
static uint32_t spi_val = 0;
static uint32_t spi_error_cnt = 0;
static float spi_error_rate = 0.0;

// Private functions
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

uint32_t encoder_spi_get_error_cnt(void) {
	return spi_error_cnt;
}

uint32_t encoder_spi_get_val(void) {
	return spi_val;
}

float encoder_spi_get_error_rate(void) {
	return spi_error_rate;
}

void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	LL_TIM_DeInit(HW_ENC_TIM);

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_INPUT_PULLUP);

	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);

	index_found = false;
	mode = ENCODER_MODE_NONE;
	last_enc_angle = 0.0;
	spi_error_rate = 0.0;
}

void encoder_init_abi(uint32_t counts) {
	LL_EXTI_InitTypeDef   EXTI_InitStructure;

	// Initialize variables
	index_found = false;
	enc_counts = counts;

	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));
//	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_ALTERNATE(HW_ENC_TIM_AF));

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Enable SYSCFG clock
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	LL_TIM_SetEncoderMode(HW_ENC_TIM, LL_TIM_ENCODERMODE_X4_TI12);
	LL_TIM_IC_SetActiveInput(HW_ENC_TIM, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetActiveInput(HW_ENC_TIM, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPolarity(HW_ENC_TIM, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetPolarity(HW_ENC_TIM, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_SetAutoReload(HW_ENC_TIM, enc_counts - 1);

	// Filter
	HW_ENC_TIM->CCMR1 |= 6 << 12 | 6 << 4;
	HW_ENC_TIM->CCMR2 |= 6 << 4;

	LL_TIM_EnableCounter(HW_ENC_TIM);

	// Interrupt on index pulse

	// Connect EXTI Line to pin
	LL_SYSCFG_SetEXTISource(HW_ENC_EXTI_PORTSRC, HW_ENC_EXTI_PINSRC);

	// Configure EXTI Line
	EXTI_InitStructure.Line_0_31 = HW_ENC_EXTI_LINE;
	EXTI_InitStructure.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStructure.Trigger = LL_EXTI_TRIGGER_RISING;
	EXTI_InitStructure.LineCommand = ENABLE;
	LL_EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Line Interrupt to the highest priority
	nvicEnableVector(HW_ENC_EXTI_CH, 0);

	mode = ENCODER_MODE_ABI;
}

void encoder_init_as5047p_spi(void) {
	LL_TIM_InitTypeDef  TIM_TimeBaseStructure;

	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1
#if (AS5047_USE_HW_SPI_PINS || AD2S1205_USE_HW_SPI_PINS)
	palSetPadMode(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN);
#endif

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.Prescaler = 0;
	TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_TimeBaseStructure.Autoreload = ((168000000 / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.ClockDivision = 0;
	TIM_TimeBaseStructure.RepetitionCounter = 0;
	LL_TIM_Init(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	LL_TIM_EnableIT_UPDATE(HW_ENC_TIM);

	// Enable timer
	LL_TIM_EnableCounter(HW_ENC_TIM);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	mode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;
	spi_error_rate = 0.0;
}

bool encoder_is_configured(void) {
	return mode != ENCODER_MODE_NONE;
}

/**
 * Read angle from configured encoder.
 *
 * @return
 * The current encoder angle in degrees.
 */
float encoder_read_deg(void) {
	static float angle = 0.0;

	switch (mode) {
	case ENCODER_MODE_ABI:
		angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
		break;

	case ENCODER_MODE_AS5047P_SPI:
		angle = last_enc_angle;
		break;

	default:
		break;
	}

	return angle;
}

/**
 * Reset the encoder counter. Should be called from the index interrupt.
 */
void encoder_reset(void) {
	// Only reset if the pin is still high to avoid too short pulses, which
	// most likely are noise.
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	if (palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)) {
		const unsigned int cnt = HW_ENC_TIM->CNT;
		static int bad_pulses = 0;
		const unsigned int lim = enc_counts / 20;

		if (index_found) {
			// Some plausibility filtering.
			if (cnt > (enc_counts - lim) || cnt < lim) {
				HW_ENC_TIM->CNT = 0;
				bad_pulses = 0;
			} else {
				bad_pulses++;

				if (bad_pulses > 5) {
					index_found = 0;
				}
			}
		} else {
			HW_ENC_TIM->CNT = 0;
			index_found = true;
			bad_pulses = 0;
		}
	}
}

// returns true for even number of ones (no parity error according to AS5047 datasheet
bool spi_check_parity(uint16_t x) {
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return (~x) & 1;
}

/**
 * Timer interrupt
 */
void encoder_tim_isr(void) {
	uint16_t pos;

	if(mode == ENCODER_MODE_AS5047P_SPI) {
		spi_begin();
		spi_transfer(&pos, 0, 1);
		spi_end();

		spi_val = pos;
		if(spi_check_parity(pos) && pos != 0xffff) {  // all ones = disconnect
			pos &= 0x3FFF;
			last_enc_angle = ((float)pos * 360.0) / 16384.0;
			UTILS_LP_FAST(spi_error_rate, 0.0, 1./AS5047_SAMPLE_RATE_HZ);
		} else {
			++spi_error_cnt;
			UTILS_LP_FAST(spi_error_rate, 1.0, 1./AS5047_SAMPLE_RATE_HZ);
		}		
	}
}

/**
 * Set the number of encoder counts.
 *
 * @param counts
 * The number of encoder counts
 */
void encoder_set_counts(uint32_t counts) {
	if (counts != enc_counts) {
		enc_counts = counts;
		LL_TIM_SetAutoReload(HW_ENC_TIM, enc_counts - 1);
		index_found = false;
	}
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

// Software SPI
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t receive = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			palSetPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			int samples = 0;
			samples += palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			samples += palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			samples += palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			samples += palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			samples += palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);

			receive <<= 1;
			if (samples > 2) {
				receive |= 1;
			}

			palClearPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

static void spi_begin(void) {
	palClearPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_end(void) {
	palSetPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}
