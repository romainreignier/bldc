/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "timer.h"
#include "ch.h"
#include "hal.h"

// Settings
#define TIMER_HZ					1e7

// GPT5 configuration.
static const GPTConfig gpt5cfg = {
  TIMER_HZ, // Time frequency
  NULL, // Callback
  0, // CR2 register
  0 // DIER register
};

void timer_init(void) {
	gptStart(&GPTD5, &gpt5cfg);
	gptStartContinuous(&GPTD5, 0xFFFFFFFF);
}

uint32_t timer_time_now(void) {
	return gptGetCounterX(&GPTD5);
}

float timer_seconds_elapsed_since(uint32_t time) {
	uint32_t diff = gptGetCounterX(&GPTD5) - time;
	return (float)diff / (float)TIMER_HZ;
}

/**
 * Blocking sleep based on timer.
 *
 * @param seconds
 * Seconds to sleep.
 */
void timer_sleep(float seconds) {
	uint32_t start_t = gptGetCounterX(&GPTD5);

	for (;;) {
		if (timer_seconds_elapsed_since(start_t) >= seconds) {
			return;
		}
	}
}
