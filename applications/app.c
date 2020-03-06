/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"

// Private variables
static app_configuration appconf;
//static virtual_timer_t output_vt;
static bool output_vt_init_done = false;
static volatile bool output_disabled_now = false;

// Private functions
static void output_vt_cb(void *arg);

const app_configuration* app_get_configuration(void) {
	return &appconf;
}

/**
 * Reconfigure and restart all apps. Some apps don't have any configuration options.
 *
 * @param conf
 * The new configuration to use.
 */
void app_set_configuration(app_configuration *conf) {
	appconf = *conf;

	app_uartcomm_stop();

#ifdef APP_CUSTOM_TO_USE
	app_custom_stop();
#endif

	switch (appconf.app_to_use) {
	case APP_UART:
		app_uartcomm_start();
		break;
	default:
		break;
	}

	app_uartcomm_configure(appconf.app_uart_baudrate, appconf.permanent_uart_enabled);
}

/**
 * Disable output on apps
 *
 * @param time_ms
 * The amount of time to disable output in ms
 * 0: Enable output now
 * -1: Disable forever
 * >0: Amount of milliseconds to disable output
 */
void app_disable_output(int time_ms) {
	/*
	if (!output_vt_init_done) {
		chVTObjectInit(&output_vt);
		output_vt_init_done = true;
	}
	*/

	if (time_ms == 0) {
		output_disabled_now = false;
	} 
	/*else if (time_ms == -1) {
		output_disabled_now = true;
		chVTReset(&output_vt);
	} else {
		output_disabled_now = true;
		chVTSet(&output_vt, TIME_MS2I(time_ms), output_vt_cb, 0);
	}
	*/
}

bool app_is_output_disabled(void) {
	return output_disabled_now;
}

static void output_vt_cb(void *arg) {
	(void)arg;
	output_disabled_now = false;
}
