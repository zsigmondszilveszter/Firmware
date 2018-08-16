/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "calibration_tunes.h"

#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_hrt.h>

namespace calibration
{

static hrt_abstime tune_end = 0;		// end time of currently played tune, 0 for repeating tunes or silence
static int tune_current = TONE_STOP_TUNE;		// currently playing tune, can be interrupted after tune_end
static unsigned int tune_durations[TONE_NUMBER_OF_TUNES];

static tune_control_s tune_control = {};
static orb_advert_t tune_control_pub = nullptr;

// blinks?
static orb_advert_t led_control_pub = nullptr;
#include <uORB/topics/led_control.h>
static hrt_abstime blink_msg_end = 0;	// end time for currently blinking LED message, 0 if no blink message
#define BLINK_MSG_TIME	700000	// 3 fast blinks (in us)

void set_tune(int tune)
{
	unsigned int new_tune_duration = tune_durations[tune];

	/* don't interrupt currently playing non-repeating tune by repeating */
	if (tune_end == 0 || new_tune_duration != 0 || hrt_absolute_time() > tune_end) {
		/* allow interrupting current non-repeating tune by the same tune */
		if (tune != tune_current || new_tune_duration != 0) {
			tune_control.tune_id = tune;
			tune_control.strength = tune_control_s::STRENGTH_NORMAL;
			tune_control.tune_override = 0;
			tune_control.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
		}

		tune_current = tune;

		if (new_tune_duration != 0) {
			tune_end = hrt_absolute_time() + new_tune_duration;

		} else {
			tune_end = 0;
		}
	}
}

void tune_positive()
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_GREEN, led_control_s::MODE_BLINK_FAST);

	set_tune(TONE_NOTIFY_POSITIVE_TUNE);
}

void tune_negative()
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_RED, led_control_s::MODE_BLINK_FAST);

	set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
}

void tune_neutral()
{
	blink_msg_end = hrt_absolute_time() + BLINK_MSG_TIME;
	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_BLINK_FAST);

	set_tune(TONE_NOTIFY_NEUTRAL_TUNE);
}


// lights

void rgbled_set_color_and_mode(uint8_t color, uint8_t mode, uint8_t blinks, uint8_t prio)
{
	led_control_s led_control = {};

	led_control.mode = mode;
	led_control.color = color;
	led_control.num_blinks = blinks;
	led_control.priority = prio;
	led_control.timestamp = hrt_absolute_time();

	orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
}

void rgbled_set_color_and_mode(uint8_t color, uint8_t mode)
{
	rgbled_set_color_and_mode(color, mode, 0, 0);
}

} // namespace calibration
