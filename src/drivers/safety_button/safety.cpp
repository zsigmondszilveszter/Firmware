/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#include <cfloat>

#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <circuit_breaker/circuit_breaker.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>

using namespace time_literals;

#define SCHEDULE_INTERVAL	100000	/**< The schedule interval in usec (10 Hz) */

static constexpr uint8_t CYCLE_COUNT = 10; /* safety switch must be held for 1 second to activate */

/*
 * Define the various LED flash sequences for each system state.
 */
#define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
#define LED_PATTERN_FMU_REFUSE_TO_ARM 		0x5555		/**< fast blinking			*/
#define LED_PATTERN_IO_ARMED 			0x5050		/**< long off, then double blink 	*/
#define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
#define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

class SafetyButton : public ModuleBase<SafetyButton>
{
public:
	SafetyButton() = default;
	virtual ~SafetyButton();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * run the main loop: execute only one single cycle
	 */
	void cycle();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	hrt_abstime _last_safety_check{0};

	static struct work_s	_work;

	int		_armed_sub{-1};

	actuator_armed_s	_armed{};

	bool		_safety_off{false};			///< State of the safety button from the subscribed safety topic
	bool		_safety_btn_off{false};		///< State of the safety button read from the HW button

	orb_advert_t		_to_safety{nullptr};

	static void	cycle_trampoline(void *arg);
	int 		start();

	void safety_check_button(void);
	void flash_safety_button(void);

};

work_s SafetyButton::_work = {};

SafetyButton::~SafetyButton()
{
	orb_unsubscribe(_armed_sub);

	orb_unadvertise(_to_safety);
}

void
SafetyButton::safety_check_button(void)
{
	static int counter = 0;
	/*
	 * Debounce the safety button, change state if it has been held for long enough.
	 *
	 */
	bool safety_button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

	/*
	 * Keep pressed for a while to arm.
	 *
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
	 */
	if (safety_button_pressed && !_safety_btn_off) {

		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* switch to armed state */
			_safety_btn_off = true;
			counter++;
		}

	} else if (safety_button_pressed && _safety_btn_off) {

		if (counter < CYCLE_COUNT) {
			counter++;

		} else if (counter == CYCLE_COUNT) {
			/* change to disarmed state and notify the FMU */
			_safety_btn_off = false;
			counter++;
		}

	} else {
		counter = 0;
	}
}

void
SafetyButton::flash_safety_button()
{
	bool armed_updated = false;
	orb_check(_armed_sub, &armed_updated);

	if (armed_updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}

	/* Select the appropriate LED flash pattern depending on the current arm state */
	uint16_t pattern = LED_PATTERN_FMU_REFUSE_TO_ARM;

	/* cycle the blink state machine at 10Hz */
	static int blink_counter = 0;

	if (_safety_btn_off) {
		if (_armed.armed) {
			pattern = LED_PATTERN_IO_FMU_ARMED;

		} else {
			pattern = LED_PATTERN_IO_ARMED;
		}

	} else if (_armed.armed) {
		pattern = LED_PATTERN_FMU_ARMED;

	} else {
		pattern = LED_PATTERN_FMU_OK_TO_ARM;

	}

	/* Turn the LED on if we have a 1 at the current bit position */
	px4_arch_gpiowrite(GPIO_LED_SAFETY, !(pattern & (1 << blink_counter++)));

	if (blink_counter > 15) {
		blink_counter = 0;
	}
}

int
SafetyButton::task_spawn(int argc, char *argv[])
{
	if (PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		PX4_ERR("not starting (use px4io for safety button)");

		return PX4_ERROR;

	} else if (circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY)) {
		PX4_WARN("disabled by CBRK_IO_SAFETY, exiting");
		return PX4_ERROR;

	} else {
		/* schedule a cycle to start things */
		int ret = work_queue(HPWORK, &_work, (worker_t)&SafetyButton::cycle_trampoline, nullptr, 0);

		if (ret < 0) {
			return ret;
		}

		_task_id = task_id_is_work_queue;
	}

	return PX4_OK;
}

void
SafetyButton::cycle_trampoline(void *arg)
{
	SafetyButton *dev = reinterpret_cast<SafetyButton *>(arg);

	// check if the trampoline is called for the first time
	if (!dev) {
		dev = new SafetyButton();

		if (!dev) {
			PX4_ERR("alloc failed");
			return;
		}

		_object = dev;
	}

	dev->cycle();
}

void
SafetyButton::cycle()
{
	const hrt_abstime cycle_timestamp = hrt_absolute_time();

	if (cycle_timestamp - _last_safety_check >= 100_ms) {
		_last_safety_check = cycle_timestamp;

		/**
		 * Get and handle the safety status at 10Hz
		 */
		safety_s safety = {};

		/* read safety switch input and control safety switch LED at 10Hz */
		safety_check_button();

		/* Make the safety button flash anyway, no matter if it's used or not. */
		flash_safety_button();

		safety.timestamp = hrt_absolute_time();
		safety.safety_switch_available = true;
		safety.safety_off = _safety_btn_off;

		/* lazily publish the safety status */
		if (_to_safety != nullptr) {
			orb_publish(ORB_ID(safety), _to_safety, &safety);

		} else {
			int instance;
			_to_safety = orb_advertise_multi(ORB_ID(safety), &safety, &instance, ORB_PRIO_DEFAULT);
		}
	}

	if (should_exit()) {
		exit_and_cleanup();

	} else {
		/* schedule next cycle */
		work_queue(HPWORK, &_work, (worker_t)&SafetyButton::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
	}
}

int SafetyButton::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SafetyButton::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for the safety button.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safety_button", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the safety button driver");

	return 0;
}

int SafetyButton::print_status()
{
#ifdef GPIO_BTN_SAFETY
	if (!PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		PX4_INFO("Safety State (from button): %s", _safety_btn_off ? "off" : "on");
	}
#endif

	return 0;
}

extern "C" __EXPORT int safety_button_main(int argc, char *argv[]);

int
safety_button_main(int argc, char *argv[])
{
	return SafetyButton::main(argc, argv);
}
