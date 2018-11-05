/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file hmc5883.cpp
 *
 * Driver for the HMC5883 / HMC5983 magnetometer connected via I2C or SPI.
 */

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <lib/conversion/rotation.h>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_workqueue.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/uORB.h>

#include <getopt.h>
#include <string.h>

#include "hmc5883.h"

/*
 * HMC5883 internal constants and data structures.
 */

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define HMC5883_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */

#define ADDR_CONF_A			0x00
#define ADDR_CONF_B			0x01
#define ADDR_MODE			0x02
#define ADDR_DATA_OUT_X_MSB		0x03
#define ADDR_DATA_OUT_X_LSB		0x04
#define ADDR_DATA_OUT_Z_MSB		0x05
#define ADDR_DATA_OUT_Z_LSB		0x06
#define ADDR_DATA_OUT_Y_MSB		0x07
#define ADDR_DATA_OUT_Y_LSB		0x08
#define ADDR_STATUS			0x09

/* temperature on hmc5983 only */
#define ADDR_TEMP_OUT_MSB		0x31
#define ADDR_TEMP_OUT_LSB		0x32

/* modes not changeable outside of driver */
#define HMC5883L_MODE_NORMAL		(0 << 0)  /* default */
#define HMC5883L_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5883L_MODE_NEGATIVE_BIAS	(1 << 1)  /* negative bias */

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

#define HMC5983_TEMP_SENSOR_ENABLE	(1 << 7)

// Configuration Register A
// TS | MA1 | MA0 | DO2 | DO1 | DO0 | MS1 | MS0

static constexpr uint8_t HMC5983_CONFIG_TEMP_SENSOR_ENABLE = (1 << 7);

static constexpr uint8_t HMC5983_CONFIG_A_TS = (1 << 7);
static constexpr uint8_t HMC5983_CONFIG_A_MA = (1 << 5);
static constexpr uint8_t HMC5983_CONFIG_A_DO = (1 << 2);
static constexpr uint8_t HMC5983_CONFIG_A_MS = (1 << 0);


static constexpr uint8_t HMC5983_CONFIG_SAMPLES_AVG_0 = (0 << 5);
static constexpr uint8_t HMC5983_CONFIG_SAMPLES_AVG_2 = (1 << 5);
static constexpr uint8_t HMC5983_CONFIG_SAMPLES_AVG_4 = (2 << 5);
static constexpr uint8_t HMC5983_CONFIG_SAMPLES_AVG_8 = (3 << 5);

enum HMC5883_BUS {
	HMC5883_BUS_ALL = 0,
	HMC5883_BUS_I2C_INTERNAL,
	HMC5883_BUS_I2C_EXTERNAL,
	HMC5883_BUS_SPI
};

class HMC5883 : public cdev::CDev
{
public:
	HMC5883(device::Device *interface, const char *path, enum Rotation rotation);
	~HMC5883() override;

	int				init() override;

	int				ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	device::Device			*_interface;

private:
	work_s			_work{};
	unsigned		_measure_ticks{0};

	mag_calibration_s	_scale{};

	float 			_range_scale{0.0f};
	float 			_range_ga{1.9f};

	bool			_collect_phase{false};

	int			_class_instance{-1};
	int			_orb_class_instance{-1};

	orb_advert_t		_mag_topic{nullptr};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok{false};		/**< sensor was found and reports ok */

	enum Rotation		_rotation;

	sensor_mag_s		_last_report{};         /**< used for info() */

	uint8_t			_range_bits{0};
	uint8_t			_conf_reg{0};
	uint8_t			_temperature_counter{0};
	uint8_t			_temperature_error_count{0};

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 */
	int			calibrate();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(bool enable);

	/**
	 * enable hmc5983 temperature compensation
	 */
	int			set_temperature_compensation(bool enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * check the sensor range.
	 *
	 * checks that the range of the sensor is correctly set, to
	 * cope with communication errors causing the range to change
	 */
	void 			check_range();

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]) const;

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hmc5883_main(int argc, char *argv[]);

HMC5883::HMC5883(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev(path),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, "hmc5883_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hmc5883_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "hmc5883_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "hmc5883_conf_err")),
	_rotation(rotation)
{
	_interface->set_device_type(DRV_MAG_DEVTYPE_HMC5883);

	// default scaling
	_scale.x_offset = 0.0f;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0.0f;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0.0f;
	_scale.z_scale = 1.0f;
}

HMC5883::~HMC5883()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
HMC5883::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_DEBUG("CDev init failed");
		return ret;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	/* sensor is ok, but not calibrated */
	_sensor_ok = true;

	return PX4_OK;
}

int HMC5883::set_range(unsigned range)
{
	if (range < 0.88f) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1.3f) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return !(range_bits_in == (_range_bits << 5));
}

/**
   check that the range register has the right value. This is done
   periodically to cope with I2C bus noise causing the range of the
   compass changing.
 */
void HMC5883::check_range()
{
	int ret;

	uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (range_bits_in != (_range_bits << 5)) {
		perf_count(_range_errors);
		ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void HMC5883::check_conf()
{
	int ret;

	uint8_t conf_reg_in = 0;
	ret = read_reg(ADDR_CONF_A, conf_reg_in);

	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}

	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CONF_A, _conf_reg);

		if (OK != ret) {
			perf_count(_comms_errors);
		}
	}
}

int
HMC5883::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(HMC5883_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(HMC5883_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (struct mag_calibration_s *)arg, sizeof(_scale));
		PX4_WARN("SC: (%.3f, %.3f, %.3f) OF: (%.3f, %.3f, %.3f) ", (double)_scale.x_scale, (double)_scale.y_scale,
			 (double)_scale.z_scale, (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
		return 0;

	case DEVIOCGDEVICEID:
		return _interface->get_device_id();

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
HMC5883::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HMC5883::cycle_trampoline, this, 1);
}

void
HMC5883::stop()
{
	if (_measure_ticks > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_ticks = 0;
		work_cancel(HPWORK, &_work);
	}
}

int
HMC5883::reset()
{
	/* set range, ceil floating point number */
	return set_range(_range_ga + 0.5f);
}

void
HMC5883::cycle_trampoline(void *arg)
{
	HMC5883 *dev = (HMC5883 *)arg;

	dev->cycle();
}

void
HMC5883::cycle()
{
	if (_measure_ticks == 0) {
		return;
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (PX4_OK != collect()) {
			PX4_DEBUG("collection error");

			/* restart the measurement state machine */
			start();

			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(HMC5883_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK, &_work, (worker_t)&HMC5883::cycle_trampoline, this,
				   _measure_ticks - USEC2TICK(HMC5883_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_ticks > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK, &_work, (worker_t)&HMC5883::cycle_trampoline, this, USEC2TICK(HMC5883_CONVERSION_INTERVAL));
	}
}

int
HMC5883::measure()
{
	/*
	 * Send the command to begin a measurement.
	 */
	int ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
HMC5883::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	}	hmc_report;
#pragma pack(pop)

	perf_begin(_sample_perf);

	sensor_mag_s new_report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);
	new_report.scaling = _range_scale;
	new_report.device_id = _interface->get_device_id();

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	int ret = _interface->read(ADDR_DATA_OUT_X_MSB, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("data/status read error");

		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	struct {
		int16_t	x;
		int16_t y;
		int16_t z;
	} report;

	/* swap the data we just received */
	report.x = (((int16_t)hmc_report.x[0]) << 8) + hmc_report.x[1];
	report.y = (((int16_t)hmc_report.y[0]) << 8) + hmc_report.y[1];
	report.z = (((int16_t)hmc_report.z[0]) << 8) + hmc_report.z[1];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 2048) ||
	    (abs(report.y) > 2048) ||
	    (abs(report.z) > 2048)) {

		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	/* get measurements from the device */
	new_report.temperature = 0;

	if (_conf_reg & HMC5983_TEMP_SENSOR_ENABLE) {
		/*
		  if temperature compensation is enabled read the
		  temperature too.

		  We read the temperature every 10 samples to avoid
		  excessive I2C traffic
		 */
		if (_temperature_counter++ == 10) {
			uint8_t raw_temperature[2];

			_temperature_counter = 0;

			ret = _interface->read(ADDR_TEMP_OUT_MSB, raw_temperature, sizeof(raw_temperature));

			if (ret == PX4_OK) {
				int16_t temp16 = (((int16_t)raw_temperature[0]) << 8) + raw_temperature[1];
				new_report.temperature = 25 + (temp16 / (16 * 8.0f));
				_temperature_error_count = 0;

			} else {
				_temperature_error_count++;

				if (_temperature_error_count == 10) {
					/*
					  it probably really is an old HMC5883,
					  and can't do temperature. Disable it
					*/
					_temperature_error_count = 0;
					PX4_DEBUG("disabling temperature compensation");
					set_temperature_compensation(false);
				}
			}

		} else {
			new_report.temperature = _last_report.temperature;
		}
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	new_report.x_raw = -report.y;
	new_report.y_raw = report.x;
	new_report.z_raw = report.z; /* z remains z */

	new_report.is_external = _interface->external();

	if (new_report.is_external) {
		// convert onboard so it matches offboard for the
		// scaling below
		report.y = -report.y;
		report.x = -report.x;
	}

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	float xraw_f = -report.y;
	float yraw_f = report.x;
	float zraw_f = report.z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	orb_publish_auto(ORB_ID(sensor_mag), &_mag_topic, &new_report, &_orb_class_instance,
			 new_report.is_external ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

	_last_report = new_report;

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	uint8_t check_counter = perf_event_count(_sample_perf) % 256;

	if (check_counter == 0) {
		check_range();

	} else if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the HMC5883L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
int HMC5883::calibrate()
{
	int ret = PX4_OK;
	uint8_t good_count = 0;

	mag_calibration_s mscale_previous{};
	mscale_previous.x_offset = 0.0f;
	mscale_previous.x_scale = 1.0f;
	mscale_previous.y_offset = 0.0f;
	mscale_previous.y_scale = 1.0f;
	mscale_previous.z_offset = 0.0f;
	mscale_previous.z_scale = 1.0f;

	mag_calibration_s mscale_null{};
	mscale_null.x_offset = 0.0f;
	mscale_null.x_scale = 1.0f;
	mscale_null.y_offset = 0.0f;
	mscale_null.y_scale = 1.0f;
	mscale_null.z_offset = 0.0f;
	mscale_null.z_scale = 1.0f;

	float sum_excited[3] = {0.0f, 0.0f, 0.0f};

	/* expected axis scaling. The datasheet says that 766 will
	 * be places in the X and Y axes and 713 in the Z
	 * axis. Experiments show that in fact 766 is placed in X,
	 * and 713 in Y and Z. This is relative to a base of 660
	 * LSM/Ga, giving 1.16 and 1.08 */
	const float expected_cal[3] = { 1.16f, 1.08f, 1.08f };

	/* start the sensor polling at 50 Hz */
//	if (px4_ioctl(filp, SENSORIOCSPOLLRATE, 50) != PX4_OK) {
//		warn("FAILED: SENSORIOCSPOLLRATE 50Hz");
//		ret = 1;
//		goto out;
//	}

	/* Set to 2.5 Gauss. We ask for 3 to get the right part of
	 * the chained if statement above. */
//	if (OK != ioctl(filp, MAGIOCSRANGE, 3)) {
//		warnx("FAILED: MAGIOCSRANGE 2.5 Ga");
//		ret = 1;
//		goto out;
//	}

//	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
//		warnx("FAILED: MAGIOCEXSTRAP 1");
//		ret = 1;
//		goto out;
//	}

//	if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
//		warn("FAILED: MAGIOCGSCALE 1");
//		ret = 1;
//		goto out;
//	}

//	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
//		warn("FAILED: MAGIOCSSCALE 1");
//		ret = 1;
//		goto out;
//	}

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = -1;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 1");
			goto out;
		}

		/* now go get it */
		sensor_mag_s report{};
		ssize_t sz = px4_read(-1, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("READ 1");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 150x, stopping when we have 50 good values */
	for (uint8_t i = 0; i < 150 && good_count < 50; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = -1;
		fds.events = POLLIN;
		ret = px4_poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("ERROR: TIMEOUT 2");
			goto out;
		}

		/* now go get it */
		sensor_mag_s report{};
		ssize_t sz = px4_read(-1, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("ERROR: READ 2");
			ret = -EIO;
			goto out;
		}

		float cal[3] = {fabsf(expected_cal[0] / report.x),
				fabsf(expected_cal[1] / report.y),
				fabsf(expected_cal[2] / report.z)
			       };

		if (cal[0] > 0.3f && cal[0] < 1.7f &&
		    cal[1] > 0.3f && cal[1] < 1.7f &&
		    cal[2] > 0.3f && cal[2] < 1.7f) {

			good_count++;
			sum_excited[0] += cal[0];
			sum_excited[1] += cal[1];
			sum_excited[2] += cal[2];
		}
	}

	if (good_count < 5) {
		ret = -EIO;
		goto out;
	}

	float scaling[3];

	scaling[0] = sum_excited[0] / good_count;
	scaling[1] = sum_excited[1] / good_count;
	scaling[2] = sum_excited[2] / good_count;

	/* set scaling in device */
	mscale_previous.x_scale = 1.0f / scaling[0];
	mscale_previous.y_scale = 1.0f / scaling[1];
	mscale_previous.z_scale = 1.0f / scaling[2];

	PX4_WARN("new scale X: %.3f Y: %.3f Z: %.3f",
		 (double)mscale_previous.x_scale, (double)mscale_previous.y_scale, (double)mscale_previous.z_scale);

	ret = OK;

out:

//	if (OK != px4_ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
//		warn("FAILED: MAGIOCSSCALE 2");
//	}

	/* set back to normal mode */
	/* Set to 1.9 Gauss */
//	if (OK != ::ioctl(fd, MAGIOCSRANGE, 2)) {
//		warnx("FAILED: MAGIOCSRANGE 1.9 Ga");
//	}

//	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
//		warnx("FAILED: MAGIOCEXSTRAP 0");
//	}

	return ret;
}

int HMC5883::set_excitement(bool enable)
{
	int ret = PX4_OK;

	/* arm the excitement strap */

	if (read_reg(ADDR_CONF_A, _conf_reg) != PX4_OK) {
		perf_count(_comms_errors);
	}

	_conf_reg &= ~0x03; // reset previous excitement mode

	if (((int)enable) < 0) {
		_conf_reg |= 0x01;

	} else if (enable > 0) {
		_conf_reg |= 0x02;

	}

	// ::printf("set_excitement enable=%d regA=0x%x\n", (int)enable, (unsigned)_conf_reg);

	ret = write_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	uint8_t conf_reg_ret = 0;
	read_reg(ADDR_CONF_A, conf_reg_ret);

	//print_info();

	return !(_conf_reg == conf_reg_ret);
}

/*
  enable/disable temperature compensation on the HMC5983

  Unfortunately we don't yet know of a way to auto-detect the
  difference between the HMC5883 and HMC5983. Both of them do
  temperature sensing, but only the 5983 does temperature
  compensation. We have noy yet found a behaviour that can be reliably
  distinguished by reading registers to know which type a particular
  sensor is

  update: Current best guess is that many sensors marked HMC5883L on
  the package are actually 5983 but without temperature compensation
  tables. Reading the temperature works, but the mag field is not
  automatically adjusted for temperature. We suspect that there may be
  some early 5883L parts that don't have the temperature sensor at
  all, although we haven't found one yet. The code that reads the
  temperature looks for 10 failed transfers in a row and disables the
  temperature sensor if that happens. It is hoped that this copes with
  the genuine 5883L parts.
 */
int HMC5883::set_temperature_compensation(bool enable)
{
	/* get current config */

	if (read_reg(ADDR_CONF_A, _conf_reg) != PX4_OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	if (enable) {
		_conf_reg |= HMC5983_TEMP_SENSOR_ENABLE;

	} else {
		_conf_reg &= ~HMC5983_TEMP_SENSOR_ENABLE;
	}

	if (write_reg(ADDR_CONF_A, _conf_reg) != PX4_OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	uint8_t conf_reg_ret = 0;

	if (read_reg(ADDR_CONF_A, conf_reg_ret) != PX4_OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	return conf_reg_ret == _conf_reg;
}

int
HMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
HMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
HMC5883::meas_to_float(uint8_t in[2]) const
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
HMC5883::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	print_message(_last_report);

	PX4_INFO("SC: (%.3f, %.3f, %.3f) OF: (%.3f, %.3f, %.3f) ", (double)_scale.x_scale, (double)_scale.y_scale,
		 (double)_scale.z_scale, (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
}

/**
 * Local functions in support of the shell command.
 */
namespace hmc5883
{

/*
  list of supported bus configurations
 */
struct hmc5883_bus_option {
	enum HMC5883_BUS busid;
	const char *devpath;
	HMC5883_constructor interface_constructor;
	uint8_t busnum;
	HMC5883	*dev;
} bus_options[] = {
	{ HMC5883_BUS_I2C_EXTERNAL, "/dev/hmc5883_ext", &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
	{ HMC5883_BUS_I2C_EXTERNAL, "/dev/hmc5883_ext1", &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
	{ HMC5883_BUS_I2C_EXTERNAL, "/dev/hmc5883_ext2", &HMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ HMC5883_BUS_I2C_INTERNAL, "/dev/hmc5883_int", &HMC5883_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_HMC
	{ HMC5883_BUS_SPI, "/dev/hmc5883_spi", &HMC5883_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum HMC5883_BUS busid, enum Rotation rotation);
int		stop();
bool	start_bus(struct hmc5883_bus_option &bus, enum Rotation rotation);
struct hmc5883_bus_option &find_bus(enum HMC5883_BUS busid);
void	test(enum HMC5883_BUS busid);
void	reset(enum HMC5883_BUS busid);
int	info(enum HMC5883_BUS busid);
int	calibrate(enum HMC5883_BUS busid);
int	temp_enable(HMC5883_BUS busid, bool enable);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct hmc5883_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u (type: %u)", (unsigned)bus.busnum, (unsigned)bus.busid);
		return false;
	}

	bus.dev = new HMC5883(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = px4_open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
	}

	px4_close(fd);

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum HMC5883_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == HMC5883_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != HMC5883_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		exit(1);
	}
}

int
stop()
{
	bool stopped = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_options[i].dev != nullptr) {
			bus_options[i].dev->stop();
			delete bus_options[i].dev;
			bus_options[i].dev = nullptr;
			stopped = true;
		}
	}

	return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct hmc5883_bus_option &find_bus(enum HMC5883_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == HMC5883_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum HMC5883_BUS busid)
{
	struct hmc5883_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'hmc5883 start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		print_message(report);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(enum HMC5883_BUS busid)
{
	struct hmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = px4_open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
int
info(enum HMC5883_BUS busid)
{
	struct hmc5883_bus_option &bus = find_bus(busid);

	warnx("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'info'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
hmc5883_main(int argc, char *argv[])
{
	int ch;
	enum HMC5883_BUS busid = HMC5883_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool calibrate = false;
	bool temp_compensation = false;

	if (argc < 2) {
		hmc5883::usage();
		exit(0);
	}

	while ((ch = getopt(argc, argv, "XISR:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

		case 'I':
			busid = HMC5883_BUS_I2C_INTERNAL;
			break;
#endif

		case 'X':
			busid = HMC5883_BUS_I2C_EXTERNAL;
			break;

		case 'S':
			busid = HMC5883_BUS_SPI;
			break;

		default:
			hmc5883::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		hmc5883::start(busid, rotation);

		exit(0);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		return hmc5883::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		hmc5883::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		hmc5883::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		hmc5883::info(busid);
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate', 'tempoff', 'tempon' or 'info'");
}
