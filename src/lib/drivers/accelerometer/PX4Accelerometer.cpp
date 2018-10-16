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


#include "PX4Accelerometer.hpp"

PX4Accelerometer::PX4Accelerometer(const char *path, device::Device *interface, uint8_t dev_type,
				   enum Rotation rotation, float scale) :
	CDev(path)
{
	CDev::init();

	_class_device_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	_rotation = rotation;

	_report.device_id = _interface->get_device_id();
	_report.scaling = _scale;
}

PX4Accelerometer::~PX4Accelerometer()
{
	if (_topic != nullptr) {
		orb_unadvertise(_topic);
	}
}

int PX4Accelerometer::init()
{
	accel_report report{};
	report.device_id = _device_id.devid;

	if (_topic == nullptr) {
		_topic = orb_advertise_multi(ORB_ID(sensor_accel), &report, &_orb_class_instance, ORB_PRIO_HIGH - 1);

		if (_topic == nullptr) {
			PX4_ERR("Advertise failed.");
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

int PX4Accelerometer::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case ACCELIOCSSCALE: {
			// Copy scale in.
			accel_calibration_s cal{};
			memcpy(&cal, (accel_calibration_s *) arg, sizeof(cal));

			_calibration_offset = matrix::Vector3f{cal.x_offset, cal.y_offset, cal.z_offset};
			_calibration_scale = matrix::Vector3f{cal.x_scale, cal.y_scale, cal.z_scale};
		}

		return PX4_OK;

	case DEVIOCGDEVICEID:
		//return _interface->get_device_id();
		return 0;

	default:
		// Give it to the superclass.
		return CDev::ioctl(filp, cmd, arg);
	}
}

void PX4Accelerometer::configure_filter(float sample_freq, float cutoff_freq)
{
	_filter.set_cutoff_frequency(sample_freq, cutoff_freq);
}

void PX4Accelerometer::update(int16_t x, int16_t y, int16_t z)
{
	_report.timestamp = hrt_absolute_time();

	// Apply the rotation.
	rotate_3f(_rotation, x, y, z);

	const matrix::Vector3f val{x, y, z};

	// Apply FS range scale and the calibrating offset/scale
	const matrix::Vector3f val_calibrated = ((val * _report.scale) - _calibration_offset) * _calibration_scale;

	// Filtered values
	const matrix::Vector3f val_filtered = _filter.apply(val_calibrated);

	// Integrated values
	matrix::Vector3f integrated_value;
	bool accel_notify = _integrator.put(report.timestamp, val_calibrated, integrated_value, report.integral_dt);

	if (accel_notify) {
		_report.x = val_filtered(0);
		_report.y = val_filtered(1);
		_report.z = val_filtered(2);

		_report.x_integral = integrated_value(0);
		_report.y_integral = integrated_value(1);
		_report.z_integral = integrated_value(2);

		// Raw values (ADC units 0 - 65535)
		_report.x_raw = x;
		_report.y_raw = y;
		_report.z_raw = z;

		poll_notify(POLLIN);
		orb_publish(ORB_ID(sensor_accel), _topic, &_report);
	}
}
