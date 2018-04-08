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
 * @file ms5611.cpp
 * Driver for the MS5611 and MS6507 barometric pressure sensor connected via I2C or SPI.
 */

#include <string.h>

#include <px4_config.h>
#include <px4_workqueue.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <platforms/px4_getopt.h>

#include "ms5611.h"

enum MS56XX_DEVICE_TYPES {
	MS56XX_DEVICE = 0,
	MS5611_DEVICE = 5611,
	MS5607_DEVICE = 5607,
};

enum MS5611_BUS {
	MS5611_BUS_ALL = 0,
	MS5611_BUS_I2C_INTERNAL,
	MS5611_BUS_I2C_EXTERNAL,
	MS5611_BUS_SPI_INTERNAL,
	MS5611_BUS_SPI_EXTERNAL
};

static constexpr int64_t pow2(uint8_t x) { return 2 << (x - 1); }
static constexpr int64_t sq(int64_t x) { return x * x; }

enum class CONVERT_D1 {
	OSR256 = 0x40,
	OSR512 = 0x42,
	OSR1024 = 0x44,
	OSR2048 = 0x46,
	OSR4096 = 0x48,
};

enum class CONVERT_D2 {
	OSR256 = 0x50,
	OSR512 = 0x52,
	OSR1024 = 0x54,
	OSR2048 = 0x56,
	OSR4096 = 0x58,
};

// MS5611 (and MS5607) conversion time maximum delays used for scheduling
enum class DELAY_MAX_US {
	OSR256 = 600,
	OSR512 = 1170,
	OSR1024 = 2280,
	OSR2048 = 4540,
	OSR4096 = 9040,
};

class MS5611 : public device::CDev
{
public:
	MS5611(device::Device *interface, ms5611::prom_u &prom_buf, const char *path, enum MS56XX_DEVICE_TYPES device_type,
	       bool temp_comp);
	~MS5611();

	virtual int		init();

	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			stop() { work_cancel(HPWORK, &_work); }

protected:
	Device			*_interface;

	ms5611::prom_s		_prom;

	struct work_s		_work {};

	enum MS56XX_DEVICE_TYPES _device_type;

	enum {
		TEMP_CONVERT,
		TEMP_READ,
		PRESSURE_CONVERT,
		PRESSURE_READ,
	} _state{TEMP_CONVERT};

	uint64_t			_last_temperature_convert{0};
	uint64_t			_last_temperature_read{0};
	uint64_t			_last_pressure_convert{0};

	static constexpr unsigned PRESSURE_CONVERSION_CMD = (unsigned)CONVERT_D1::OSR1024;
	static constexpr uint64_t PRESSURE_CONVERSION_TIME = (uint64_t)DELAY_MAX_US::OSR1024;

	static constexpr unsigned TEMPERATURE_CONVERSION_CMD = (unsigned)CONVERT_D2::OSR1024;
	static constexpr uint64_t TEMPERATURE_CONVERSION_TIME = (uint64_t)DELAY_MAX_US::OSR1024;

	static constexpr uint64_t CYCLE_SLEEP = 1000000 / 100;

	bool			_temp_compensation{true};

	// intermediate temperature values per MS5611/MS5607 datasheet

	int64_t			_Tref{0};		// reference temperature
	int64_t			_dT{0};			// Difference between actual and reference temperature

	int64_t			_OFF_T1{0};
	int64_t			_OFF{0};		// Offset at actual temperature

	int64_t			_SENS_T1{0};
	int64_t			_SENS{0};		// Sensitivity at actual temperature

	/* altitude conversion calibration */
	unsigned		_msl_pressure{101325};	/* in Pa */

	orb_advert_t		_baro_topic{nullptr};

	int			_orb_class_instance{-1};
	int			_class_instance{-1};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	sensor_baro_s		_sensor_baro{};

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void			cycle();

	/**
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
	bool			update(bool cycle = true);

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	bool			read_adc(uint32_t &adc);

	bool			read_temperature();
	bool			read_pressure();
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ms5611_main(int argc, char *argv[]);

MS5611::MS5611(device::Device *interface, ms5611::prom_u &prom_buf, const char *path,
	       enum MS56XX_DEVICE_TYPES device_type, bool temp_comp) :
	CDev("MS5611", path),
	_interface(interface),
	_prom(prom_buf.s),
	_device_type(device_type),
	_temp_compensation(temp_comp),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5611_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "ms5611_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "ms5611_com_err"))
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();

	/* set later on init */
	_device_id.devid_s.devtype = 0;
}

MS5611::~MS5611()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(get_devname(), _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
MS5611::init()
{
	int ret;
	bool autodetect = false;

	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	if (_device_type == MS56XX_DEVICE) {
		autodetect = true;
		/* try first with MS5611 and fallback to MS5607 */
		_device_type = MS5611_DEVICE;
	}

	// the same for both MS5611 & MS5607
	_Tref = _prom.C5 * pow2(8);

	while (true) {

		switch (_device_type) {
		default:

		// FALLTHROUGH
		case MS5611_DEVICE:
			_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_MS5611;
			_OFF_T1 = _prom.C2 * pow2(16);
			_SENS_T1 = _prom.C1 * pow2(15);
			break;

		case MS5607_DEVICE:
			_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_MS5607;
			_OFF_T1 = _prom.C2 * pow2(17);
			_SENS_T1 = _prom.C1 * pow2(16);
			break;
		}

		/* do temperature first */
		_state = TEMP_CONVERT;

		if ((_state != TEMP_CONVERT) || !update(false)) {
			PX4_ERR("init temperature convert failed");
			continue;
		}

		usleep(TEMPERATURE_CONVERSION_TIME);

		if ((_state != TEMP_READ) || !update(false)) {
			PX4_ERR("init temperature read failed");
			continue;
		}

		// now pressure
		_state = PRESSURE_CONVERT;

		if ((_state != PRESSURE_CONVERT) || !update(false)) {
			PX4_ERR("init pressure convert failed");
			continue;
		}

		usleep(PRESSURE_CONVERSION_TIME);

		if ((_state != PRESSURE_READ) || !update(false)) {
			PX4_ERR("init pressure read failed");
			continue;
		}

		if (autodetect) {
			if (_device_type == MS5611_DEVICE) {
				if (_sensor_baro.altitude > 5300.f) {
					/* This is likely not this device, try again */
					_device_type = MS5607_DEVICE;

					continue;
				}

			} else if (_device_type == MS5607_DEVICE) {
				if (_sensor_baro.altitude > 5300.f) {
					/* Both devices returned very high altitude;
					 * have fun on Everest using MS5611 */
					_device_type = MS5611_DEVICE;
				}
			}
		}

		/* ensure correct devid */
		_sensor_baro.device_id = _device_id.devid;

		ret = OK;

		_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &_sensor_baro, &_orb_class_instance,
						  _interface->external() ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

		break;
	}

	// start cycle
	if (ret == PX4_OK) {
		PX4_INFO("started successfully");
		cycle();

	} else {
		PX4_ERR("failed to start");
	}

out:
	return ret;
}

int
MS5611::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
}

void
MS5611::cycle_trampoline(void *arg)
{
	MS5611 *dev = reinterpret_cast<MS5611 *>(arg);

	dev->cycle();
}

void
MS5611::cycle()
{
	update(true);
}

bool
MS5611::update(bool cycle)
{
	uint32_t delay = 0;

	bool result = false;

	switch (_state) {
	case TEMP_CONVERT: {
			unsigned int cmd = TEMPERATURE_CONVERSION_CMD;
			int ret = _interface->ioctl(IOCTL_MEASURE, cmd);

			if (ret == PX4_OK) {
				_last_temperature_convert = hrt_absolute_time();
				_state = TEMP_READ;
				delay = CYCLE_SLEEP;
			}

			result = (ret == PX4_OK);
		}
		break;

	case TEMP_READ: {
			result = read_temperature();
			_state = PRESSURE_CONVERT;
		}
		break;

	case PRESSURE_CONVERT: {
			unsigned cmd = PRESSURE_CONVERSION_CMD;
			int ret = _interface->ioctl(IOCTL_MEASURE, cmd);

			if (ret == PX4_OK) {
				_last_pressure_convert = hrt_absolute_time();
				_state = PRESSURE_READ;
				delay = CYCLE_SLEEP;
			}

			result = (ret == PX4_OK);
		}
		break;

	case PRESSURE_READ: {
			result = read_pressure();

			if (hrt_elapsed_time(&_last_temperature_read) > 1000000) {
				// update temperature ~ once per second
				_state = TEMP_CONVERT;

			} else {
				_state = PRESSURE_CONVERT;
			}

			if (!result) {
				PX4_ERR("read pressure failed");
			}
		}
		break;
	}

	if (!result) {
		PX4_ERR("resetting");

		// issue a reset command to the sensor
		unsigned dummy;
		_interface->ioctl(IOCTL_RESET, dummy);

		// restart state machine with delay
		_state = TEMP_CONVERT;
		delay = CYCLE_SLEEP;
	}

	if (cycle) {
		// schedule next cycle
		return (work_queue(HPWORK, &_work, (worker_t)&MS5611::cycle_trampoline, this, USEC2TICK(delay)) == PX4_OK);

	} else  {
		return result;
	}
}

bool
MS5611::read_adc(uint32_t &adc)
{
	uint32_t val = 0;

	int ret = _interface->read(0, (void *)&val, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		return false;

	} else if (val == 0) {
		// If the conversion is not executed before the ADC read command, or the ADC read command is repeated, it will give 0 as the output result.
		// If the ADC read command is sent during conversion the result will be 0, the conversion will not stop and the final
		// result will be wrong.

		perf_count(_comms_errors);
		return false;
	}

	adc = val;
	return true;
}

bool
MS5611::read_temperature()
{
	if (hrt_elapsed_time(&_last_temperature_convert) < TEMPERATURE_CONVERSION_TIME) {
		return false;
	}

	// Read digital temperature data
	uint32_t adc = 0;

	if (read_adc(adc)) {
		const uint32_t D2 = adc;

		// Difference between actual and reference temperature
		_dT = (int64_t)D2 - _Tref;

		// Actual temperature (-40 - 85°C with 0.01°C resolution)
		int32_t TEMP = 2000 + ((int64_t)(_dT * _prom.C6) / pow2(23));

		if (_temp_compensation) {
			if (_device_type == MS5611_DEVICE) {
				// MS5611 Calculation

				// Offset at actual temperature
				_OFF = _OFF_T1 + ((int64_t)(_prom.C4 * _dT) / pow2(7));

				// Sensitivity at actual temperature
				_SENS = _SENS_T1 + ((int64_t)(_prom.C3 * _dT) / pow2(8));

				// SECOND ORDER TEMPERATURE COMPENSATION
				if (TEMP < 2000) {
					// Low temperature
					int64_t T2 = sq(_dT) / pow2(31);
					int64_t OFF2 = 5 * sq(TEMP - 2000) / pow2(1);
					int64_t SENS2 = 5 * sq(TEMP - 2000) / pow2(2);

					// Very low temperature
					if (TEMP < -1500) {
						OFF2 += 7 * sq(TEMP + 1500);
						SENS2 += 11 * sq(TEMP + 1500) / pow2(1);
					}

					TEMP -= T2;
					_OFF  -= OFF2;
					_SENS -= SENS2;
				}

			} else if (_device_type == MS5607_DEVICE) {
				// MS5607 Calculation

				// Offset at actual temperature
				_OFF = _OFF_T1 + ((int64_t)(_prom.C4 * _dT) / pow2(6));

				//Sensitivity at actual temperature
				_SENS = _SENS_T1 + ((int64_t)(_prom.C3 * _dT) / pow2(7));

				// SECOND ORDER TEMPERATURE COMPENSATION
				if (TEMP < 2000) {
					// Low temperature
					int64_t T2 = sq(_dT) / pow2(31);
					int64_t OFF2 = 61 * sq(TEMP - 2000) / pow2(4);
					int64_t SENS2 = 2 * sq(TEMP - 2000);

					// Very low temperature
					if (TEMP < -1500) {
						OFF2 += 15 * sq(TEMP + 1500);
						SENS2 += 8 * sq(TEMP + 1500);
					}

					TEMP -= T2;
					_OFF  -= OFF2;
					_SENS -= SENS2;
				}
			}

		} else {
			// skip manufacturer temperature compensation
			_dT = 0;
			_OFF = _OFF_T1;
			_SENS = _SENS_T1;
		}

		_sensor_baro.temperature = TEMP / 100.0f;
		_last_temperature_read = hrt_absolute_time();

		return true;
	}

	return false;
}

bool
MS5611::read_pressure()
{
	if (hrt_elapsed_time(&_last_pressure_convert) < PRESSURE_CONVERSION_TIME) {
		return false;
	}

	uint32_t adc = 0;

	if (read_adc(adc)) {
		_sensor_baro.timestamp = hrt_absolute_time();

		// Pressure measurement phase
		const uint32_t D1 = adc;

		// Temperature compensated pressure (10…1200mbar with 0.01mbar resolution)
		const int64_t P = ((D1 * _SENS) / pow2(21) - _OFF) / pow2(15);

		_sensor_baro.pressure = P / 100.0f; // pressure in millibar

		/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

		/*
		 * PERFORMANCE HINT:
		 *
		 * The single precision calculation is 50 microseconds faster than the double
		 * precision variant. It is however not obvious if double precision is required.
		 * Pending more inspection and tests, we'll leave the double precision variant active.
		 *
		 * Measurements:
		 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
		 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
		 */

		/* tropospheric properties (0-11km) for standard atmosphere */
		const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
		const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
		const double g  = 9.80665;	/* gravity constant in m/s/s */
		const double R  = 287.05;	/* ideal gas constant in J/kg/K */

		/* current pressure at MSL in kPa */
		double p1 = _msl_pressure / 1000.0;

		/* measured pressure in kPa */
		double p = (double)_sensor_baro.pressure * 0.1; // mbar to kPa

		/*
		 * Solve:
		 *
		 *     /        -(aR / g)     \
		 *    | (p / p1)          . T1 | - T1
		 *     \                      /
		 * h = -------------------------------  + h1
		 *                   a
		 */
		_sensor_baro.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

		_sensor_baro.error_count = perf_event_count(_comms_errors);

		if (_baro_topic) {
			orb_publish(ORB_ID(sensor_baro), _baro_topic, &_sensor_baro);
		}

		return true;
	}

	return false;
}

void
MS5611::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	PX4_INFO("device: %s", _device_type == MS5611_DEVICE ? "ms5611" : "ms5607");

	if (_temp_compensation) {
		PX4_INFO("Manufacturer second order temperature compensation enabled");
	}

	PX4_INFO("factory_setup  %u", _prom.factory_setup);
	PX4_INFO("serial_and_crc %u", _prom.serial_and_crc);

	PX4_INFO("");

	PX4_INFO("C1 %u", _prom.C1);
	PX4_INFO("C2 %u", _prom.C2);
	PX4_INFO("C3 %u", _prom.C3);
	PX4_INFO("C4 %u", _prom.C4);
	PX4_INFO("C5 %u", _prom.C5);
	PX4_INFO("C6 %u", _prom.C6);

	PX4_INFO("");

	PX4_INFO("Tref:    %lld", _Tref);
	PX4_INFO("dT:      %lld", _dT);

	PX4_INFO("");
	PX4_INFO("OFF_T1:  %lld", _OFF_T1);
	PX4_INFO("OFF:     %lld", _OFF);
	PX4_INFO("SENS_T1: %lld", _SENS_T1);
	PX4_INFO("SENS:    %lld", _SENS);

	print_message(_sensor_baro);
}

/**
 * Local functions in support of the shell command.
 */
namespace ms5611
{

/*
  list of supported bus configurations
 */
struct ms5611_bus_option {
	enum MS5611_BUS busid;
	const char *devpath;
	MS5611_constructor interface_constructor;
	uint8_t busnum;
	MS5611 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ MS5611_BUS_SPI_EXTERNAL, "/dev/ms5611_spi_ext", &MS5611_spi_interface, PX4_SPI_BUS_EXT, nullptr },
#endif
#ifdef PX4_SPIDEV_BARO
	{ MS5611_BUS_SPI_INTERNAL, "/dev/ms5611_spi_int", &MS5611_spi_interface, PX4_SPI_BUS_BARO, nullptr },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
	{ MS5611_BUS_I2C_INTERNAL, "/dev/ms5611_int", &MS5611_i2c_interface, PX4_I2C_BUS_ONBOARD, nullptr },
#endif
#ifdef PX4_I2C_BUS_EXPANSION
	{ MS5611_BUS_I2C_EXTERNAL, "/dev/ms5611_ext", &MS5611_i2c_interface, PX4_I2C_BUS_EXPANSION, nullptr },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type, bool temp_comp);

struct ms5611_bus_option *find_bus(enum MS5611_BUS busid);

int	start(enum MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type, bool temp_comp);
int	stop();
int	reset(enum MS5611_BUS busid);
void	info();
void	usage();

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

/**
 * Start the driver.
 */
bool
start_bus(struct ms5611_bus_option &bus, enum MS56XX_DEVICE_TYPES device_type, bool temp_comp)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		return false;
	}

	prom_u prom_buf;
	device::Device *interface = bus.interface_constructor(prom_buf, bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MS5611(interface, prom_buf, bus.devpath, device_type, temp_comp);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	return true;
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
int
start(enum MS5611_BUS busid, enum MS56XX_DEVICE_TYPES device_type, bool temp_comp)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MS5611_BUS_ALL && bus_options[i].dev != nullptr) {
			// this device is already started
			continue;
		}

		if (busid != MS5611_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started = started | start_bus(bus_options[i], device_type, temp_comp);
	}

	return (started ? PX4_OK : PX4_ERROR);
}

int
stop()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct ms5611_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			bus.dev->stop();
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * find a bus structure for a busid
 */
struct ms5611_bus_option *find_bus(enum MS5611_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MS5611_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != nullptr) {
			return &bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);

	return nullptr;
}

/**
 * Reset the driver.
 */
int
reset(enum MS5611_BUS busid)
{
	int ret = PX4_OK;

	struct ms5611_bus_option *bus = find_bus(busid);

	if (bus == nullptr) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	int fd = px4_open(bus->devpath, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open failed");
		ret = PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		ret = PX4_ERROR;
	}

	px4_close(fd);

	return ret;
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct ms5611_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_INFO("%s", bus.devpath);
			bus.dev->print_info();
		}
	}
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'reset'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external I2C bus)");
	PX4_INFO("    -I    (internal I2C bus)");
	PX4_INFO("    -S    (external SPI bus)");
	PX4_INFO("    -s    (internal SPI bus)");
	PX4_INFO("    -T    5611|5607 (default 5611)");
	PX4_INFO("    -T    0 (autodetect version)");
	PX4_INFO("    -C    disable manufacturer temperature compensation");
}

} // namespace

int
ms5611_main(int argc, char *argv[])
{
	enum MS5611_BUS busid = MS5611_BUS_ALL;
	enum MS56XX_DEVICE_TYPES device_type = MS5611_DEVICE;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	bool temp_comp = true;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "T:XISsC", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MS5611_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MS5611_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MS5611_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = MS5611_BUS_SPI_INTERNAL;
			break;

		case 'C':
			temp_comp = false;
			break;

		case 'T': {
				int val = atoi(myoptarg);

				if (val == 5611) {
					device_type = MS5611_DEVICE;
					break;

				} else if (val == 5607) {
					device_type = MS5607_DEVICE;
					break;

				} else if (val == 0) {
					device_type = MS56XX_DEVICE;
					break;
				}
			}

		default:
			ms5611::usage();
			return 0;
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		return ms5611::start(busid, device_type, temp_comp);

	} else if (!strcmp(verb, "stop")) {
		return ms5611::reset(busid);

	} else if (!strcmp(verb, "reset")) {
		return ms5611::reset(busid);

	} else if (!strcmp(verb, "info")) {
		ms5611::info();
		return PX4_OK;
	}

	PX4_WARN("unrecognised command, try 'start', 'reset' or 'info'");

	return 1;
}
