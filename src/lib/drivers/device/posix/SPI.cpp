/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */

#include "SPI.hpp"

#include <px4_config.h>

#if defined(__PX4_QURT)
#include "dev_fs_lib_spi.h"
#else
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#endif

namespace device
{

SPI::SPI(const char *name, const char *devname, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	CDev(name, devname),
	locking_mode(LOCK_PREEMPTION),
	_device(device),
	_mode(mode),
	_frequency(frequency)
{
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;

	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SPI::~SPI()
{
	// XXX no way to let go of the bus...
}

int
SPI::init()
{
	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}

	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		goto out;
	}

out:
	return ret;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = locking_mode;

	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: {
			ATOMIC_ENTER;
			result = _transfer(send, recv, len);
			ATOMIC_LEAVE;
		}
		break;

	case LOCK_THREADS:
		result = _transfer(send, recv, len);
		break;

	case LOCK_NONE:
		result = _transfer(send, recv, len);
		break;
	}

	return result;
}

int
SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{


	return OK;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = locking_mode;

	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: {
			ATOMIC_ENTER;
			result = _transferhword(send, recv, len);
			ATOMIC_LEAVE;
		}
		break;

	case LOCK_THREADS:
		result = _transferhword(send, recv, len);
		break;

	case LOCK_NONE:
		result = _transferhword(send, recv, len);
		break;
	}

	return result;
}

int
SPI::_transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	return OK;
}

} // namespace device
