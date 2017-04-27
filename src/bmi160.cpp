/****************************************************************************
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
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

#include "bmi160.h"

#include <fcntl.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include "log.h"
#include "spi.h"
#include "util.h"

/* Registers and bits definitions. The indented ones are the bits for the upper
 * register. */
#define BMI160_REG_CHIPID 0x00
#define     BMI160_CHIPID 0xD1
#define BMI160_REG_ERR_REG 0x02
#define BMI160_REG_FIFO_LENGTH 0x22
#define BMI160_REG_FIFO_DATA 0x24
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
/* For convenience, use log2(range) - 1 instead of bits defined in
 * the datasheet. See _configure_accel(). */
#define     BMI160_ACC_RANGE_16G 3
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define     BMI160_GYR_RANGE_2000DPS 0x00
#define BMI160_REG_FIFO_CONFIG_0 0x46
#define BMI160_REG_FIFO_CONFIG_1 0x47
#define     BMI160_FIFO_ACC_EN 0x40
#define     BMI160_FIFO_GYR_EN 0x80
#define BMI160_REG_INT_EN_1 0x51
#define     BMI160_INT_FWM_EN 0x40
#define BMI160_REG_INT_OUT_CTRL 0x53
#define     BMI160_INT1_LVL 0x02
#define     BMI160_INT1_OUTPUT_EN 0x08
#define BMI160_REG_INT_MAP_1 0x56
#define     BMI160_INT_MAP_INT1_FWM 0x40
#define BMI160_REG_CMD 0x7E
#define     BMI160_CMD_ACCEL_NORMAL_POWER_MODE 0x11
#define     BMI160_CMD_GYRO_NORMAL_POWER_MODE 0x15
#define     BMI160_CMD_FIFO_FLUSH 0xB0
#define     BMI160_CMD_SOFTRESET 0xB6

#define     BMI160_OSR_NORMAL 0x20
#define     BMI160_ODR_1600HZ 0x0C

/* Datasheet says that the device powers up in less than 10ms, so waiting for
 * 10 ms before initialization is enough. */
#define BMI160_POWERUP_DELAY_MSEC 10
/* TODO: Investigate this. The delay below is way too high and with that
 * there's still at least 1% of failures on initialization. Lower values
 * increase that percentage. */
#define BMI160_SOFTRESET_DELAY_MSEC 100
/* Define a little bit more than the maximum value in the datasheet's timing
 * table. The datasheet recommends adding 300 us to the time for startup
 * occasions. */
#define BMI160_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC 4
#define BMI160_GYRO_NORMAL_POWER_MODE_DELAY_MSEC 81

#define BMI160_OSR BMI160_OSR_NORMAL
#define BMI160_ODR BMI160_ODR_1600HZ
#define BMI160_ACC_RANGE BMI160_ACC_RANGE_16G
#define BMI160_GYR_RANGE BMI160_GYR_RANGE_2000DPS

/* By looking at the datasheet, the accel range i (as defined by the macros
 * BMI160_ACC_RANGE_*G) maps to the range bits by the function f defined:
 *     f(0) = 3; f(i) = f(i - 1) + i + 1
 * Which can be written as the closed formula:
 *     f(i) = (i * (i + 3)) / 2 + 3 */
#define BMI160_ACC_RANGE_BITS \
    (BMI160_ACC_RANGE * (BMI160_ACC_RANGE + 3) / 2 + 3)

/* The rate in Hz based on the ODR bits can be calculated with
 * 100 / (2 ^ (8 - odr) */
#define BMI160_ODR_TO_HZ(odr_) \
    (uint16_t)(odr_ > 8 ? 100 << (odr_ - 8) : 100 >> (8 - odr_))

/* This number of samples should provide only one read burst operation on the
 * FIFO most of the time (99.99%). */
#define BMI160_MAX_FIFO_SAMPLES 8

#define BMI160_READ_FLAG 0x80
#define BMI160_HARDWARE_INIT_MAX_TRIES 5

#define BMI160_PARAMETERS_FILE "bmi160.param"
#define BMI160_PARAMETERS_FILE_MAGIC_UINT64_T 0xbadc0ffee0000000
// 5 seconds
#define BMI160_SAMPLES_TO_CALIBRATE (1600 * 5)

struct __attribute__((__packed__)) RawData
{
	struct
	{
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} gyro;
	struct
	{
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} accel;
};

BMI160::BMI160(const char *spi_device, const char *parameters_folder)
{
	_spi = new SPI(spi_device);
	_parameters_folder = parameters_folder;
}

BMI160::~BMI160()
{
	delete _spi;
}

int BMI160::init()
{
	bool ret = false;

	// mode = 3, freq = 10MHz
	_spi->init(3, 10 * 1000 * 1000);

	for (unsigned i = 0; i < BMI160_HARDWARE_INIT_MAX_TRIES; i++) {
		uint8_t v = 0;

		ret = write_register(BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
		if (!ret) {
			continue;
		}
		usleep(BMI160_SOFTRESET_DELAY_MSEC * USEC_PER_MSEC);

		/* The datasheet recommends doing a read operation on the register 0x7F
		 * in order to guarantee the sensor works using the SPI protocol. This
		 * shouldn't have side effects for I2C. */
		ret = read_register(0x7F, &v, 1);
		if (!ret) {
			continue;
		}

		ret = read_register(BMI160_REG_CHIPID, &v, 1);
		if (!ret) {
			continue;
		}
		if (v != BMI160_CHIPID) {
			ret = false;
			continue;
		}

		ret = write_register(BMI160_REG_CMD,
				BMI160_CMD_ACCEL_NORMAL_POWER_MODE);
		if (!ret) {
			continue;
		}
		usleep(BMI160_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC * USEC_PER_MSEC);

		ret = write_register(BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL_POWER_MODE);
		if (!ret) {
			continue;
		}
		usleep(BMI160_GYRO_NORMAL_POWER_MODE_DELAY_MSEC * USEC_PER_MSEC);

		break;
	}

	return ret ? 0 : -1;
}

bool BMI160::_configure_accel()
{
	bool r;

	r = write_register(BMI160_REG_ACC_CONF, BMI160_OSR | BMI160_ODR);
	if (!r) {
		return false;
	}
	usleep(USEC_PER_MSEC);

	r = write_register(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_BITS);
	if (!r) {
		return false;
	}
	usleep(USEC_PER_MSEC);

	/* The sensitivity in LSb/g for an accel range i (as defined by the macros
	 * BMI160_ACC_RANGE_*G) can be calculated with:
	 *     2 ^ 16 / (2 * 2 ^ (i + 1)) = 2 ^(14 - i)
	 * That matches the typical values in the datasheet. */
	_accel_scale = GRAVITY_MSS / (1 << (14 - BMI160_ACC_RANGE));

	return true;
}

bool BMI160::_configure_gyro()
{
	bool r;

	r = write_register(BMI160_REG_GYR_CONF, BMI160_OSR | BMI160_ODR);
	if (!r) {
		return false;
	}
	usleep(USEC_PER_MSEC);

	r = write_register(BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE);
	if (!r) {
		return false;
	}
	usleep(USEC_PER_MSEC);

	/* The sensitivity in LSb/degrees/s a gyro range i can be calculated with:
	 *     2 ^ 16 / (2 * 2000 / 2 ^ i) = 2 ^ (14 + i) / 1000
	 * The scale is the inverse of that. */
	_gyro_scale = radians(1000.f / (1 << (14 + BMI160_GYR_RANGE)));

	return true;
}

bool BMI160::_configure_fifo()
{
	bool r;

	/* The unit for the FIFO watermark is 4 bytes. */
	r = write_register(BMI160_REG_FIFO_CONFIG_0, sizeof(struct RawData) / 4);
	if (!r) {
		ERROR("BMI160: Unable to configure FIFO watermark level\n");
		return false;
	}
	usleep(USEC_PER_MSEC);

	r = write_register(BMI160_REG_FIFO_CONFIG_1,
			BMI160_FIFO_ACC_EN | BMI160_FIFO_GYR_EN);
	if (!r) {
		ERROR("BMI160: Unable to enable FIFO\n");
		return false;
	}
	usleep(USEC_PER_MSEC);

	r = write_register(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
	if (!r) {
		ERROR("BMI160: Unable to flush FIFO\n");
		return false;
	}

	return true;
}

void BMI160::_read_fifo()
{
	struct RawData raw_data[BMI160_MAX_FIFO_SAMPLES];
	uint16_t num_bytes = 0;
	uint16_t excess;
	uint8_t num_samples = 0;
	bool r = true;

	r = read_register(BMI160_REG_FIFO_LENGTH, (uint8_t *) &num_bytes,
			sizeof(num_bytes));
	if (!r) {
		goto read_fifo_end;
	}

	if (!num_bytes) {
		goto read_fifo_end;
	}

read_fifo_read_data:
	if (num_bytes > sizeof(raw_data)) {
		excess = num_bytes - sizeof(raw_data);
		num_bytes = sizeof(raw_data);
	} else {
		excess = 0;
	}

	r = read_register(BMI160_REG_FIFO_DATA, (uint8_t *) raw_data, num_bytes);
	if (!r) {
		goto read_fifo_end;
	}

	/* Read again just once */
	if (excess && num_samples) {
		DEBUG("BMI160: dropping %u samples from fifo\n",
				(uint8_t )(excess / sizeof(struct RawData)));
		write_register(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
		excess = 0;
	}

	num_samples = num_bytes / sizeof(struct RawData);
	for (uint8_t i = 0; i < num_samples; i++) {
		Point3_<double> gyro;
		gyro.x = (double)(int16_t)raw_data[i].gyro.x;
		gyro.y = (double)(int16_t)raw_data[i].gyro.y;
		gyro.z = (double)(int16_t)raw_data[i].gyro.z;

		/* Apply roll 180 rotation, it specific to Aero board */
		gyro.y = gyro.y * -1.0;
		gyro.z = gyro.z * -1.0;

		gyro *= _gyro_scale;

		if (_calibration_samples_counter) {
			if (_calibration_samples_counter == BMI160_SAMPLES_TO_CALIBRATE) {
				_gyro_offsets = gyro;
			} else {
				_gyro_offsets += gyro;
				_gyro_offsets /= 2.0;
			}
			_calibration_samples_counter--;

			// last sample? save offsets
			if (!_calibration_samples_counter) {
				_calibration_save();
			}
		} else {
			gyro -= _gyro_offsets;

			/*
			 * Integrate it.
			 * f = 1/t => 1600Hz = 1/t => t = 0.000625sec
			 */
			gyro *= 0.000625;
			_gyro_integrated += gyro;

			clock_gettime(CLOCK_MONOTONIC, &_gyro_last_update);
		}
	}

	if (excess) {
		num_bytes = excess;
		goto read_fifo_read_data;
	}

read_fifo_end:
	if (!r) {
		DEBUG("BMI160: error on reading FIFO\n");
	}
}

int BMI160::_calibration_load()
{
	char path[strlen(_parameters_folder) + strlen(BMI160_PARAMETERS_FILE)];

	sprintf(path, "%s/%s", _parameters_folder, BMI160_PARAMETERS_FILE);
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		ERROR("BMI160 Unable to open %s", path);
		return -1;
	}

	uint64_t magic;
	int ret = read(fd, &magic, sizeof(magic));
	if (ret != sizeof(magic)) {
		ERROR("BMI160 Magic number not found on parameter file");
		ret = -1;
		goto end;
	}

	if (magic != BMI160_PARAMETERS_FILE_MAGIC_UINT64_T) {
		ERROR("BMI160 Wrong magic number found on parameter file");
		ret = -1;
		goto end;
	}

	double v[3];
	ret = read(fd, v, sizeof(v));
	if (ret != sizeof(v)) {
		ERROR("BMI160 Unable to read offsets from parameter file");
		ret = -1;
		goto end;
	}

	_gyro_offsets.x = v[0];
	_gyro_offsets.y = v[1];
	_gyro_offsets.z = v[2];
	DEBUG("BMI160 Gyroscope offsets loaded(%f %f %f)", v[0], v[1], v[2]);
	ret = 0;

end:
	close(fd);
	return ret;
}

int BMI160::_calibration_save()
{
	char path[strlen(_parameters_folder) + strlen(BMI160_PARAMETERS_FILE)];

	sprintf(path, "%s/%s", _parameters_folder, BMI160_PARAMETERS_FILE);
	int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC);
	if (fd < 0) {
		ERROR("BMI160 Unable to open %s", path);
		return -1;
	}

	uint64_t magic = BMI160_PARAMETERS_FILE_MAGIC_UINT64_T;
	int ret = write(fd, &magic, sizeof(magic));
	if (ret != sizeof(magic)) {
		ERROR("BMI160 Unable to write magic number on calibration file");
		ret = -1;
		goto end;
	}

	double v[3];
	v[0] = _gyro_offsets.x;
	v[1] = _gyro_offsets.y;
	v[2] = _gyro_offsets.z;
	ret = write(fd, v, sizeof(v));
	if (ret != sizeof(v)) {
		ERROR("BMI160 Unable to write offsets on calibration file");
		ret = -1;
		goto end;
	}

	DEBUG("BMI160 Gyroscope offsets saved");
	ret = 0;
end:
	close(fd);
	return ret;
}

int BMI160::start()
{
	bool r;
	struct itimerspec ts;

	r = _configure_accel();
	if (!r) {
		ERROR("BMI160: Unable to configure accelerometer");
		return -1;
	}

	r = _configure_gyro();
	if (!r) {
		ERROR("BMI160: Unable to configure gyroscope");
		return -1;
	}

	r = _configure_fifo();
	if (!r) {
		ERROR("BMI160: Unable to configure FIFO");
		return -1;
	}

	_fd = timerfd_create(CLOCK_MONOTONIC, O_NONBLOCK);
	if (_fd < 0) {
		ERROR("Unable to create timerfd");
		return -1;
	}

	_gyro_integrated.x = 0;
	_gyro_integrated.y = 0;
	_gyro_integrated.z = 0;

	if (!_calibration_samples_counter) {
		if (_calibration_load()) {
			_gyro_offsets.x = 0;
			_gyro_offsets.y = 0;
			_gyro_offsets.z = 0;
		}
	}

	/*
	 * Set a 1kHz timeout
	 * f=1/t => 1000=1/t => t=1/1000 => t=0.001sec
	 */
	ts.it_interval.tv_sec = 0;
	ts.it_interval.tv_nsec = (0.001 * NSEC_PER_SEC);
	ts.it_value.tv_sec = ts.it_interval.tv_sec;
	ts.it_value.tv_nsec = ts.it_interval.tv_nsec;
	timerfd_settime(_fd, 0, &ts, NULL);

	return 0;
}

void BMI160::stop()
{
	if (_fd > -1) {
		close(_fd);
		_fd = -1;
	}
}

bool BMI160::write_register(uint8_t reg, uint8_t val)
{
	uint8_t buffer[2];
	buffer[0] = reg;
	buffer[1] = val;
	return _spi->transfer(buffer, 2, NULL, 0) == 2;
}

bool BMI160::read_register(uint8_t reg, uint8_t *recv_buffer, uint16_t recv_len)
{
	reg |= BMI160_READ_FLAG;
	return _spi->transfer(&reg, 1, recv_buffer, recv_len) == (recv_len + 1);
}

void BMI160::gyro_integrated_get(Point3_<double> *gyro, struct timespec *t)
{
	*gyro = _gyro_integrated;
	*t = _gyro_last_update;
}

void BMI160::handle_read()
{
	uint64_t val = 0;
	int ret = read(_fd, &val, sizeof(val));

	if (ret < 1 || val == 0)
		return;

	_read_fifo();
}

bool BMI160::handle_canwrite()
{
	return false;
}

void BMI160::calibrate()
{
	_calibration_samples_counter = BMI160_SAMPLES_TO_CALIBRATE;
}
