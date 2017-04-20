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

#include "spi.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "config.h"
#include "log.h"

#define BITS_PER_WORD 8


SPI::SPI(const char *device)
{
	_device = strdup(device);
}

SPI::~SPI()
{
	free(_device);
	if (_fd != -1) {
		close(_fd);
	}
}

int SPI::init(uint8_t mode, uint32_t freq)
{
	int ret = -1;
	uint8_t bits_per_word = BITS_PER_WORD;
	uint8_t lsb_first = 0;

	_fd = open(_device, O_RDWR);
	if (_fd == -1) {
		ERROR("Unable to open device file descriptor: %s", _device);
		return -1;
	}

	ret = ioctl(_fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		ERROR("Unable to set SPI mode");
		goto error;
	}

	ret = ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
	if (ret == -1) {
		ERROR("Unable to set SPI bits per word");
		goto error;
	}

	ret = ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);
	if (ret == -1) {
		ERROR("Unable to set SPI max frequency");
		goto error;
	}

	ret = ioctl(_fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
	if (ret == -1) {
		ERROR("Unable to set SPI bit order");
		goto error;
	}

	_freq = freq;

	return 0;

error:
	close(_fd);
	_fd = -1;
	return ret;
}

int SPI::transfer(const uint8_t *send_buffer, uint16_t send_len, uint8_t *recv_buffer, uint16_t recv_len)
{
	struct spi_ioc_transfer t[2] = { };
	uint8_t t_len = 0;

	if (send_buffer && send_len) {
		t[t_len].tx_buf = (uint64_t) send_buffer;
		t[t_len].len = send_len;
		t[t_len].rx_buf = 0;
		t[t_len].speed_hz = _freq;
		t[t_len].delay_usecs = 0;
		t[t_len].bits_per_word = BITS_PER_WORD;
		t[t_len].cs_change = 0;
		t[t_len].tx_nbits = 0;
		t[t_len].rx_nbits = 0;
		t_len++;
	}

	if (recv_buffer && recv_len) {
		t[t_len].rx_buf = (uint64_t) recv_buffer;
		t[t_len].len = recv_len;
		t[t_len].tx_buf = 0;
		t[t_len].speed_hz = _freq;
		t[t_len].delay_usecs = 0;
		t[t_len].bits_per_word = BITS_PER_WORD;
		t[t_len].cs_change = 0;
		t[t_len].tx_nbits = 0;
		t[t_len].rx_nbits = 0;
		t_len++;
	}

	if (!t_len) {
		return 0;
	}

	return ioctl(_fd, SPI_IOC_MESSAGE(t_len), &t);
}
