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

#pragma once

#include <opencv2/opencv.hpp>

#include "pollable.h"
#include "spi.h"

using namespace cv;

class BMI160 : public Pollable {
public:
	BMI160(const char *spi_device);
	virtual ~BMI160();
	int init();
	int start();
	void stop();
	/* Should be called before start() */
	void calibrate();

	void gyro_integrated_get(Point3_<double> *gyro, struct timespec *t);

	void handle_read() override;
	bool handle_canwrite() override;

private:
	SPI *_spi;

	double _accel_scale;
	double _gyro_scale;

	Point3_<double> _gyro_integrated;
	struct timespec _gyro_last_update;

	Point3_<double> _gyro_offsets;
	uint32_t _calibration_samples_counter = 0;
	int _calibration_load();
	int _calibration_save();

	bool write_register(uint8_t reg, uint8_t val);
	bool read_register(uint8_t reg, uint8_t *revc, uint16_t recv_len);

	bool _configure_accel();
	bool _configure_gyro();
    bool _configure_fifo();
    void _read_fifo();
};
