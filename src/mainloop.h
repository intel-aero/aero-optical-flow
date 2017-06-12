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

#include <flow_opencv.hpp>
#include <mavlink.h>

#include "bmi160.h"
#include "camera.h"
#include "config.h"
#include "mavlink_udp.h"

class Mainloop {
public:
	int init(const char *camera_device, int camera_id, uint32_t camera_width,
			uint32_t camera_height, uint32_t crop_width, uint32_t crop_height,
			unsigned long mavlink_udp_port, int flow_output_rate,
			float focal_length_x, float focal_length_y, bool calibrate_bmi,
			const char *parameters_folder);
	int run();
	void shutdown();

	void camera_callback(const void *img, size_t len, const struct timeval *timestamp);
	void timestamp_vehicle_set(uint64_t time_usec);
	void *camera_thread();

private:

#if DEBUG_LEVEL
	const char *_window_name = "Aero down face camera test";
#endif

	uint64_t _camera_initial_timestamp = 0;
	uint64_t _camera_prev_timestamp = 0;
	uint64_t _offset_timestamp_usec = 0;
	uint64_t _next_exposure_update_timestap = 0;

	float _exposure_msv_error_int = 0.0f;
	float _exposure_msv_error_old = 0.0f;

	Camera *_camera;
	OpticalFlowOpenCV *_optical_flow;
	Mavlink_UDP *_mavlink;
	BMI160 *_bmi;

	pthread_mutex_t _mainloop_lock;

	struct timespec _gyro_last_timespec = {};

	void _signal_handlers_setup();
	void _loop();

	void _exposure_update(Mat frame, uint64_t timestamp);
};
