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
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "config.h"
#include "mavlink_tcp.h"
#ifdef RTPS
#include <vehicle_command_Publisher.h>
#include <sensor_combined_Subscriber.h>
#include <optical_flow_Publisher.h>
#endif

using namespace cv;

class Mainloop {
public:
	int init(const char *camera_device, int camera_id, uint32_t camera_width,
			uint32_t camera_height, uint32_t crop_width, uint32_t crop_height,
			const char *mavlink_tcp_ip, unsigned long mavlink_tcp_port,
			int flow_output_rate, float focal_length_x, float focal_length_y);
	int run();
	void shutdown();

	void camera_callback(const void *img, size_t len, const struct timeval *timestamp);
	void highres_imu_msg_callback(const mavlink_highres_imu_t *msg);
	void *camera_thread();

private:

#if DEBUG_LEVEL
	const char *_window_name = "Aero down face camera test";
#endif

	uint64_t _camera_initial_timestamp = 0;
	uint64_t _camera_prev_timestamp = 0;
	uint64_t _offset_timestamp_usec = 0;
	uint64_t _next_exposure_update_timestap = 0;

	float _msv_error_int = 0.0f;
	float _msv_error_old = 0.0f;

	Point3_<double> _gyro_integrated = {};
	uint64_t _gyro_prev_timestamp = 0;

	Camera *_camera;
	OpticalFlowOpenCV *_optical_flow;
	Mavlink_TCP *_mavlink;
#ifdef RTPS
	vehicle_command_Publisher *_cmd_pub;
	optical_flow_Publisher *_of_pub;
	sensor_combined_Subscriber *_sen_sub;
#endif

	pthread_mutex_t _mainloop_lock;

	uint64_t _gyro_last_usec_timestamp = 0;

	void _signal_handlers_setup();
	void _loop();
	void _loop_rtps();

	void _exposure_update(Mat frame, uint64_t timestamp);
};
