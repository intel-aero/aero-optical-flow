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

#include <signal.h>
#include <stdio.h>
#include <poll.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include <flow_opencv.hpp>
#include <flow_px4.hpp>

#include <mavlink.h>

#include "config.h"
#include "camera.h"
#include "mavlink_udp.h"
#include "log.h"
#include "util.h"

using namespace cv;

/*
 * OV7251 only supports this resolutions each with a different FPS according
 * with datasheet but we are only being able to get 30fps in any of those
 * resolution, this can be a problem with atomisp.
 */
static struct {
	uint32_t width;
	uint32_t height;
} resolutions[] = {
		{640, 480},// some glitches in the right and bottom
		{320, 240},
		{160, 120},
};

#define DEFAULT_RESOLUTION 1
#define DEFAULT_IMG_WIDTH resolutions[DEFAULT_RESOLUTION].width
#define DEFAULT_IMG_HEIGHT resolutions[DEFAULT_RESOLUTION].height
#define DEFAULT_IMG_CROP_WIDTH 128
#define DEFAULT_IMG_CROP_HEIGHT 128
#define DEFAULT_FLOW_OUTPUT_RATE 15

#define DEFAULT_PIXEL_FORMAT V4L2_PIX_FMT_YUV420
#define DEFAULT_DEVICE_ID 1

class Mainloop {
public:
	int run();

	void camera_callback(const void *img, size_t len, const struct timeval *timestamp);
	void highres_imu_msg_callback(const mavlink_highres_imu_t *msg);

private:

#if DEBUG_LEVEL
	const char *_window_name = "Aero down face camera test";
#endif
	const char *default_device = "/dev/video2";

	struct gyro_data_t {
		float x, y, z;
		uint64_t time_usec;
	} _gyro_data;

	bool _ready_to_send_msg = false;

	uint32_t _camera_initial_timestamp = 0;
	uint32_t _camera_prev_timestamp = 0;

	Camera *_camera;
	OpticalFlowOpenCV *_optical_flow;
	Mavlink_UDP *_mavlink;

	void signal_handlers_setup();
	void loop();
};

static bool _should_run;

static void exit_signal_handler(int signum)
{
    _should_run = false;
}

void Mainloop::signal_handlers_setup(void)
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

void Mainloop::loop()
{
	Pollable *pollables[] = { _camera, _mavlink };
	const uint8_t len = sizeof(pollables) / sizeof(Pollable *);
	struct pollfd desc[len];

	signal_handlers_setup();
	_should_run = true;

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN;
		desc[i].revents = 0;
	}

	while (_should_run) {
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), -1);
		if (ret < 1) {
			continue;
		}

		for (int i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			for (uint8_t j = 0; j < len; j++) {
				if (desc[i].fd == pollables[j]->_fd) {
					if (desc[i].revents & (POLLIN | POLLPRI)) {
						pollables[j]->handle_read();
					}
					if (desc[i].revents & POLLOUT) {
						pollables[j]->handle_canwrite();
					}
					break;
				}
			}
		}
	}
}

static void _camera_callback(const void *img, size_t len, const struct timeval *timestamp, void *data)
{
	Mainloop *mainloop = (Mainloop *)data;
	mainloop->camera_callback(img, len, timestamp);
}

void Mainloop::camera_callback(const void *img, size_t len, const struct timeval *timestamp)
{
	int dt_us = 0;
	float flow_x_ang = 0, flow_y_ang = 0;

	Mat frame_gray = Mat(DEFAULT_IMG_HEIGHT, DEFAULT_IMG_WIDTH, CV_8UC1);
	frame_gray.data = (uchar*)img;
	//crop the image (optical flow assumes narrow field of view)
	static cv::Rect crop(DEFAULT_IMG_WIDTH/2 - DEFAULT_IMG_CROP_WIDTH/2, DEFAULT_IMG_HEIGHT/2 - DEFAULT_IMG_CROP_HEIGHT/2,
			DEFAULT_IMG_CROP_WIDTH, DEFAULT_IMG_CROP_HEIGHT);
	cv::Mat croppedImage = frame_gray(crop);

#if DEBUG_LEVEL
	imshow(_window_name, frame_gray);
#endif

	uint32_t img_time_us = timestamp->tv_usec + timestamp->tv_sec * USEC_PER_SEC;
	float fps = 0;

	if (_camera_initial_timestamp) {
		img_time_us -= _camera_initial_timestamp;
		fps = 1.0f / ((float)(img_time_us - _camera_prev_timestamp) / USEC_PER_SEC);
	} else {
		_camera_initial_timestamp = img_time_us;
		img_time_us = 0;
	}

	int flow_quality = _optical_flow->calcFlow(croppedImage.data, img_time_us, dt_us, flow_x_ang, flow_y_ang);

#if DEBUG_LEVEL
	DEBUG("Optical flow quality=%i x=%f y=%f timestamp sec=%lu usec=%lu fps=%f", flow_quality, flow_x_ang, flow_y_ang,
		img_time_us / USEC_PER_SEC, img_time_us % USEC_PER_SEC, fps);
#endif

	_camera_prev_timestamp = img_time_us;

	if (!_ready_to_send_msg) {
		DEBUG("Not ready to send optical flow message");
		return;
	}

	if (flow_quality >= 0) { //check if flow is ready/integraed -> flow output rate
		mavlink_optical_flow_rad_t msg;

		msg.time_usec = timestamp->tv_usec + timestamp->tv_sec * USEC_PER_SEC;
		msg.integration_time_us = dt_us;
		msg.integrated_x = flow_x_ang;
		msg.integrated_y = flow_y_ang;
		msg.integrated_xgyro = _gyro_data.x;
		msg.integrated_ygyro = _gyro_data.y;
		msg.integrated_zgyro = _gyro_data.z;
		msg.time_delta_distance_us = 0;
		msg.distance = -1.0;
		msg.temperature = 0;
		msg.sensor_id = 0;
		msg.quality = flow_quality;

		//send mavlink message
		_mavlink->optical_flow_rad_msg_write(&msg);
	}
}

static void _highres_imu_msg_callback(const mavlink_highres_imu_t *msg, void *data)
{
	Mainloop *mainloop = (Mainloop *)data;
	mainloop->highres_imu_msg_callback(msg);
}

void Mainloop::highres_imu_msg_callback(const mavlink_highres_imu_t *msg)
{
	_gyro_data.x = msg->xgyro;
	_gyro_data.y = msg->ygyro;
	_gyro_data.z = msg->zgyro;
	_gyro_data.time_usec = msg->time_usec;
	_ready_to_send_msg = true;//TODO check liveness
}

int Mainloop::run()
{
	int ret;

	_camera = new Camera(default_device);
	if (!_camera) {
		ERROR("No memory to instantiate Camera");
		return -1;
	}
	ret = _camera->init(DEFAULT_DEVICE_ID, DEFAULT_IMG_WIDTH, DEFAULT_IMG_HEIGHT, DEFAULT_PIXEL_FORMAT);
	if (ret) {
		ERROR("Unable to initialize camera");
		goto camera_init_error;
	}

	_mavlink = new Mavlink_UDP();
	if (!_mavlink) {
		ERROR("No memory to instantiate Mavlink_UDP");
		goto mavlink_memory_error;
	}
	ret = _mavlink->init("127.0.0.1", 14555);
	if (ret) {
		ERROR("Unable to initialize mavlink");
		goto mavlink_init_error;
	}
	_mavlink->highres_imu_msg_subscribe(_highres_imu_msg_callback, this);

	// TODO: get the real value of f_length_x and f_length_y
	// TODO:  check that image format it uses
	// TODO: user input for img width/height, rate, etc.
	_optical_flow = new OpticalFlowOpenCV(1, 1, DEFAULT_FLOW_OUTPUT_RATE, DEFAULT_IMG_CROP_WIDTH, DEFAULT_IMG_CROP_HEIGHT);
	if (!_optical_flow) {
		ERROR("No memory to instantiate OpticalFlowOpenCV");
		goto optical_memory_error;
	}
	_camera->callback_set(_camera_callback, this);

#if DEBUG_LEVEL
	namedWindow(_window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	loop();

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	delete _optical_flow;
	delete _mavlink;
	_camera->shutdown();
	delete _camera;

	return 0;

optical_memory_error:
mavlink_init_error:
	delete _mavlink;
mavlink_memory_error:
	_camera->shutdown();
camera_init_error:
	delete _camera;
	return -1;
}

int main()
{
	Mainloop mainloop;
	return mainloop.run();
}
