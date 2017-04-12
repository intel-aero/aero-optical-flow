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

#if DEBUG_LEVEL
const char *window_name = "Aero down face camera test";
#endif

// OV7251 only supports this resolutions each with a different FPS
struct {
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

#define DEFAULT_PIXEL_FORMAT V4L2_PIX_FMT_YUV420
#define DEFAULT_DEVICE_ID 1

static struct gyro_data_t {
	float x, y, z;
	uint64_t time_usec;
} gyro_data;
static bool ready_to_send_msg;
static Mavlink_UDP *mavlink;

static const char *default_device = "/dev/video2";

static bool should_run;

static void exit_signal_handler(int signum)
{
    should_run = false;
}

static void signal_handlers_setup(void)
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

static void loop(Pollable *pollables[], uint8_t len)
{
	struct pollfd desc[len];

	signal_handlers_setup();
	should_run = true;

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN;
		desc[i].revents = 0;
	}

	while (should_run) {
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

static void camera_callback(const void *img, size_t len, struct timeval *timestamp, void *data)
{
	int dt_us;
	float x = 0, y = 0;
	OpticalFlowOpenCV *optical_flow = (OpticalFlowOpenCV *)data;

	Mat frame_gray = Mat(DEFAULT_IMG_HEIGHT, DEFAULT_IMG_WIDTH, CV_8UC1);
	frame_gray.data = (uchar*)img;

#if DEBUG_LEVEL
	imshow(window_name, frame_gray);
#endif

	DEBUG("camera callback timestamp: sec=%lu usec=%lu", timestamp->tv_sec, timestamp->tv_usec);

	int quality = optical_flow->calcFlow(frame_gray.data, timestamp->tv_sec, dt_us, x, y);
	DEBUG("Optical flow data: quality=%i x=%f y=%f dt_us=%i", quality, x, y, dt_us);

	if (!ready_to_send_msg) {
		DEBUG("Not ready to send optical flow message");
		return;
	}

	mavlink_optical_flow_rad_t msg;
	msg.time_usec = timestamp->tv_usec + timestamp->tv_sec * USEC_PER_SEC;
	msg.integration_time_us = dt_us;
	msg.integrated_x = x;
	msg.integrated_y = y;
	msg.integrated_xgyro = gyro_data.x;
	msg.integrated_ygyro = gyro_data.y;
	msg.integrated_zgyro = gyro_data.z;
	msg.time_delta_distance_us = 0;
	msg.distance = -1.0;
	msg.temperature = 0;
	msg.sensor_id = 0;
	msg.quality = quality;

	mavlink->optical_flow_rad_msg_write(&msg);
}

static void highres_imu_msg_subscribe(const mavlink_highres_imu_t *msg, void *data)
{
	gyro_data.x = msg->xgyro;
	gyro_data.y = msg->ygyro;
	gyro_data.z = msg->zgyro;
	gyro_data.time_usec = msg->time_usec;
	ready_to_send_msg = true;//TODO check liveness
}

int main()
{
	Camera *camera;
	OpticalFlowOpenCV *optical_flow;
	Pollable *pollables[2];
	int ret;

	camera = new Camera(default_device);
	if (!camera) {
		ERROR("No memory to instantiate Camera");
		return -1;
	}
	ret = camera->init(DEFAULT_DEVICE_ID, DEFAULT_IMG_WIDTH, DEFAULT_IMG_HEIGHT, DEFAULT_PIXEL_FORMAT);
	if (ret) {
		ERROR("Unable to initialize camera");
		goto camera_init_error;
	}

	mavlink = new Mavlink_UDP();
	if (!mavlink) {
		ERROR("No memory to instantiate Mavlink_UDP");
		goto mavlink_memory_error;
	}
	ret = mavlink->init("127.0.0.1", 14555);
	if (ret) {
		ERROR("Unable to initialize mavlink");
		goto mavlink_init_error;
	}
	mavlink->highres_imu_msg_subscribe(highres_imu_msg_subscribe, NULL);

	// TODO: get the real value of f_length_x and f_length_y
	// TODO:  check that image format it uses
	optical_flow = new OpticalFlowOpenCV(1, 1, -1, DEFAULT_IMG_WIDTH, DEFAULT_IMG_HEIGHT);
	if (!optical_flow) {
		ERROR("No memory to instantiate OpticalFlowOpenCV");
		goto optical_memory_error;
	}
	camera->callback_set(camera_callback, optical_flow);

#if DEBUG_LEVEL
	namedWindow(window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	pollables[0] = camera;
	pollables[1] = mavlink;
	loop(pollables, 2);

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	delete optical_flow;
	delete mavlink;
	camera->shutdown();
	delete camera;

	return 0;

optical_memory_error:
mavlink_init_error:
	delete mavlink;
mavlink_memory_error:
	camera->shutdown();
camera_init_error:
	delete camera;
	return -1;
}
