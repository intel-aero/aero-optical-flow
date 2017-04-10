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

#include "config.h"
#include "camera.h"
#include "mavlink_udp.h"
#include "log.h"

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

#if DEBUG_LEVEL
static void image_show(const void *img, size_t len)
{
	Mat input(DEFAULT_IMG_HEIGHT , DEFAULT_IMG_WIDTH, CV_8UC1, (unsigned char *)img);
	imshow(window_name, input);
}
#endif

static void camera_callback(const void *img, size_t len, void *data)
{
#if DEBUG_LEVEL
	image_show(img, len);
#endif
}

int main()
{
	Camera *camera;
	Mavlink_UDP *mavlink;
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

	optical_flow = new OpticalFlowOpenCV(0, 0, 0);
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
