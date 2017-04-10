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

#include <poll.h>
#include <signal.h>
#include <stdio.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "camera.h"

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

static void loop(int fd)
{
	struct pollfd desc[1];

	signal_handlers_setup();
	should_run = true;

	desc[0].events = POLLIN;
	desc[0].fd = fd;
	desc[0].revents = 0;

	while (should_run) {
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), -1);
		if (ret < 1) {
			continue;
		}

		for (int i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			if (desc[i].revents & POLLIN) {
				camera_frame_read(desc[i].fd);
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

static void video_callback(int fd, const void *img, size_t len, void *data)
{
#if DEBUG_LEVEL
	image_show(img, len);
#endif
}

int main()
{
	int fd = camera_open(default_device);
	if (fd == -1) {
		goto open_error;
	}
	if (camera_init(fd, DEFAULT_DEVICE_ID, DEFAULT_IMG_WIDTH,
			DEFAULT_IMG_HEIGHT, DEFAULT_PIXEL_FORMAT)) {
		goto init_error;
	}
	camera_callback_set(video_callback, NULL);

#if DEBUG_LEVEL
	namedWindow(window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	loop(fd);

	camera_shutdown(fd);
	camera_close(fd);

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	return 0;

init_error:
	camera_close(fd);
open_error:
	return -1;
}
