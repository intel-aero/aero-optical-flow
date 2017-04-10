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
#include <sys/epoll.h>

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
static int epollfd;

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

static void loop(void)
{
	const int max_events = 8;
	struct epoll_event events[max_events];

	signal_handlers_setup();
	should_run = true;

	while (should_run) {
		int ret = epoll_wait(epollfd, events, max_events, -1);
		if (ret < 1) {
			continue;
		}

		for (int i = 0; ret; i++, ret--) {
			Pollable *p = static_cast<Pollable *>(events[i].data.ptr);
			if (events[i].events & EPOLLIN) {
				p->handle_read();
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

static void video_callback(const void *img, size_t len, void *data)
{
#if DEBUG_LEVEL
	image_show(img, len);
#endif
}

static int fd_add(int fd, void *data, int events)
{
    struct epoll_event epev = { };

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &epev) < 0) {
        ERROR("Could not add fd to epoll.");
        return -1;
    }

    return 0;
}

static int fd_del(int fd)
{
	if (epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, NULL) < 0) {
		ERROR("Could not remove fd from epoll");
		return -1;
	}

	return 0;
}

static int fd_mod(int fd, void *data, int events)
{
    struct epoll_event epev = { };

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &epev) < 0) {
        ERROR("Could not mod fd");
        return -1;
    }

    return 0;
}

int main()
{
	Camera *camera;
	Mavlink_UDP *mavlink;
	OpticalFlowOpenCV *optical_flow;
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
	camera->callback_set(video_callback, NULL);

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

	epollfd = epoll_create1(EPOLL_CLOEXEC);
	if (epollfd == -1) {
		ERROR("Unable to create epoll");
		goto epoll_error;
	}

	fd_add(camera->_fd, camera, EPOLLIN);
	fd_add(mavlink->_fd, mavlink, EPOLLIN);

#if DEBUG_LEVEL
	namedWindow(window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	loop();

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	fd_del(camera->_fd);
	fd_del(mavlink->_fd);

	delete optical_flow;
	delete mavlink;
	camera->shutdown();
	delete camera;

	return 0;

epoll_error:
	delete optical_flow;
optical_memory_error:
mavlink_init_error:
	delete mavlink;
mavlink_memory_error:
	camera->shutdown();
camera_init_error:
	delete camera;
	return -1;
}
