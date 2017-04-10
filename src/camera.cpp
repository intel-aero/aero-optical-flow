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

#include "camera.h"

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "log.h"

// TODO handle buffers for multiple cameras
#define BUFFER_LEN 4
static void *buffers[BUFFER_LEN];
static size_t buffer_len;

static void (*video_callback)(int fd, const void *img, size_t len, void *data);
static const void *video_callback_data;

// TODO: Specific values for Intel?
enum {
	CAPTURE_MODE_PREVIEW = 0x8000,
	CAPTURE_MODE_VIDEO = 0x4000,
	CAPTURE_MODE_STILL = 0x2000
};

static int xioctl(int fd, int request, void *arg)
{
	int r;

	do {
		r = ioctl(fd, request, arg);
	} while (-1 == r && EINTR == errno);

    return r;
}

static int _backend_user_ptr_streaming_init(int fd, uint32_t sizeimage)
{
	struct v4l2_requestbuffers req;
	struct v4l2_buffer buf;
	int pagesize;

	// initialize v4l2 backend
	req.count = BUFFER_LEN;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;
	int ret = xioctl(fd, VIDIOC_REQBUFS, &req);
	if (ret) {
		goto error;
	}

	// allocate buffer
	pagesize = getpagesize();
	buffer_len = (sizeimage + pagesize - 1) & ~(pagesize - 1);
	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		buffers[i] = memalign(pagesize, buffer_len);
		if (!buffers[i]) {
			goto error;
		}
	}
#if DEBUG_LEVEL
	DEBUG("pagesize=%i buffer_len=%u", pagesize, buffer_len);
#endif

	// give buffers to backend
	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.length = buffer_len;

	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		buf.index = i;
		buf.m.userptr = (unsigned long)buffers[i];
		ret = xioctl(fd, VIDIOC_QBUF, &buf);
		if (ret) {
			ERROR("Error giving buffers to backend: %s | i=%i", strerror(errno), i);
			goto error;
		}
	}

	ret = 0;

error:
	return ret;
}

int camera_init(int fd, int device_id, uint32_t width, uint32_t height, uint32_t pixel_format)
{
	struct v4l2_streamparm parm;
	struct v4l2_capability cap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret;

	// set device_id
	ret = xioctl(fd, VIDIOC_S_INPUT, (int *)&device_id);
	if (ret) {
		ERROR("Error setting device id: %s", strerror(errno));
		goto error;
	}

	ret = xioctl(fd, VIDIOC_STREAMOFF, &type);
	if (ret) {
		ERROR("Error stopping streaming: %s", strerror(errno));
		goto error;
	}

	// get capabilities and check if can be used to capture
	ret = xioctl(fd, VIDIOC_QUERYCAP, &cap);
	if (ret) {
		ERROR("Error getting capabilities");
		goto error;
	}
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		ERROR("Video capture not supported: %s", strerror(errno));
		goto error;
	}
#if DEBUG_LEVEL
	if (cap.capabilities & V4L2_CAP_READWRITE) {
		DEBUG("support sync read/write");
	}
	if (cap.capabilities & V4L2_CAP_ASYNCIO) {
		DEBUG("support async read/write");
	}
	if (cap.capabilities & V4L2_CAP_STREAMING) {
		DEBUG("streaming I/O ioctls");
	}
#endif

	// set stream parameters
	memset(&parm, 0, sizeof(struct v4l2_streamparm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parm.parm.capture.capturemode = CAPTURE_MODE_PREVIEW;
	ret = xioctl(fd, VIDIOC_S_PARM, &parm);
	if (ret) {
		ERROR("Unable to set stream parameters: %s", strerror(errno));
		goto error;
	}

	// set pixel format
	memset(&fmt, 0, sizeof(struct v4l2_format));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	DEBUG("w=%u h=%u", width, height);
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = pixel_format;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	ret = xioctl(fd, VIDIOC_S_FMT, &fmt);
	if (ret) {
		ERROR("Setting pixel format: %s", strerror(errno));
		goto error;
	}
#if DEBUG_LEVEL
	DEBUG("size image=%u", fmt.fmt.pix.sizeimage);
#endif

	ret = _backend_user_ptr_streaming_init(fd, fmt.fmt.pix.sizeimage);
	if (ret) {
		ERROR("Error initializing streaming backend: %s", strerror(errno));
		goto error;
	}

	// finally start streaming
	ret = xioctl(fd, VIDIOC_STREAMON, &type);
	if (ret) {
		ERROR("Error starting streaming: %s", strerror(errno));
		goto error_starting;
	}

	return 0;
error_starting:
	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		free(buffers[i]);
	}
error:
	return ret;
}

int camera_open(const char *device)
{
	struct stat st;
	int ret = -1;

	ret = stat(device, &st);
	if (ret) {
		ERROR("Error getting device stat");
		goto end;
	}

	if (!S_ISCHR(st.st_mode)) {
		ERROR("Device is not a character device");
		goto end;
	}

	ret = open(device, O_RDWR | O_NONBLOCK);
	if (ret == -1) {
		ERROR("Error opening device file descriptor");
	}

end:
	return ret;
}

static void _backend_user_ptr_streaming_read(int fd)
{
	struct v4l2_buffer buf;

	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;

	int ret = xioctl(fd, VIDIOC_DQBUF, &buf);
	if (ret) {
		return;
	}

	if (video_callback) {
		video_callback(fd, (const void *)buf.m.userptr, buf.length, (void *)video_callback_data);
	}

	// give buffer back to backend
	xioctl(fd, VIDIOC_QBUF, &buf);
}

void camera_frame_read(int fd)
{
	_backend_user_ptr_streaming_read(fd);
}

void camera_shutdown(int fd)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	// stop streaming
	xioctl(fd, VIDIOC_STREAMOFF, &type);

	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		free(buffers[i]);
	}
}

void camera_close(int fd)
{
	close(fd);
}

void camera_callback_set(void (*callback)(int fd, const void *img, size_t len, void *data), const void *data)
{
	video_callback = callback;
	video_callback_data = data;
}
