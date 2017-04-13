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
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "log.h"

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

Camera::Camera(const char *device)
{
	_device = strdup(device);
}

Camera::~Camera()
{
	free(_device);
}

int Camera::_backend_user_ptr_streaming_init(uint32_t sizeimage)
{
	struct v4l2_requestbuffers req;
	struct v4l2_buffer buf;
	int pagesize;

	// initialize v4l2 backend
	memset(&req, 0, sizeof(struct v4l2_requestbuffers));
	req.count = BUFFER_LEN;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;
	int ret = xioctl(_fd, VIDIOC_REQBUFS, &req);
	if (ret) {
		goto error;
	}

	// allocate buffer
	pagesize = getpagesize();
	_buffer_len = (sizeimage + pagesize - 1) & ~(pagesize - 1);
	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		_buffers[i] = memalign(pagesize, _buffer_len);
		if (!_buffers[i]) {
			goto error;
		}
	}
#if DEBUG_LEVEL
	DEBUG("pagesize=%i buffer_len=%u", pagesize, _buffer_len);
#endif

	// give buffers to backend
	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.length = _buffer_len;

	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		buf.index = i;
		buf.m.userptr = (unsigned long)_buffers[i];
		ret = xioctl(_fd, VIDIOC_QBUF, &buf);
		if (ret) {
			ERROR("Error giving buffers to backend: %s | i=%i", strerror(errno), i);
			goto error;
		}
	}

	ret = 0;

error:
	return ret;
}

int Camera::init(int device_id, uint32_t width, uint32_t height, uint32_t pixel_format)
{
	struct v4l2_streamparm parm;
	struct v4l2_capability cap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	struct stat st;
	int ret = -1;

	ret = stat(_device, &st);
	if (ret) {
		ERROR("Unable to get device stat");
		goto end;
	}

	if (!S_ISCHR(st.st_mode)) {
		ERROR("Device is not a character device");
		goto end;
	}

	_fd = open(_device, O_RDWR | O_NONBLOCK);
	if (_fd == -1) {
		ret = -1;
		ERROR("Unable to open device file descriptor: %s", _device);
		goto end;
	}

	// set device_id
	ret = xioctl(_fd, VIDIOC_S_INPUT, (int *)&device_id);
	if (ret) {
		ERROR("Error setting device id: %s", strerror(errno));
		goto error;
	}

	ret = xioctl(_fd, VIDIOC_STREAMOFF, &type);
	if (ret) {
		ERROR("Error stopping streaming: %s", strerror(errno));
		goto error;
	}

	// get capabilities and check if can be used to capture
	ret = xioctl(_fd, VIDIOC_QUERYCAP, &cap);
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
	ret = xioctl(_fd, VIDIOC_S_PARM, &parm);
	if (ret) {
		ERROR("Unable to set stream parameters: %s", strerror(errno));
		goto error;
	}

	// set pixel format
	memset(&fmt, 0, sizeof(struct v4l2_format));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = pixel_format;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	ret = xioctl(_fd, VIDIOC_S_FMT, &fmt);
	if (ret) {
		ERROR("Setting pixel format: %s", strerror(errno));
		goto error;
	}
#if DEBUG_LEVEL
	DEBUG("size image=%u", fmt.fmt.pix.sizeimage);
#endif

	ret = _backend_user_ptr_streaming_init(fmt.fmt.pix.sizeimage);
	if (ret) {
		ERROR("Error initializing streaming backend: %s", strerror(errno));
		goto error;
	}

	// finally start streaming
	ret = xioctl(_fd, VIDIOC_STREAMON, &type);
	if (ret) {
		ERROR("Error starting streaming: %s", strerror(errno));
		goto error_starting;
	}

	return 0;

error_starting:
	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		free(_buffers[i]);
	}
error:
	close(_fd);
end:
	return ret;
}

void Camera::_backend_user_ptr_streaming_read(void)
{
	struct v4l2_buffer buf;

	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;

	int ret = xioctl(_fd, VIDIOC_DQBUF, &buf);
	if (ret) {
		ERROR("Error getting frame from camera");
		return;
	}

	if (_callback) {
		_callback((const void *)buf.m.userptr, buf.length, &buf.timestamp, (void *)_callback_data);
	}

	// give buffer back to backend
	ret = xioctl(_fd, VIDIOC_QBUF, &buf);
	if (ret) {
		ERROR("Error returning buffer to backend");
	}
}

void Camera::handle_read(void)
{
	_backend_user_ptr_streaming_read();
}

bool Camera::handle_canwrite(void)
{
	return false;
}

int Camera::shutdown(void)
{
	if (_fd == -1) {
		return -1;
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	// stop streaming
	xioctl(_fd, VIDIOC_STREAMOFF, &type);

	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		free(_buffers[i]);
	}

	close(_fd);
	_fd = -1;

	return 0;
}

void Camera::callback_set(void (*callback)(const void *img, size_t len, const struct timeval *timestamp, void *data), const void *data)
{
	_callback = callback;
	_callback_data = data;
}
