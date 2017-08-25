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
	if (_fd != -1) {
		_fd_close();
	}
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
	if (xioctl(_fd, VIDIOC_REQBUFS, &req)) {
		goto error;
	}

	if (!_buffer_len) {
		// allocate buffer
		pagesize = getpagesize();
		_buffer_len = (sizeimage + pagesize - 1) & ~(pagesize - 1);

		uint8_t i = 0;
		for (; i < BUFFER_LEN; i++) {
			_buffers[i] = memalign(pagesize, _buffer_len);
			if (!_buffers[i]) {
				_buffer_len = 0;
				while (i) {
					i--;
					free(_buffers[i]);
					_buffers[i] = NULL;
				}

				goto error;
			}
		}
#if DEBUG_LEVEL
		DEBUG("pagesize=%i buffer_len=%li", pagesize, _buffer_len);
#endif
	}

	// give buffers to backend
	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;
	buf.length = _buffer_len;

	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		buf.index = i;
		buf.m.userptr = (unsigned long)_buffers[i];
		if (xioctl(_fd, VIDIOC_QBUF, &buf)) {
			ERROR("Error giving buffers to backend: %s | i=%i", strerror(errno), i);
			goto backend_error;
		}
	}

	return 0;

backend_error:
	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		free(_buffers[i]);
		_buffers[i] = NULL;
	}
	_buffer_len = 0;
error:
	return -1;
}

int Camera::_fd_open()
{
	struct stat st;

	int ret = stat(_device, &st);
	if (ret) {
		ERROR("Unable to get device stat");
		return -1;
	}

	if (!S_ISCHR(st.st_mode)) {
		ERROR("Device is not a character device");
		return -1;
	}

	_fd = open(_device, O_RDWR | O_NONBLOCK);
	if (_fd == -1) {
		ERROR("Unable to open device file descriptor: %s", _device);
		return -1;
	}

	return 0;
}

void Camera::_fd_close()
{
	if (_fd == -1) {
		return;
	}

	close(_fd);
	_fd = -1;
}

int Camera::restart()
{
	stop();
	_shutdown(true);
	if (init(this->device_id, this->width, this->height, this->pixel_format)) {
		return -1;
	}
	return start();
}

int Camera::init(int id, uint32_t w, uint32_t h, uint32_t pf)
{
	struct v4l2_streamparm parm;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	int ret = -1;

	if (_fd == -1) {
		ret = _fd_open();
		if (ret) {
			goto error;
		}
	}

	// set device_id
	ret = xioctl(_fd, VIDIOC_S_INPUT, (int *)&id);
	if (ret) {
		ERROR("Error setting device id: %s", strerror(errno));
		goto error;
	}

	stop();

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
	fmt.fmt.pix.width = w;
	fmt.fmt.pix.height = h;
	fmt.fmt.pix.pixelformat = pf;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	ret = xioctl(_fd, VIDIOC_S_FMT, &fmt);
	if (ret) {
		ERROR("Setting pixel format: %s", strerror(errno));
		goto error;
	}
#if DEBUG_LEVEL
	DEBUG("size image=%u", fmt.fmt.pix.sizeimage);
#endif

	/* if restarting set the previous exposure value */
	if (_exposure_value != 0) {
		exposure_set(_exposure_value);
	} else {
		struct v4l2_control ctrl;

		ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
		ctrl.value = _exposure_value;
		ret = xioctl(_fd, VIDIOC_G_CTRL, &ctrl);
		if (ret) {
			ERROR("Getting exposure: %s", strerror(errno));
			goto error;
		}

		_exposure_value = ctrl.value;
	}

	ret = _backend_user_ptr_streaming_init(fmt.fmt.pix.sizeimage);
	if (ret) {
		ERROR("Error initializing streaming backend: %s", strerror(errno));
		goto error;
	}

	this->width = w;
	this->height = h;
	this->device_id = id;
	this->pixel_format = pf;

	return 0;

error:
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
		ERROR("Error getting frame from camera: %s", strerror(errno));
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

void Camera::_backend_user_ptr_streaming_shutdown()
{
	for (uint8_t i = 0; i < BUFFER_LEN; i++) {
		free(_buffers[i]);
		_buffers[i] = NULL;
	}
	_buffer_len = 0;
}

void Camera::_shutdown(bool soft_reset)
{
	if (soft_reset) {
		return;
	}

	_backend_user_ptr_streaming_shutdown();
	_fd_close();
}

void Camera::callback_set(void (*callback)(const void *img, size_t len, const struct timeval *timestamp, void *data), const void *data)
{
	_callback = callback;
	_callback_data = data;
}

int Camera::start()
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	int ret = xioctl(_fd, VIDIOC_STREAMON, &type);
	if (ret) {
		ERROR("Error starting streaming: %s", strerror(errno));
	}

	return ret;
}

void Camera::stop()
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (xioctl(_fd, VIDIOC_STREAMOFF, &type)) {
		ERROR("Error stopping streaming: %s", strerror(errno));
	}
}

int Camera::exposure_set(uint16_t value)
{
	struct v4l2_control ctrl;
	int ret;

	ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	ctrl.value = value;
	ret = xioctl(_fd, VIDIOC_S_CTRL, &ctrl);
	if (ret) {
		ERROR("Error setting exposure: %s", strerror(errno));
		return ret;
	}

	_exposure_value = value;
	return 0;
}

uint16_t Camera::exposure_get()
{
	return _exposure_value;
}
