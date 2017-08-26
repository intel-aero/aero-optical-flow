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

#include <stdint.h>
#include <stddef.h>

#include "pollable.h"

#define BUFFER_LEN 4

class Camera : public Pollable {
public:
	uint32_t width, height, pixel_format;
	int device_id;

	Camera(const char *device);
	virtual ~Camera();

	int init(int device_id, uint32_t width, uint32_t height, uint32_t pixel_format);
	int start();
	void stop();
	void shutdown() { return _shutdown(false); };
	int restart();

	int exposure_set(uint16_t value);
	uint16_t exposure_get();

	int gain_set(uint8_t value);
	uint8_t gain_get();

	void callback_set(void (*callback)(const void *img, size_t len, const struct timeval *timestamp, void *data), const void *data);

	void handle_read() override;
	bool handle_canwrite() override;

private:
	char *_device;
	void *_buffers[BUFFER_LEN];
	size_t _buffer_len = 0;

	void (*_callback)(const void *img, size_t len, const struct timeval *timestamp, void *data) = NULL;
	const void *_callback_data;

	uint16_t _exposure_value = 0;
	uint8_t _gain_value = 0;

	int _backend_user_ptr_streaming_init(uint32_t sizeimage);
	void _backend_user_ptr_streaming_read();
	void _backend_user_ptr_streaming_shutdown();

	int _fd_open();
	void _fd_close();
	void _shutdown(bool soft_reset);
};
