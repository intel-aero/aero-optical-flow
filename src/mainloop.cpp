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
#include "mainloop.h"

#include "log.h"

#include <poll.h>

#define POLL_ERROR_EVENTS (POLLERR | POLLHUP | POLLNVAL)

int Mainloop::loop(Pollable *pollables[], size_t len, volatile bool *should_run, int timeout)
{
	struct pollfd desc[len];

	*should_run = true;

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN | POLLPRI | POLL_ERROR_EVENTS;
		desc[i].revents = 0;
	}

	while (*should_run) {
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), timeout);
		if (ret == 0) {
			if (_timeout_callback) {
				_timeout_callback((void *)_timeout_callback_data);
			}
			continue;
		}

		for (unsigned i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			const struct pollfd *d = &desc[i];

			for (uint8_t j = 0; j < len; j++) {
				if (d->fd == pollables[j]->_fd) {
					if (d->revents & (POLLIN | POLLPRI)) {
						pollables[j]->handle_read();
					}
					if (d->revents & POLLOUT) {
						pollables[j]->handle_canwrite();
					}
					if (d->revents & POLL_ERROR_EVENTS) {
						ERROR("Poll error event: %u fd: %i", d->revents, d->fd);
					}
					break;
				}
			}
		}
	}

	return 0;
}

void Mainloop::loop_timeout_callback_set(void (*callback)(void *data), const void *data)
{
	_timeout_callback = callback;
	_timeout_callback_data = data;
}
