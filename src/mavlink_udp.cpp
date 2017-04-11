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

#include "mavlink_udp.h"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <mavlink.h>

#include "log.h"

int Mavlink_UDP::init(const char *ip, unsigned long port)
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (_fd == -1) {
		ERROR("Unable to create socket.");
		return -1;
	}

	memset(&_sockaddr, 0, sizeof(struct sockaddr_in));

	_sockaddr.sin_family = AF_INET;
	_sockaddr.sin_addr.s_addr = inet_addr(ip);
	_sockaddr.sin_port = htons(port);
	if (fcntl(_fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		ERROR("Error setting socket fd as non-blocking");
		goto network_fcntl_error;
	}

	if (bind(_fd, (struct sockaddr *)&_sockaddr, sizeof(sockaddr_in))) {
		ERROR("Unable to bind to socket");
		goto network_bind_error;
	}

	DEBUG("Mavlink UDP initialized %s:%lu", ip, port);

	return 0;

network_bind_error:
network_fcntl_error:
	close(_fd);
	_fd = -1;
	return -1;
}

Mavlink_UDP::~Mavlink_UDP()
{
	if (_fd == -1) {
		return;
	}

	close(_fd);
	_fd = -1;
}

void Mavlink_UDP::handle_read()
{
	socklen_t addrlen = sizeof(sockaddr);
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	ssize_t ret = ::recvfrom(_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&_sockaddr, &addrlen);

	if (ret < 1) {
		return;
	}

	mavlink_message_t msg;
	mavlink_status_t status;

	for (int i = 0; i < ret; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
			_handle(&msg);
		}
	}
}

void Mavlink_UDP::_handle(mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
		_handle_highres_imu(msg);
		return;
	}
}

void Mavlink_UDP::_handle_highres_imu(mavlink_message_t *msg)
{
	mavlink_highres_imu_t highres_imu;
	mavlink_msg_highres_imu_decode(msg, &highres_imu);

	// TODO
}

bool Mavlink_UDP::handle_canwrite()
{
	return false;
}
