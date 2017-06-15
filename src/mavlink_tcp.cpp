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

#include "mavlink_tcp.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <mavlink.h>

#include "log.h"

int Mavlink_TCP::init(const char *ip, unsigned long port)
{
	int ret;

	_fd = socket(AF_INET, SOCK_STREAM, 0);
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

	ret = connect(_fd, (struct sockaddr *)&_sockaddr, sizeof(struct sockaddr_in));
	if (ret && errno != EINPROGRESS) {
		ERROR("Unable to connect to socket errno=%i", errno);
		goto network_bind_error;
	}

	DEBUG("Mavlink TCP initialized %s:%lu", ip, port);

	return 0;

network_bind_error:
network_fcntl_error:
	close(_fd);
	_fd = -1;
	return -1;
}

Mavlink_TCP::~Mavlink_TCP()
{
	if (_fd == -1) {
		return;
	}

	close(_fd);
	_fd = -1;
}

void Mavlink_TCP::handle_read()
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

void Mavlink_TCP::_handle(mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_HIGHRES_IMU && _highres_imu_msg_callback) {
		mavlink_highres_imu_t highres_imu;
		mavlink_msg_highres_imu_decode(msg, &highres_imu);

		_highres_imu_msg_callback(&highres_imu, (void *)_highres_imu_msg_callback_data);
		return;
	}
}

void Mavlink_TCP::highres_imu_msg_subscribe(void (*callback)(const mavlink_highres_imu_t *msg, void *data), const void *data)
{
	_highres_imu_msg_callback = callback;
	_highres_imu_msg_callback_data = data;
}

bool Mavlink_TCP::handle_canwrite()
{
	return false;
}

int Mavlink_TCP::optical_flow_rad_msg_write(mavlink_optical_flow_rad_t *optical_msg)
{
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_optical_flow_rad_encode(_system_id, _component_id, &msg, optical_msg);
	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

	ssize_t r = sendto(_fd, buffer, len, 0, (struct sockaddr *)&_sockaddr, sizeof(_sockaddr));
	if (r == -1) {
		ERROR("Error sending mavlink_optical_flow_rad_t: %s", strerror(errno));
		return -1;
	}

	if (r != len) {
		ERROR("mavlink_optical_flow_rad_t was send incomplete");
		return -1;
	}

	return 0;
}
