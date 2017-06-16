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
#include <stdbool.h>

#define DEFAULT_RESOLUTION 1
#define DEFAULT_IMG_WIDTH 320
#define DEFAULT_IMG_HEIGHT 240
#define DEFAULT_IMG_CROP_WIDTH 128
#define DEFAULT_IMG_CROP_HEIGHT 128
#define DEFAULT_FLOW_OUTPUT_RATE 15
#define DEFAULT_FOCAL_LENGTH_X 216.6677
#define DEFAULT_FOCAL_LENGTH_Y 216.2457

#define DEFAULT_DEVICE_FILE "/dev/video2"
#define DEFAULT_DEVICE_ID 1

#define MAVLINK_TCP_PORT 5760

#define DEFAULT_PARAMETERS_FOLDER "."

extern const char *camera_device;
extern unsigned long camera_dev_id;
extern unsigned long camera_width;
extern unsigned long camera_height;
extern unsigned long crop_width;
extern unsigned long crop_height;
extern unsigned long mavlink_tcp_port;
extern int flow_output_rate;
extern float focal_length_x;
extern float focal_length_y;
extern bool bmi160_calibrate;
extern const char *parameter_folder;

int parse_args(int argc, char *argv[]);
