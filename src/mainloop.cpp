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

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <poll.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include <flow_opencv.hpp>

#include <mavlink.h>

#include "bmi160.h"
#include "config.h"
#include "camera.h"
#include "mavlink_udp.h"
#include "log.h"
#include "util.h"

using namespace cv;

/*
 * OV7251 only supports this resolutions each with a different FPS according
 * with datasheet but we are only being able to get 30fps in any of those
 * resolution, this can be a problem with atomisp.
 */
static struct {
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
#define DEFAULT_IMG_CROP_WIDTH 128
#define DEFAULT_IMG_CROP_HEIGHT 128
#define DEFAULT_FLOW_OUTPUT_RATE 15
#define DEFAULT_FOCAL_LENGTH_X 216.6677
#define DEFAULT_FOCAL_LENGTH_Y 216.2457

#define DEFAULT_PIXEL_FORMAT V4L2_PIX_FMT_YUV420
#define DEFAULT_DEVICE_FILE "/dev/video2"
#define DEFAULT_DEVICE_ID 1

#define MAVLINK_UDP_PORT 14555

#define DEFAULT_PARAMETERS_FOLDER "."

class Mainloop {
public:
	int run(const char *camera_device, int camera_id, uint32_t camera_width,
			uint32_t camera_height, uint32_t crop_width, uint32_t crop_height,
			unsigned long mavlink_udp_port, int flow_output_rate,
			float focal_length_x, float focal_length_y, bool calibrate_bmi,
			const char *parameters_folder);

	void camera_callback(const void *img, size_t len, const struct timeval *timestamp);
	void highres_imu_msg_callback(const mavlink_highres_imu_t *msg);

private:

#if DEBUG_LEVEL
	const char *_window_name = "Aero down face camera test";
#endif

	uint32_t _camera_initial_timestamp = 0;
	uint32_t _camera_prev_timestamp = 0;

	Camera *_camera;
	OpticalFlowOpenCV *_optical_flow;
	Mavlink_UDP *_mavlink;
	BMI160 *_bmi;

	struct timespec _gyro_last_timespec;

	void signal_handlers_setup();
	void loop();
};

static bool _should_run;

static void exit_signal_handler(int signum)
{
    _should_run = false;
}

void Mainloop::signal_handlers_setup(void)
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

void Mainloop::loop()
{
	Pollable *pollables[] = { _camera, _bmi, _mavlink };
	const uint8_t len = sizeof(pollables) / sizeof(Pollable *);
	struct pollfd desc[len];

	signal_handlers_setup();
	_should_run = true;

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN;
		desc[i].revents = 0;
	}

	while (_should_run) {
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), -1);
		if (ret < 1) {
			continue;
		}

		for (int i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			for (uint8_t j = 0; j < len; j++) {
				if (desc[i].fd == pollables[j]->_fd) {
					if (desc[i].revents & (POLLIN | POLLPRI)) {
						pollables[j]->handle_read();
					}
					if (desc[i].revents & POLLOUT) {
						pollables[j]->handle_canwrite();
					}
					break;
				}
			}
		}
	}
}

static void _camera_callback(const void *img, size_t len, const struct timeval *timestamp, void *data)
{
	Mainloop *mainloop = (Mainloop *)data;
	mainloop->camera_callback(img, len, timestamp);
}

void Mainloop::camera_callback(const void *img, size_t len, const struct timeval *timestamp)
{
	int dt_us = 0;
	float flow_x_ang = 0, flow_y_ang = 0;

	Mat frame_gray = Mat(_camera->height, _camera->width, CV_8UC1);
	frame_gray.data = (uchar*)img;

	// crop the image (optical flow assumes narrow field of view)
	cv::Rect crop(_camera->width / 2 - _optical_flow->getImageWidth() / 2,
			_camera->height / 2 - _optical_flow->getImageHeight() / 2,
			_optical_flow->getImageWidth(), _optical_flow->getImageHeight());
	cv::Mat cropped_image = frame_gray(crop);

#if DEBUG_LEVEL
	imshow(_window_name, frame_gray);
#endif

	uint32_t img_time_us = timestamp->tv_usec + timestamp->tv_sec * USEC_PER_SEC;
	float fps = 0;

	if (_camera_initial_timestamp) {
		img_time_us -= _camera_initial_timestamp;
		fps = 1.0f / ((float)(img_time_us - _camera_prev_timestamp) / USEC_PER_SEC);
	} else {
		_camera_initial_timestamp = img_time_us;
		img_time_us = 0;
	}

	int flow_quality = _optical_flow->calcFlow(cropped_image.data, img_time_us, dt_us, flow_x_ang, flow_y_ang);

#if DEBUG_LEVEL
	DEBUG("Optical flow quality=%i x=%f y=%f timestamp sec=%lu usec=%lu fps=%f", flow_quality, flow_x_ang, flow_y_ang,
		img_time_us / USEC_PER_SEC, img_time_us % USEC_PER_SEC, fps);
#endif

	_camera_prev_timestamp = img_time_us;

	Point3_<double> gyro_data;
	struct timespec gyro_timespec;
	_bmi->gyro_integrated_get(&gyro_data, &gyro_timespec);

	// check liveness of BMI160
	if (_gyro_last_timespec.tv_sec == gyro_timespec.tv_sec
			&& _gyro_last_timespec.tv_nsec == gyro_timespec.tv_nsec) {
		DEBUG("No new gyroscope data available, sensor is calibrating?");
		return;
	}
	_gyro_last_timespec = gyro_timespec;

#if DEBUG_LEVEL
	DEBUG("Gyro data(%f %f %f)", gyro_data.x, gyro_data.y, gyro_data.z);
#endif

	// check if flow is ready/integrated -> flow output rate
	if (flow_quality < 0) {
		return;
	}

	mavlink_optical_flow_rad_t msg;
	msg.time_usec = timestamp->tv_usec + timestamp->tv_sec * USEC_PER_SEC;
	msg.integration_time_us = dt_us;
	msg.integrated_x = flow_x_ang;
	msg.integrated_y = flow_y_ang;
	msg.integrated_xgyro = gyro_data.x;
	msg.integrated_ygyro = gyro_data.y;
	msg.integrated_zgyro = gyro_data.z;
	msg.time_delta_distance_us = 0;
	msg.distance = -1.0;
	msg.temperature = 0;
	msg.sensor_id = 0;
	msg.quality = flow_quality;

	_mavlink->optical_flow_rad_msg_write(&msg);
}

int Mainloop::run(const char *camera_device, int camera_id,
		uint32_t camera_width, uint32_t camera_height, uint32_t crop_width,
		uint32_t crop_height, unsigned long mavlink_udp_port,
		int flow_output_rate, float focal_length_x, float focal_length_y,
		bool calibrate_bmi, const char *parameters_folder)
{
	int ret;

	_camera = new Camera(camera_device);
	if (!_camera) {
		ERROR("No memory to instantiate Camera");
		return -1;
	}
	ret = _camera->init(camera_id, camera_width, camera_height,
			DEFAULT_PIXEL_FORMAT);
	if (ret) {
		ERROR("Unable to initialize camera");
		goto camera_init_error;
	}

	_mavlink = new Mavlink_UDP();
	if (!_mavlink) {
		ERROR("No memory to instantiate Mavlink_UDP");
		goto mavlink_memory_error;
	}
	ret = _mavlink->init("127.0.0.1", mavlink_udp_port);
	if (ret) {
		ERROR("Unable to initialize mavlink");
		goto mavlink_init_error;
	}

	// TODO: load parameters from yaml file
	_optical_flow = new OpticalFlowOpenCV(focal_length_x, focal_length_y, flow_output_rate, crop_width,
			crop_height);
	if (!_optical_flow) {
		ERROR("No memory to instantiate OpticalFlowOpenCV");
		goto optical_memory_error;
	}
	_camera->callback_set(_camera_callback, this);

	_bmi = new BMI160("/dev/spidev3.0", parameters_folder);
	if (!_bmi) {
		ERROR("No memory to allocate BMI160");
		goto bmi_memory;
	}
	if (_bmi->init()) {
		ERROR("BMI160 init error");
		goto bmi_error;
	}
	if (calibrate_bmi) {
		_bmi->calibrate();
	}
	if (_bmi->start()) {
		ERROR("BMI160 start error");
		goto bmi_error;
	}

#if DEBUG_LEVEL
	namedWindow(_window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	loop();

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	_bmi->stop();
	delete _bmi;
	delete _optical_flow;
	delete _mavlink;
	_camera->shutdown();
	delete _camera;

	return 0;

bmi_error:
	delete _bmi;
bmi_memory:
	delete _optical_flow;
optical_memory_error:
mavlink_init_error:
	delete _mavlink;
mavlink_memory_error:
	_camera->shutdown();
camera_init_error:
	delete _camera;
	return -1;
}

static int safe_atoul(const char *s, unsigned long *ret)
{
	char *x = NULL;
	unsigned long l;

	errno = 0;
	l = strtoul(s, &x, 0);

	if (!x || x == s || *x || errno)
		return errno ? -errno : -EINVAL;

	*ret = l;

	return 0;
}

static int safe_atoi(const char *s, int *ret)
{
	char *x = NULL;
	long l;

	errno = 0;
	l = strtol(s, &x, 0);

	if (!x || x == s || *x || errno)
		return errno > 0 ? -errno : -EINVAL;

	if ((long) (int) l != l)
		return -ERANGE;

	*ret = (int) l;
	return 0;
}

static int safe_atof(const char *s, float *ret)
{
	char *x = NULL;
	float l;

	errno = 0;
	l = strtof(s, &x);

	if (!x || x == s || *x || errno)
		return errno > 0 ? -errno : -EINVAL;

	*ret = (float) l;
	return 0;
}

static void help()
{
	printf("%s [OPTIONS...]\n\n"
			"  -c --camera_device       Camera filepath\n"
			"                           Default %s\n"
			"  -i --camera_id           Camera id\n"
			"                           Default %u\n"
			"  -r --camera_resolution   Resolution of the video streaming from camera\n"
			"                           Default %ux%u\n"
			"  -x --crop_resolution     Resolution of the video that will be used to calculate optical flow\n"
			"                           Default %ux%u\n"
			"  -o --flow_output_rate    Output rate of the optical flow\n"
			"                           Default %u\n"
			"  -p --mavlink_udp_port    MAVLink UDP port where it will listen and send messages\n"
			"                           Default %u\n"
			"  -f --focal_length        Set camera focal lenght in pixels\n"
			"                           Default %fx%f\n"
			"  -b --bmi160_calibrate    Calibrate BMI160\n"
			"  -a --parameters_folder   Default parameters folder\n"
			"                           Default %s\n"
			,
			program_invocation_short_name,
			DEFAULT_DEVICE_FILE,
			DEFAULT_DEVICE_ID,
			DEFAULT_IMG_WIDTH,
			DEFAULT_IMG_HEIGHT,
			DEFAULT_IMG_CROP_WIDTH,
			DEFAULT_IMG_CROP_HEIGHT,
			DEFAULT_OUTPUT_RATE,
			MAVLINK_UDP_PORT,
			DEFAULT_FOCAL_LENGTH_X,
			DEFAULT_FOCAL_LENGTH_Y,
			DEFAULT_PARAMETERS_FOLDER);
}

static int x_y_split(char *arg, unsigned long *x, unsigned long *y)
{
	char *divider = strchrnul(optarg, 'x');
	const char *x_str, *y_str;

	if (!divider) {
		return -1;
	}

	x_str = optarg;
	y_str = divider + 1;
	*divider = '\0';

	if (safe_atoul(x_str, x)) {
		return -1;
	}
	if (safe_atoul(y_str, y)) {
		return -1;
	}

	return 0;
}

static int x_y_float_split(char *arg, float *x, float *y)
{
	char *divider = strchrnul(optarg, 'x');
	const char *x_str, *y_str;

	if (!divider) {
		return -1;
	}

	x_str = optarg;
	y_str = divider + 1;
	*divider = '\0';

	if (safe_atof(x_str, x)) {
		return -1;
	}
	if (safe_atof(y_str, y)) {
		return -1;
	}

	return 0;
}

int main (int argc, char *argv[])
{
	Mainloop mainloop;
	int c;
	const struct option options[] = {
			{ "camera_device",			required_argument,	NULL,	'c' },
			{ "camera_id",				required_argument,	NULL,	'i' },
			{ "camera_width",			required_argument,	NULL,	'w' },
			{ "camera_height",			required_argument,	NULL,	'h' },
			{ "crop_width",				required_argument,	NULL,	'x' },
			{ "crop_height",			required_argument,	NULL,	'y' },
			{ "flow_output_rate",		required_argument,	NULL,	'o' },
			{ "mavlink_udp_port",		required_argument,	NULL,	'p' },
			{ "focal_length",			required_argument,	NULL,	'f' },
			{ "bmi160_calibrate",		no_argument,		NULL,	'b' },
			{ "parameters_folder",		required_argument,  NULL,   'a' },
			{ }
	};
	const char *camera_device = DEFAULT_DEVICE_FILE;
	unsigned long camera_id = DEFAULT_DEVICE_ID;
	unsigned long camera_width = DEFAULT_IMG_WIDTH;
	unsigned long camera_height = DEFAULT_IMG_HEIGHT;
	unsigned long crop_width = DEFAULT_IMG_CROP_WIDTH;
	unsigned long crop_height = DEFAULT_IMG_CROP_HEIGHT;
	unsigned long mavlink_udp_port = MAVLINK_UDP_PORT;
	int flow_output_rate = DEFAULT_FLOW_OUTPUT_RATE;
	float focal_length_x = DEFAULT_FOCAL_LENGTH_X;
	float focal_length_y = DEFAULT_FOCAL_LENGTH_Y;
	bool bmi160_calibrate = false;
	const char *parameter_folder = DEFAULT_PARAMETERS_FOLDER;

	while ((c = getopt_long(argc, argv, "?c:i:r:x:o:p:f:ba:", options, NULL)) >= 0) {
		switch (c) {
		case '?':
			help();
			return 0;
		case 'c':
			camera_device = optarg;
			break;
		case 'i':
			if (safe_atoul(optarg, &camera_id) < 0) {
				ERROR("Invalid argument for camera_id = %s", optarg);
				help();
				return -EINVAL;
			}
			break;
		case 'r':
			x_y_split(optarg, &camera_width, &camera_height);
			break;
		case 'x':
			x_y_split(optarg, &crop_width, &crop_height);
			break;
		case 'o':
			if (safe_atoi(optarg, &flow_output_rate) < 0) {
				ERROR("Invalid argument for flow_output_rate = %s", optarg);
				help();
				return -EINVAL;
			}
			break;
		case 'p':
			if (safe_atoul(optarg, &mavlink_udp_port) < 0) {
				ERROR("Invalid argument for mavlink_udp_port = %s", optarg);
				help();
				return -EINVAL;
			}
			break;
		case 'f':
			x_y_float_split(optarg, &focal_length_x, &focal_length_y);
			break;
		case 'b':
			bmi160_calibrate = true;
			break;
		case 'a':
			parameter_folder = optarg;
			break;
		default:
			help();
			return -EINVAL;
		}
	}

	printf("Parameters:\n\tcamera_device=%s\n\tcamera_id=%u\n\tcamera_width=%u\n", camera_device, camera_id, camera_width);
	printf("\tcamera_height=%u\n\tcrop_width=%u\n\tcrop_height=%u\n", camera_height, crop_width, crop_height);
	printf("\tflow_output_rate=%i\n\tmavlink_udp_port=%u\n", flow_output_rate, mavlink_udp_port);
	printf("\tfocal_length_x=%f\n\tfocal_length_y=%f\n\tbmi160_calibrate=%u\n", focal_length_x, focal_length_y, bmi160_calibrate);
	printf("\tparameter_folder=%s\n", parameter_folder);

	return mainloop.run(camera_device, camera_id, camera_width, camera_height,
			crop_width, crop_height, mavlink_udp_port, flow_output_rate,
			focal_length_x, focal_length_y, bmi160_calibrate, parameter_folder);
}
