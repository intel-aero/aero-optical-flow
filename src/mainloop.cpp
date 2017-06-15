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

#include <signal.h>
#include <stdio.h>
#include <poll.h>
#include <pthread.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "log.h"
#include "util.h"

#define DEFAULT_PIXEL_FORMAT V4L2_PIX_FMT_YUV420
#define CAMERA_MSEC_TIMEOUT 100

#define POLL_ERROR_EVENTS (POLLERR | POLLHUP | POLLNVAL)

#define EXPOSURE_MASK_SIZE 128
#define EXPOSURE_MSV_TARGET 5.0f
#define EXPOSURE_P_GAIN 100.0f
#define EXPOSURE_I_GAIN 0.5f
#define EXPOSURE_D_GAIN 0.5f
#define EXPOSURE_CHANGE_THRESHOLD 30.0
#define EXPOSURE_ABOSULUTE_MAX_VALUE 1727

using namespace cv;

static volatile bool _should_run;

static void exit_signal_handler(UNUSED int signum)
{
    _should_run = false;
}

void Mainloop::_signal_handlers_setup(void)
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

static void *_thread_callback(void *data)
{
	Mainloop *mainloop = (Mainloop *)data;
	return mainloop->camera_thread();
}

void *Mainloop::camera_thread()
{
	Pollable *pollables[] = { _camera };
	const uint8_t len = sizeof(pollables) / sizeof(Pollable *);
	struct pollfd desc[len];

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN | POLLPRI | POLL_ERROR_EVENTS;
		desc[i].revents = 0;
	}

	while (_should_run) {
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), CAMERA_MSEC_TIMEOUT);
		if (ret == 0) {
			DEBUG("Camera timeout, restarting...");
			_camera->restart();
			continue;
		}
		if (ret < 1) {
			continue;
		}

		for (unsigned i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			for (uint8_t j = 0; j < len; j++) {
				if (desc[i].fd == pollables[j]->_fd) {
					if (desc[i].revents & (POLLIN | POLLPRI)) {
						pollables[j]->handle_read();
					}
					if (desc[i].revents & POLLOUT) {
						pollables[j]->handle_canwrite();
					}
					if (desc[i].revents & POLL_ERROR_EVENTS) {
						ERROR("Poll error event on camera: %u", desc[i].revents);
					}
					break;
				}
			}
		}
	}

	return NULL;
}

void Mainloop::_loop()
{
	Pollable *pollables[] = { _bmi, _mavlink };
	uint8_t len = sizeof(pollables) / sizeof(Pollable *);
	struct pollfd desc[len];

	_signal_handlers_setup();
	_should_run = true;

	pthread_mutex_init(&_mainloop_lock, NULL);
	pthread_t thread;
	if (pthread_create(&thread, NULL, _thread_callback, this)) {
		ERROR("Unable to create a thread");
		return;
	}

	for (uint8_t i = 0; i < len; i++) {
		desc[i].fd = pollables[i]->_fd;
		desc[i].events = POLLIN | POLLPRI | POLL_ERROR_EVENTS;
		desc[i].revents = 0;
	}

	while (_should_run) {
		int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), -1);
		if (ret < 1) {
			continue;
		}

		pthread_mutex_lock(&_mainloop_lock);

		for (unsigned i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
			for (uint8_t j = 0; j < len; j++) {
				if (desc[i].fd == pollables[j]->_fd) {
					if (desc[i].revents & (POLLIN | POLLPRI)) {
						pollables[j]->handle_read();
					}
					if (desc[i].revents & POLLOUT) {
						pollables[j]->handle_canwrite();
					}
					if (desc[i].revents & POLL_ERROR_EVENTS) {
						ERROR("Poll error event: %u fd: %i", desc[i].revents, desc[i].fd);
					}
					break;
				}
			}
		}

		pthread_mutex_unlock(&_mainloop_lock);
	}

	pthread_join(thread, NULL);
}

static void _camera_callback(const void *img, size_t len, const struct timeval *timestamp, void *data)
{
       Mainloop *mainloop = (Mainloop *)data;
       mainloop->camera_callback(img, len, timestamp);
}

void Mainloop::_exposure_update(Mat frame, uint64_t timestamp_us)
{
	if (timestamp_us < _next_exposure_update_timestap) {
		return;
	}

	cv::Mat mask(frame.rows, frame.cols, CV_8U,cv::Scalar(0));
	mask(cv::Rect(frame.cols / 2 - EXPOSURE_MASK_SIZE / 2,
			frame.rows / 2 - EXPOSURE_MASK_SIZE / 2,
			EXPOSURE_MASK_SIZE, EXPOSURE_MASK_SIZE)) = 255;

	int channels[] = { 0 };
	cv::Mat hist;
	int histSize[] = { 10 };
	float range[] = { 0, 255 };
	const float* ranges[] = { range };

	calcHist(&frame, 1, channels, mask, hist, 1, histSize, ranges, true, false);

	/* calculate Mean Sample Value (MSV) */
	float msv = 0.0f;
	for (int i = 0; i < histSize[0]; i++) {
		msv += (i + 1) *hist.at<float>(i) / 16384.0f;// 128x128 -> 16384
	}

	/* PID-controller */
	float msv_error = EXPOSURE_MSV_TARGET - msv;
	float msv_error_d = msv_error - _exposure_msv_error_old;
	_exposure_msv_error_int += msv_error;

	float exposure = _camera->exposure_get();
	exposure += (EXPOSURE_P_GAIN * msv_error) + (EXPOSURE_I_GAIN * _exposure_msv_error_int) + (EXPOSURE_D_GAIN * msv_error_d);

	if (exposure > EXPOSURE_ABOSULUTE_MAX_VALUE) {
		exposure = EXPOSURE_ABOSULUTE_MAX_VALUE;
	} else if (exposure < 1) {
		exposure = 1;
	}

	_exposure_msv_error_old = msv_error;

	/* set new exposure value if bigger than threshold */
	if (fabs(exposure - _camera->exposure_get()) > EXPOSURE_CHANGE_THRESHOLD) {
#if DEBUG_LEVEL
		DEBUG("exposure set %u", (uint16_t)exposure);
#endif
		_camera->exposure_set(exposure);
#if DEBUG_LEVEL
	} else {
		DEBUG("exposure not set because it did not meet the threshold %u", (uint16_t)exposure);
#endif
	}

	/* update exposure at 5Hz */
	_next_exposure_update_timestap = timestamp_us + (USEC_PER_SEC / 5);
}

/* Callback called from another thread */
void Mainloop::camera_callback(const void *img, UNUSED size_t len, const struct timeval *timestamp)
{
	int dt_us = 0;
	float flow_x_ang = 0, flow_y_ang = 0;

	pthread_mutex_lock(&_mainloop_lock);

	Mat frame_gray = Mat(_camera->height, _camera->width, CV_8UC1);
	frame_gray.data = (uchar*)img;

#if DEBUG_LEVEL
	imshow(_window_name, frame_gray);
#endif

	uint64_t img_time_us = timestamp->tv_usec + timestamp->tv_sec * USEC_PER_SEC;

	// crop the image (optical flow assumes narrow field of view)
	cv::Rect crop(_camera->width / 2 - _optical_flow->getImageWidth() / 2,
			_camera->height / 2 - _optical_flow->getImageHeight() / 2,
			_optical_flow->getImageWidth(), _optical_flow->getImageHeight());
	cv::Mat cropped_image = frame_gray(crop);

	// auto exposure for cropped image
	_exposure_update(cropped_image, img_time_us);

#if DEBUG_LEVEL
	float fps = 0;
#endif

	if (_camera_initial_timestamp) {
		img_time_us -= _camera_initial_timestamp;
#if DEBUG_LEVEL
		fps = 1.0f / ((float)(img_time_us - _camera_prev_timestamp) / USEC_PER_SEC);
#endif
	} else {
		_camera_initial_timestamp = img_time_us;
		img_time_us = 0;
	}

	if (img_time_us > UINT32_MAX) {
		ERROR("img_time_us > UINT32_MAX");
	}

	cv::Mat cropped;
	// Copy the data into new matrix -> cropped_image.data can not be used in calcFlow()...
	cropped_image.copyTo(cropped);
	cropped_image.release();

	int flow_quality = _optical_flow->calcFlow(cropped.data, (uint32_t)img_time_us, dt_us, flow_x_ang, flow_y_ang);

	cropped.release();
	_camera_prev_timestamp = img_time_us;

	// check if flow is ready/integrated -> flow output rate
	if (flow_quality < 0) {
		pthread_mutex_unlock(&_mainloop_lock);
		return;
	}

	Point3_<double> gyro_data;
	struct timespec gyro_timespec = {};
	_bmi->gyro_integrated_get(&gyro_data, &gyro_timespec);

	// check liveness of BMI160
	if (_gyro_last_timespec.tv_sec == gyro_timespec.tv_sec
			&& _gyro_last_timespec.tv_nsec == gyro_timespec.tv_nsec) {
		DEBUG("No new gyroscope data available, sensor is calibrating?");
		pthread_mutex_unlock(&_mainloop_lock);
		return;
	}
	_gyro_last_timespec = gyro_timespec;
	_bmi->gyro_integrated_reset();

#if DEBUG_LEVEL
	DEBUG("Optical flow quality=%i x=%f y=%f timestamp sec=%lu usec=%lu fps=%f", flow_quality, flow_y_ang, -flow_x_ang,
		img_time_us / USEC_PER_SEC, img_time_us % USEC_PER_SEC, fps);
	DEBUG("Gyro data(%f %f %f)", gyro_data.x, gyro_data.y, gyro_data.z);
#endif

	if (!_offset_timestamp_usec) {
		DEBUG("Waiting for timestamp from vehicle");
		pthread_mutex_unlock(&_mainloop_lock);
		return;
	}

	mavlink_optical_flow_rad_t msg;
	msg.time_usec = _offset_timestamp_usec + img_time_us;
	msg.integration_time_us = dt_us;
	msg.integrated_x = flow_y_ang; //switch to match correct directions
	msg.integrated_y = -flow_x_ang; //switch to match correct directions
	msg.integrated_xgyro = gyro_data.x;
	msg.integrated_ygyro = gyro_data.y;
	msg.integrated_zgyro = gyro_data.z;
	msg.time_delta_distance_us = 0;
	msg.distance = -1.0;
	msg.temperature = 0;
	msg.sensor_id = 0;
	msg.quality = flow_quality;

	_mavlink->optical_flow_rad_msg_write(&msg);
	pthread_mutex_unlock(&_mainloop_lock);
}

static void _highres_imu_msg_callback(const mavlink_highres_imu_t *msg, void *data)
{
	Mainloop *mainloop = (Mainloop *)data;
	mainloop->timestamp_vehicle_set(msg->time_usec);
}

void Mainloop::timestamp_vehicle_set(uint64_t time_usec)
{
	if (!_offset_timestamp_usec)
		_offset_timestamp_usec = time_usec;
}

int Mainloop::init(const char *camera_device, int camera_id,
		uint32_t camera_width, uint32_t camera_height, uint32_t crop_width,
		uint32_t crop_height, unsigned long mavlink_udp_port,
		int flow_output_rate, float focal_length_x, float focal_length_y,
		bool calibrate_bmi, const char *parameters_folder)
{
	_camera = new Camera(camera_device);
	if (!_camera) {
		ERROR("No memory to allocate Camera");
		return -1;
	}
	_mavlink = new Mavlink_UDP();
	if (!_mavlink) {
		ERROR("No memory to allocate Mavlink_UDP");
		goto mavlink_memory_error;
	}
	// TODO: load parameters from yaml file
	_optical_flow = new OpticalFlowOpenCV(focal_length_x, focal_length_y, flow_output_rate, crop_width,
			crop_height);
	if (!_optical_flow) {
		ERROR("No memory to instantiate OpticalFlowOpenCV");
		goto optical_memory_error;
	}
	_bmi = new BMI160("/dev/spidev3.0", parameters_folder);
	if (!_bmi) {
		ERROR("No memory to allocate BMI160");
		goto bmi_memory_error;
	}

	if (_camera->init(camera_id, camera_width, camera_height, DEFAULT_PIXEL_FORMAT)) {
		ERROR("Unable to initialize camera");
		goto camera_init_error;
	}
	if (_mavlink->init("127.0.0.1", mavlink_udp_port)) {
		ERROR("Unable to initialize Mavlink_UDP");
		goto mavlink_init_error;
	}

	if (_bmi->init()) {
		ERROR("BMI160 init error");
		goto bmi_init_error;
	}
	if (calibrate_bmi) {
		_bmi->calibrate();
	}

	return 0;

bmi_init_error:
mavlink_init_error:
	_camera->shutdown();
camera_init_error:
	delete _bmi;
bmi_memory_error:
	delete _optical_flow;
optical_memory_error:
	delete _mavlink;
mavlink_memory_error:
	delete _camera;
	return -1;
}

void Mainloop::shutdown()
{
	_camera->shutdown();

	delete _bmi;
	delete _optical_flow;
	delete _mavlink;
	delete _camera;

	_bmi = NULL;
	_optical_flow = NULL;
	_mavlink = NULL;
	_camera = NULL;
}

int Mainloop::run()
{
	_camera->callback_set(_camera_callback, this);
	_mavlink->highres_imu_msg_subscribe(_highres_imu_msg_callback, this);

	if (_camera->start()) {
		ERROR("Unable to start camera streaming");
		goto camera_start_error;
	}
	if (_bmi->start()) {
		ERROR("BMI160 start error");
		goto bmi_start_error;
	}

#if DEBUG_LEVEL
	namedWindow(_window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	_loop();

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	_bmi->stop();
	_camera->stop();

	_camera->callback_set(NULL, NULL);

	return 0;

bmi_start_error:
	_camera->stop();
camera_start_error:
	_camera->callback_set(NULL, NULL);
	return -1;
}
