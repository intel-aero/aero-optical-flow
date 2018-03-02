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
#define GAIN_P_GAIN 50.0f
#define GAIN_I_GAIN 0.5f
#define GAIN_D_GAIN 0.5f
#define EXPOSURE_CHANGE_THRESHOLD 30.0
#define EXPOSURE_ABOSULUTE_MAX_VALUE 1727
#define GAIN_CHANGE_THRESHOLD 15.0
#define GAIN_ABOSULUTE_MAX_VALUE 127
#define CAMERA_FPS_MIN 70

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
	Pollable *pollables[] = { _mavlink };
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
	float msv_error_d = msv_error - _msv_error_old;
	_msv_error_int += msv_error;

	float exposure = _camera->exposure_get();
	float gain = _camera->gain_get();

	exposure += (EXPOSURE_P_GAIN * msv_error) + (EXPOSURE_I_GAIN * _msv_error_int) + (EXPOSURE_D_GAIN * msv_error_d);

	// adjust the gain if exposure is saturated
	if (gain > 1.0f || (exposure > EXPOSURE_ABOSULUTE_MAX_VALUE-1 && _camera->exposure_get() > EXPOSURE_ABOSULUTE_MAX_VALUE-1)) {
		// calculate new gain value based on MSV
		gain += (GAIN_P_GAIN*msv_error) + (GAIN_I_GAIN*_msv_error_int) + (GAIN_D_GAIN*msv_error_d);

		if (gain > GAIN_ABOSULUTE_MAX_VALUE) {
			gain = GAIN_ABOSULUTE_MAX_VALUE;
		} else if (gain < 1.0f) {
			gain = 1.0f;
		}

		/* set new gain value if bigger than threshold */
		if (fabs(gain - _camera->gain_get()) > GAIN_CHANGE_THRESHOLD || (gain < 2.0f && _camera->gain_get() > 1.0f) ||
				(gain > GAIN_ABOSULUTE_MAX_VALUE-1 && _camera->gain_get() < GAIN_ABOSULUTE_MAX_VALUE)) {
#if DEBUG_LEVEL
			DEBUG("gain set %u", (uint16_t)gain);
#endif
			_camera->gain_set(gain);
		}

	} else { // adjust exposure

		if (exposure > EXPOSURE_ABOSULUTE_MAX_VALUE) {
			exposure = EXPOSURE_ABOSULUTE_MAX_VALUE;
		} else if (exposure < 1.0f) {
			exposure = 1.0f;
		}

		/* set new exposure value if bigger than threshold */
		if (fabs(exposure - _camera->exposure_get()) > EXPOSURE_CHANGE_THRESHOLD || (exposure < 2.0f && _camera->exposure_get() > 1.0f) ||
				(exposure > EXPOSURE_ABOSULUTE_MAX_VALUE-1 && _camera->exposure_get() < EXPOSURE_ABOSULUTE_MAX_VALUE)) {
#if DEBUG_LEVEL
			DEBUG("exposure set %u", (uint16_t)exposure);
#endif
			_camera->exposure_set(exposure);
		}

	}

	_msv_error_old = msv_error;

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

	float fps = 0;

	if (_camera_initial_timestamp) {
		img_time_us -= _camera_initial_timestamp;
		fps = 1.0f / ((float)(img_time_us - _camera_prev_timestamp) / USEC_PER_SEC);
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

	Point3_<double> gyro_data = _gyro_integrated;
	_gyro_integrated.x = _gyro_integrated.y = _gyro_integrated.z = 0;

	// check liveness of gyro data
	if (_gyro_last_usec_timestamp == _gyro_prev_timestamp) {
		DEBUG("No new gyroscope data available");
		pthread_mutex_unlock(&_mainloop_lock);
		return;
	}
	_gyro_last_usec_timestamp = _gyro_prev_timestamp;

	if (fps < CAMERA_FPS_MIN) {
		ERROR("FPS below minimum, actual=%u minimum=%u", (unsigned)fps, CAMERA_FPS_MIN);
	}
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
	msg.integrated_x = flow_x_ang;
	msg.integrated_y = flow_y_ang;
	msg.integrated_xgyro = -gyro_data.y; // switch to match pixel directions
	msg.integrated_ygyro = gyro_data.x; // switch to match pixel directions
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
	mainloop->highres_imu_msg_callback(msg);
}

void Mainloop::highres_imu_msg_callback(const mavlink_highres_imu_t *msg)
{
	// Integrate
	const double dt = (double)(msg->time_usec - _gyro_prev_timestamp) / (double)USEC_PER_SEC;

	// protect against invalid gyro and timestamp data -> huge random values
	const double max_dt = 0.05; // at least 20Hz
	const double max_gyro = 20.0; // big enough value
	if (_gyro_prev_timestamp && dt < max_dt && abs(msg->xgyro) < max_gyro
		&& abs(msg->ygyro) < max_gyro && abs(msg->zgyro) < max_gyro) {
		_gyro_integrated.x += (msg->xgyro * dt);
		_gyro_integrated.y += (msg->ygyro * dt);
		_gyro_integrated.z += (msg->zgyro * dt);
#if DEBUG_LEVEL
	} else {
		DEBUG("Invalid gyro/timestamp data: gyro = (%f, %f, %f) dt = %f", msg->xgyro, msg->ygyro, msg->zgyro, dt);
#endif
	}
	_gyro_prev_timestamp = msg->time_usec;

	if (!_offset_timestamp_usec)
		_offset_timestamp_usec = msg->time_usec;
}

int Mainloop::init(const char *camera_device, int camera_id,
		uint32_t camera_width, uint32_t camera_height, uint32_t crop_width,
		uint32_t crop_height, const char *mavlink_tcp_ip, unsigned long mavlink_tcp_port,
		int flow_output_rate, float focal_length_x, float focal_length_y)
{
	_camera = new Camera(camera_device);
	if (!_camera) {
		ERROR("No memory to allocate Camera");
		return -1;
	}
	_mavlink = new Mavlink_TCP();
	if (!_mavlink) {
		ERROR("No memory to allocate Mavlink_TCP");
		goto mavlink_memory_error;
	}
	// TODO: load parameters from yaml file
	_optical_flow = new OpticalFlowOpenCV(focal_length_x, focal_length_y, flow_output_rate, crop_width,
			crop_height);
	if (!_optical_flow) {
		ERROR("No memory to instantiate OpticalFlowOpenCV");
		goto optical_memory_error;
	}

	if (_camera->init(camera_id, camera_width, camera_height, DEFAULT_PIXEL_FORMAT)) {
		ERROR("Unable to initialize camera");
		goto camera_init_error;
	}
	if (_mavlink->init(mavlink_tcp_ip, mavlink_tcp_port)) {
		ERROR("Unable to initialize Mavlink_TCP");
		goto mavlink_init_error;
	}

	return 0;

mavlink_init_error:
	_camera->shutdown();
camera_init_error:
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

	delete _optical_flow;
	delete _mavlink;
	delete _camera;

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

#if DEBUG_LEVEL
	namedWindow(_window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	_loop();

#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	_camera->stop();

	_camera->callback_set(NULL, NULL);

	return 0;


camera_start_error:
	_camera->callback_set(NULL, NULL);
	return -1;
}
