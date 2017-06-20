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

#include "main.h"

#include <signal.h>
#include <stdio.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <flow_opencv.hpp>

#include "config.h"
#include "camera.h"
#include "log.h"
#include "mainloop.h"
#include "mavlink_tcp.h"
#include "util.h"

#define DEFAULT_PIXEL_FORMAT V4L2_PIX_FMT_YUV420
#define CAMERA_MSEC_TIMEOUT 100

#define EXPOSURE_MASK_SIZE 128
#define EXPOSURE_MSV_TARGET 5.0f
#define EXPOSURE_P_GAIN 100.0f
#define EXPOSURE_I_GAIN 0.5f
#define EXPOSURE_D_GAIN 0.5f
#define EXPOSURE_CHANGE_THRESHOLD 30.0
#define EXPOSURE_ABOSULUTE_MAX_VALUE 1727

using namespace cv;

static volatile bool _should_run;

static Camera *_camera;
static OpticalFlowOpenCV *_optical_flow;
static Mavlink_TCP *_mavlink;

static uint64_t _camera_initial_timestamp = 0;
static uint64_t _camera_prev_timestamp = 0;
static uint64_t _offset_timestamp_usec = 0;
static uint64_t _next_exposure_update_timestap = 0;

static float _exposure_msv_error_int = 0.0f;
static float _exposure_msv_error_old = 0.0f;

#if DEBUG_LEVEL
const char *_window_name = "Aero down face camera test";
#endif

static uint32_t camera_restart_count = 0;

static void exit_signal_handler(UNUSED int signum)
{
    _should_run = false;
}

static void _signal_handlers_setup(void)
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

static void exposure_update(Mat frame, uint64_t timestamp_us)
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

static void camera_callback(const void *img, UNUSED size_t len, const struct timeval *timestamp, UNUSED void *data)
{
	int dt_us = 0;
	float flow_x_ang = 0, flow_y_ang = 0;

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
	exposure_update(cropped_image, img_time_us);

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
		return;
	}

#if DEBUG_LEVEL
	DEBUG("Optical flow quality=%i x=%f y=%f timestamp sec=%lu usec=%lu fps=%f", flow_quality, flow_y_ang, -flow_x_ang,
		img_time_us / USEC_PER_SEC, img_time_us % USEC_PER_SEC, fps);
#endif

	/*if (!_offset_timestamp_usec) {
		DEBUG("Waiting for timestamp from vehicle");
		return;
	}*/

	mavlink_optical_flow_rad_t msg;
	msg.time_usec = _offset_timestamp_usec + img_time_us;
	msg.integration_time_us = dt_us;
	msg.integrated_x = flow_y_ang; //switch to match correct directions
	msg.integrated_y = -flow_x_ang; //switch to match correct directions
	msg.integrated_xgyro = 0;
	msg.integrated_ygyro = 0;
	msg.integrated_zgyro = 0;
	msg.time_delta_distance_us = 0;
	msg.distance = -1.0;
	msg.temperature = 0;
	msg.sensor_id = 0;
	msg.quality = flow_quality;

	_mavlink->optical_flow_rad_msg_write(&msg);
}

static void _highres_imu_msg_callback(const mavlink_highres_imu_t *msg, UNUSED void *data)
{
	if (!_offset_timestamp_usec)
		_offset_timestamp_usec = msg->time_usec;
}

static int init()
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

	if (_camera->init(camera_dev_id, camera_width, camera_height, DEFAULT_PIXEL_FORMAT)) {
		ERROR("Unable to initialize camera");
		goto camera_init_error;
	}
	if (_mavlink->init("127.0.0.1", mavlink_tcp_port, SYSTEM_ID_MAIN)) {
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

static void shutdown()
{
	_camera->shutdown();

	delete _optical_flow;
	delete _mavlink;
	delete _camera;

	_optical_flow = NULL;
	_mavlink = NULL;
	_camera = NULL;
}

static int start()
{
	_camera->callback_set(camera_callback, NULL);
	_mavlink->highres_imu_msg_subscribe(_highres_imu_msg_callback, NULL);

	if (_camera->start()) {
		ERROR("Unable to start camera streaming");
		goto camera_start_error;
	}

#if DEBUG_LEVEL
	namedWindow(_window_name, WINDOW_AUTOSIZE);
	startWindowThread();
#endif

	return 0;

camera_start_error:
	_camera->callback_set(NULL, NULL);
	return -1;
}

static int stop()
{
#if DEBUG_LEVEL
	destroyAllWindows();
#endif

	_camera->stop();

	_camera->callback_set(NULL, NULL);

	return 0;
}

static void timeout_callback(UNUSED void *data)
{
	camera_restart_count++;
	DEBUG("Camera timeout, restarting... camera_restart_count=%u", camera_restart_count);
	_camera->restart();
}

int main(int argc, char *argv[])
{
	Mainloop mainloop;
	Pollable *pollables[1];

	int ret = parse_args(argc, argv);
	if (ret) {
		return ret;
	}

	ret = init();
	if (ret) {
		return -1;
	}

	ret = start();
	if (ret) {
		goto start_error;
	}

	_signal_handlers_setup();

	mainloop.loop_timeout_callback_set(timeout_callback, NULL);

	pollables[0] = _camera;
	//pollables[1] = _mavlink;
	mainloop.loop(pollables, sizeof(pollables) / sizeof(Pollable *), &_should_run, CAMERA_MSEC_TIMEOUT);

	stop();
start_error:
	shutdown();

	return ret;
}
