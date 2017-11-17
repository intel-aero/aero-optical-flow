[![Build Status](https://travis-ci.org/intel-aero/aero-optical-flow.svg?branch=master)](https://travis-ci.org/intel-aero/aero-optical-flow)

# How to run it on Intel (R) Aero RTF Drone

- install Aero image v1.6 or newer (https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup#flashing)
- update the PX4 firmware to the one in image `aerofc-update.sh /etc/aerofc/px4/nuttx-aerofc-v1-default.px4`
- install a lidar (https://docs.px4.io/en/flight_controller/intel_aero.html#connecting-a-lidar-lite-range-finder), it was only tested with Garmin Lidar‑Lite V3 but we plan to test with some other cheap options.
- in QGroundControl change this parameters:
	- EKF2_AID_MASK = 3 (use GPS + use optical flow)
	- EKF2_HGT_MODE = 2 (range sensor)
- inside of a Aero terminal run: `systemctl start aero-optical-flow.service` change `start` to `enable` if you want it to run at every boot
- change the flight mode to position and enjoy optical-flow

The down-faced camera in Aero RTF was chosen having in mind VIO (visual-inertial odometry) applications not optical-flow so it have a large field of view camera, because of that it will not be able to hold optical-flow positioning in floors with low texture. You can hand held the Aero RTF and check the quality of the optical-flow in the desire floor before actually fly by looking to `QGroundControl>Widgets>MAVLink Inspector>OPTICAL_FLOW_RAD>quality` if you are getting values higher than 100 it should be enough.

# Build instructions

## Requirements
 - opencv

## Build Steps

```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```
