# DISCONTINUATION OF PROJECT #
This project will no longer be maintained by Intel.
Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.
Intel no longer accepts patches to this project.
[![Build Status](https://travis-ci.org/intel-aero/aero-optical-flow.svg?branch=master)](https://travis-ci.org/intel-aero/aero-optical-flow)

# How to run it on Intel (R) Aero RTF Drone

- install Aero image v1.6 or newer (https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup#flashing)
- build and install the latest stable PX4(the PX4 firmware in the Aero image do not have the LeddarOne driver)
- install a lidar, we recommend the LeddarOne(https://docs.px4.io/en/flight_controller/intel_aero.html) that can be connected to the telemetry port.
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
