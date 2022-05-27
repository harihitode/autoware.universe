# EKF Hornet Driver

## Overview

This version of EKF Hornet Driver is derived from [Extend Kalman Filter Localizer](../ekf_localizer/).

See details for original README.md

The EKF Hornet Driver realizes Extend Kalman Filter calculation by TCP/IP communication with EKF Accelerator.

## Requirements

### How to use this package with Autoware

You can use `pose_estimator_hornet` by specifying `use_pose_estimator_hornet:=true` at launch

Example

``` shell
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=<map_dir_path> perception:=false planning:=false control:=false vehicle_simulation:=true vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit vehicle_simulation:=true use_pose_estimator_hornet:=true
```

