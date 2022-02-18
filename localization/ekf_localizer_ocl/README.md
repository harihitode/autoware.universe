# Extended Kalman Filter Localizer (OpenCL)

## Overview

This version of EKF Localizer is derived from [Extend Kalman Filter Localizer](../ekf_localizer/).

See details for original README.md

This ekf localizer uses an opencl kernel instead of eigen for the function `predictWithDelay`.

---

* Autoware (.iv/architecture proposal v0.9.1)
* OpenCL 1.2

## Requirements

### Install OpenCL Runtimes for Intel CPU.

1. Download opencl-drivers from https://software.intel.com/content/www/us/en/develop/articles/opencl-drivers.html#cpu-section

  * version: 18.1
  * file: l_opencl_p_18.1.0.015.tgz

2. Unzip the downloaded file and run ```sudo ./install_GUI.sh``` in the directory.
3. Install runtimes in the GUI installer.
4. Check the ```clinfo``` 's output. You can see the `number of platforms` 1 or more if the installation has been completed.

### Install OpenCL SDK for Intel CPU.

1. Download opencl-sdk from https://software.seek.intel.com/intel-opencl?os=linux

  * version: 2020.3.494
  * file: intel_sdk_for_opencl_applications_2020.3.494.tar.gz

2. Unzip the downloaded file and run ```sudo ./install.sh``` in the directory.
3. You can install from GUI installer.

### [Additional Parameter] Floating Point Configurations

| name       | type | description                                                                | default |
|:-----------|:-----|:---------------------------------------------------------------------------|:--------|
| use_double | bool | true to use double precesion, false to use single precesion floating point | true    |

### How to use this package with Autoware

You can use `ekf_localizer_ocl` changing the launch file, `localization_launch/launch/pose_twist_fusion_filter/pose_twist_fusion_filter.launch`.

Example

```
<include file="$(find ekf_localizer_ocl)/launch/ekf_localizer_ocl.launch">
```
