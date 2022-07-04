# Pointcloud Preprocessor Driver

## Purpose

The Pointcloud Preprocessor Driver preprocesses point clouds by TCP/IP communication with Sensing Accelerator.

This package includes the following filters:

- cropping
- correcting distortion
- removing outlier points

## Inner-workings / Algorithms

Detail description of each filter's algorithm is in the following links to documents of `pointcloud_preprocessor` package.

| Filter Name            | Description                                                                        | Detail                                                           |
| ---------------------- | ---------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| `crop_box_filter`      | remove points within a given box                                                   | [link](../pointcloud_preprocessor/docs/crop-box-filter.md)       |
| `distortion_corrector` | compensate pointcloud distortion caused by ego vehicle's movement during 1 scan    | [link](../pointcloud_preprocessor/docs/distortion-corrector.md)  |
| `outlier_filter`       | remove points caused by hardware problems, rain drops and small insects as a noise | [link](../pointcloud_preprocessor/docs/outlier-filter.md)        |

## Inputs / Outputs

### Input

| Name                              | Type                                            | Description                 |
| --------------------------------- | ----------------------------------------------- | --------------------------- |
| `velodyne_packets`                | `velodyne_msgs/msg/VelodyneScan`                | Velodyne LiDAR scan packets |
| `/vehicle/status/velocity_status` | `autoware_auto_vehicle_msgs/msg/VelocityReport` | vehicle twist               |

### Output

| Name                           | Type                            | Description     |
| ------------------------------ | ------------------------------- | --------------- |
| `/outlier_filtered/pointcloud` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Node Parameters

| Name               | Type   | Default Value | Description                                |
| ------------------ | ------ | ------------- | ------------------------------------------ |
| `min_range`        | double | 0.4           | minimum range to publish                   |
| `max_range`        | double | 130.0         | maximum range to publish                   |
| `scan_phase`       | double | 0.0           | start/end phase for the scan (in degrees)  |
| `max_queue_size`   | int    | 5             | max queue size of input/output topics      |

## Requirements

### How to use this package with Autoware

You can use `pointcloud_preprocessor_driver` by specifying `use_pointcloud_preprocessor_driver:=true` at launch.

Example

``` shell
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=<map_dir_path> perception:=false planning:=false control:=false vehicle_simulation:=true vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit vehicle_simulation:=true use_pointcloud_preprocessor_driver:=true
```

