cmake_minimum_required(VERSION 3.14)
project(ekf_localizer_ocl)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
 )

ament_auto_add_executable(ekf_localizer_ocl
  src/ekf_localizer_ocl_node.cpp
  src/ekf_localizer_ocl.cpp
)
ament_target_dependencies(ekf_localizer_ocl kalman_filter)
target_link_libraries(ekf_localizer_ocl /usr/lib/x86_64-linux-gnu/libOpenCL.so.1)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
