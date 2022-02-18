cmake_minimum_required(VERSION 3.5)
project(ekf_localizer_ocl)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(ekf_localizer_ocl
  src/ekf_localizer_ocl_node.cpp
  src/ekf_localizer_ocl.cpp
)
ament_target_dependencies(ekf_localizer_ocl kalman_filter)
target_link_libraries(ekf_localizer_ocl /usr/lib/x86_64-linux-gnu/libOpenCL.so.1)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
