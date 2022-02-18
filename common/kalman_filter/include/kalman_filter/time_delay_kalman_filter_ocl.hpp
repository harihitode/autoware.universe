/*
 * Copyright 2021 Tier IV inc. All rights reserved.
 *
 * The TimeDelayKalmanFilterOCL class is based
 * TimeDelayKalmanFilter with OpenCL implementation.
 *
 * This class is also licensed under the Apache License, Version 2.0.
 *
 * ORIGINAL LICENSE
 *
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_OCL_HPP_
#define KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_OCL_HPP_

#include "kalman_filter/kalman_filter.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <iostream>
#include <memory>

/**
 * @file time_delay_kalman_filter_ocl.hpp
 * @brief OpenCL optimized kalman filter with delayed measurement class, originally developped by Takamasa Horibe in time_delay_kalman_filter
 * @author TierIV, inc.
 * @date 2021.07.01
 */

#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#define CL_TARGET_OPENCL_VERSION 120

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl2.hpp>
#endif

#define MAX_PLATFORMS (10)
#define MAX_DEVICES (10)
#define MAX_SOURCE_SIZE (100000)
#define MAX_THREAD_NUM (512)
#define DIM_X (6)

#define OCL_CREATE_BUFFER_CHECK(ret_mem, context, flags, size, host_ptr, errcode_ret) \
  ret_mem = clCreateBuffer(context, flags, size, host_ptr, &errcode_ret); \
  if (errcode_ret != CL_SUCCESS) {                                      \
    std::cerr << "clCreateBuffer failed. error code: " << errcode_ret << std::endl; \
    return -1;                                                          \
  }
#define OCL_WRITE_BUFFER_CHECK(command_queue, buffer, blocking_write, offset, cb, ptr, num_events_in_wait_list, event_wait_list, event) \
  {                                                                     \
    cl_int errcode_ret = clEnqueueWriteBuffer(command_queue, buffer, blocking_write, offset, cb, ptr, num_events_in_wait_list, event_wait_list, event); \
    if (errcode_ret != CL_SUCCESS) {                                    \
      std::cerr << "clEnqueueWriteBuffer failed. error code: " << errcode_ret << std::endl; \
      return -1;                                                        \
    }                                                                   \
  }
#define OCL_SET_KERNEL_ARG_CHECK(kernel, arg_index, arg_size, arg_value) \
  {                                                                     \
    cl_int errcode_ret = clSetKernelArg(kernel, arg_index, arg_size, arg_value); \
    if (errcode_ret != CL_SUCCESS) {                                    \
      std::cerr << "clSetKernelArg failed. error code: " << errcode_ret << std::endl; \
      return -1;                                                        \
    }                                                                   \
  }
#define OCL_READ_BUFFER_CHECK(command_queue, buffer, blocking_read, offset, cb, ptr, num_events_in_wait_list, event_wait_list, event) \
  {                                                                     \
    cl_int errcode_ret = clEnqueueReadBuffer(command_queue, buffer, blocking_read, offset, cb, ptr, num_events_in_wait_list, event_wait_list, event); \
    if (errcode_ret != CL_SUCCESS) {                                    \
      std::cerr << "clEnqueueReadBuffer failed. error code: " << errcode_ret << std::endl; \
      return -1;                                                        \
    }                                                                   \
  }
#define OCL_RELEASE_MEMORY_CHECK(mem)           \
  if (mem != NULL) {                            \
    clReleaseMemObject(mem);                    \
    mem = NULL;                                 \
  }
#define OCL_RELEASE_KERNEL_CHECK(kernel)        \
  if (kernel != NULL) {                         \
    clReleaseKernel(kernel);                    \
    kernel = NULL;                              \
  }
#define OCL_RELEASE_QUEUE_CHECK(queue)          \
  if (queue != NULL) {                          \
    clReleaseCommandQueue(queue);               \
    queue = NULL;                               \
  }
#define OCL_RELEASE_PROGRAM_CHECK(program)      \
  if (program != NULL) {                        \
    clReleaseProgram(program);                  \
    program = NULL;                             \
  }
#define OCL_RELEASE_CONTEXT_CHECK(context)      \
  if (context != NULL) {                        \
    clReleaseContext(context);                  \
    context = NULL;                             \
  }

class TimeDelayKalmanFilterOCL : public KalmanFilter
{
public:
  /**
   * @brief constructor. initialize OpenCL.
   */
  TimeDelayKalmanFilterOCL();

  /**
   * @brief destructor. finalize OpenCL.
   */
  ~TimeDelayKalmanFilterOCL();

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P0 initial covariance of estimated state
   * @param max_delay_step Maximum number of delay steps, which determines the dimension of the extended kalman filter
   * @param use_double flag to use double type in OpenCL kernel code
   */
  void init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P, const int max_delay_step, bool use_double);

  /**
   * @brief finalization of kalman filter
   */
  void finalize();

  /**
   * @brief get latest time estimated state
   * @param x latest time estimated state
   */
  void getLatestX(Eigen::MatrixXd & x);

  /**
   * @brief get latest time estimation covariance
   * @param P latest time estimation covariance
   */
  void getLatestP(Eigen::MatrixXd & P);

  /**
   * @brief calculate kalman filter covariance by precision model with time delay. This is mainly for EKF of nonlinear
   * process model.
   * @param x_next predicted state by prediction model
   * @param A coefficient matrix of x for process model
   * @param Q covariance matrix for process model
   */
  bool predictWithDelay(
                        const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q);

  /**
   * @brief calculate kalman filter covariance by measurement model with time delay. This is mainly for EKF of nonlinear
   * process model.
   * @param y measured values
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @param delay_step measurement delay
   */
  bool updateWithDelay(
                       const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
                       const int delay_step);

private:
  /**
   * @brief initialize OpenCL.
   * @param platform OpenCL platform ID.
   * @param device OpenCL device ID.
   * @return 0 if initialize succeeded, other than 0 if failed.
   */
  int initializeOCL(cl_uint platform, cl_uint device);

  /**
   * @brief create OpenCL memory objects.
   * @return 0 if creation succeeded, other than 0 if failed.
   */
  template <typename Type>
  int createOCLMemoryObjects();

  /**
   * @brief copy host data to OpenCL memory objects.
   * @return 0 if copy succeeded, other than 0 if failed.
   * @param x_next predicted state by prediction model
   * @param A coefficient matrix of x for process model
   * @param Q covariance matrix for process model
   */
  template <typename Type>
  int copyToOCLMemoryObjects(
                             const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q);

  /**
   * @brief call predictWithDelayCL in kernel code.
   * @return 0 if call succeeded, other than 0 if failed.
   */
  int predictWithDelayCL();

  /**
   * @brief read OpenCL memory objects.
   * @return 0 if read succeeded, other than 0 if failed.
   */
  template <typename Type>
  int readOCLMemoryObjects();

  /**
   * @brief release OpenCL memory objects.
   */
  void releaseOCLMemoryObjects();

  /**
   * @brief finalize OpenCL.
   */
  void finalizeOCL();

  int max_delay_step_;  //!< @brief maximum number of delay steps
  int dim_x_;           //!< @brief dimension of latest state
  int dim_x_ex_;        //!< @brief dimension of extended state with dime delay

  cl_platform_id platform_id_[MAX_PLATFORMS];  //!< @brief OpenCL platform ID
  cl_device_id device_id_[MAX_DEVICES];        //!< @brief OpenCL device ID
  cl_command_queue Queue_;                     //!< @brief OpenCL queue
  cl_kernel k_predict_with_delay_;             //!< @brief OpenCL kernel
  cl_context context_;                         //!< @brief OpenCL context
  cl_program program_;                         //!< @brief OpenCL program
  cl_mem d_x_next_, d_A_, d_A_transpose_;      //!< @brief OpenCL memory objects
  cl_mem d_Q_, d_x_in_, d_x_out_;              //!< @brief OpenCL memory objects
  cl_mem d_P_in_, d_P_out_;                    //!< @brief OpenCL memory objects
  size_t x_next_size_, A_size_;                //!< @brief size of OpenCL memory objects
  size_t x_size_, P_size_;                     //!< @brief size of OpenCL memory objects

  bool use_double_;                            //!< @brief flag to use double type in OpenCL kernel code
};
#endif
