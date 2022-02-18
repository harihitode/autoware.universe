/*
 * Copyright 2021 Tier IV inc. All rights reserved.
 *
 * The TimeDelayKalmanFilterOCL functions are based
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "kalman_filter/time_delay_kalman_filter_ocl.hpp"

#define EKF_KERNEL_FILE "kernel/predict_with_delay.cl"
#define EKF_KERNEL_NAME "predictWithDelayCL"

TimeDelayKalmanFilterOCL::TimeDelayKalmanFilterOCL()
{
  Queue_ = NULL;
  k_predict_with_delay_ = NULL;
  context_ = NULL;
  program_ = NULL;
  d_x_next_ = NULL;
  d_A_ = NULL;
  d_A_transpose_ = NULL;
  d_Q_ = NULL;
  d_x_in_ = NULL;
  d_x_out_ = NULL;
  d_P_in_ = NULL;
  d_P_out_ = NULL;
  use_double_ = true;
}

TimeDelayKalmanFilterOCL::~TimeDelayKalmanFilterOCL()
{
  finalize();
}

void TimeDelayKalmanFilterOCL::init(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0, const int max_delay_step, bool use_double)
{
  max_delay_step_ = max_delay_step;
  dim_x_ = x.rows();
  dim_x_ex_ = dim_x_ * max_delay_step;

  x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

  for (int i = 0; i < max_delay_step_; ++i) {
    x_.block(i * dim_x_, 0, dim_x_, 1) = x;
    P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
  }

  use_double_ = use_double;

  // initialize OpenCL
  cl_uint platform = 0;
  cl_uint device = 0;
  int ret = initializeOCL(platform, device);
  if (ret != 0) {
    std::cerr << "error : initializing OpenCL failed." << std::endl;
    finalizeOCL();
  } else {
    if (use_double_) {
      ret = createOCLMemoryObjects<double>();
    } else {
      ret = createOCLMemoryObjects<float>();
    }
    if (ret != 0) {
      std::cerr << "error : failed to create OpenCL memory objects" << std::endl;
    }
  }
}

void TimeDelayKalmanFilterOCL::finalize()
{
  finalizeOCL();
}

void TimeDelayKalmanFilterOCL::getLatestX(Eigen::MatrixXd & x) { x = x_.block(0, 0, dim_x_, 1); }
void TimeDelayKalmanFilterOCL::getLatestP(Eigen::MatrixXd & P) { P = P_.block(0, 0, dim_x_, dim_x_); }

bool TimeDelayKalmanFilterOCL::predictWithDelay(
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  /*
   * time delay model:
   *
   *     [A   0   0]      [P11   P12   P13]      [Q   0   0]
   * A = [I   0   0], P = [P21   P22   P23], Q = [0   0   0]
   *     [0   I   0]      [P31   P32   P33]      [0   0   0]
   *
   * covariance calculation in prediction : P = A * P * A' + Q
   *
   *     [A*P11*A'*+Q  A*P11  A*P12]
   * P = [     P11*A'    P11    P12]
   *     [     P21*A'    P21    P22]
   */

  int ret;
  if (use_double_) {
    ret = copyToOCLMemoryObjects<double>(x_next, A, Q);
  } else {
    ret = copyToOCLMemoryObjects<float>(x_next, A, Q);
  }

  if (ret != 0) {
    std::cerr << "error : failed to copy to OpenCL memory objects" << std::endl;
  } else {
    ret = predictWithDelayCL();
  }

  if (ret != 0) {
    std::cerr << "error : failed to call predictWithDelayCL" << std::endl;
  } else {
    if (use_double_) {
      ret = readOCLMemoryObjects<double>();
    } else {
      ret = readOCLMemoryObjects<float>();
    }
  }

  if (ret != 0) {
    std::cerr << "error : failed to read OpenCL memory objects" << std::endl;
  }
  return true;
}

bool TimeDelayKalmanFilterOCL::updateWithDelay(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
  const int delay_step)
{
  if (delay_step >= max_delay_step_) {
    std::cerr << "delay step is larger than max_delay_step. ignore update." << std::endl;
    return false;
  }

  const int dim_y = y.rows();

  /* set measurement matrix */
  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
  C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

  /* update */
  if (!update(y, C_ex, R)) return false;

  return true;
}

int TimeDelayKalmanFilterOCL::initializeOCL(cl_uint platform, cl_uint device)
{
  size_t source_size, ret_size;
  cl_uint num_platforms, num_devices;
  cl_int ret;

  // platform
  ret = clGetPlatformIDs(MAX_PLATFORMS, platform_id_, &num_platforms);
  if (ret != CL_SUCCESS) {
    std::cerr << "error : clGetPlatformIDs() error " << ret << std::endl;
    return 1;
  }
  if (platform >= num_platforms) {
    std::cerr << "error : platform = " << platform << "(limit = " << num_platforms - 1 << ")" << std::endl;
    return 1;
  }

  // device
  ret = clGetDeviceIDs(platform_id_[platform], CL_DEVICE_TYPE_ALL, MAX_DEVICES, device_id_, &num_devices);
  if (ret != CL_SUCCESS) {
    std::cerr << "error : clGetDeviceIDs() error " << ret << std::endl;
    return 1;
  }
  if (device >= num_devices) {
    std::cerr << "error : device = " << device << "(limit = " << num_devices - 1 << ")" << std::endl;
    return 1;
  }

  // device name (optional information)
  {
    char str[BUFSIZ];
    ret = clGetDeviceInfo(device_id_[device], CL_DEVICE_NAME, sizeof(str), str, &ret_size);
    if (ret != CL_SUCCESS) {
      std::cerr << "error : clGetDeviceInfo() error " << ret << std::endl;
      return 1;
    }
    std::cout << "info : " << str << " (platform = " << platform << ", device = " << device << ")" << std::endl;
  }

  // context
  context_ = clCreateContext(NULL, 1, &device_id_[device], NULL, NULL, &ret);
  if (ret != CL_SUCCESS) {
    std::cerr << "error : clCreateContext() error " << ret << std::endl;
    return 1;
  }

  // command queue
  Queue_ = clCreateCommandQueue(context_, device_id_[device], 0, &ret);
  if (ret != CL_SUCCESS) {
    std::cerr << "error : clCreateCommandQueue() error " << ret << std::endl;
    return 1;
  }

  // source
  {
    FILE * fp;
    std::string kernel_file_path = ament_index_cpp::get_package_share_directory("kalman_filter") + "/" + EKF_KERNEL_FILE;
    if ((fp = fopen(kernel_file_path.c_str(), "r")) == NULL) {
      std::cerr << "error : could not open " << kernel_file_path << std::endl;
      return 1;
    }
    // alloc
    char source_str[MAX_SOURCE_SIZE];
    char * source_ptr = source_str;
    source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
    // create program
    program_ = clCreateProgramWithSource(context_, 1, (const char **)&source_ptr, (const size_t *)&source_size, &ret);
    fclose(fp);
    if (ret != CL_SUCCESS) {
      std::cerr << "error : clCreateProgramWithSource() error " << ret << std::endl;
      return 1;
    }
  }

  // build program
  std::string option = "-DTYPE=";
  if (use_double_) {
    option += "double";
  } else {
    option += "float";
  }
  if (clBuildProgram(program_, 1, &device_id_[device], option.c_str(), NULL, NULL) != CL_SUCCESS) {
    std::cerr << "error : clBuildProgram() error" << std::endl;
    size_t logSize;
    clGetProgramBuildInfo(program_, device_id_[device], CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
    std::unique_ptr<char[]> buildLog(new char[logSize + 1]);
    clGetProgramBuildInfo(program_, device_id_[device], CL_PROGRAM_BUILD_LOG, logSize, buildLog.get(), NULL);
    std::cout << buildLog.get() << std::endl;
    return 1;
  }

  // kernel
  k_predict_with_delay_ = clCreateKernel(program_, EKF_KERNEL_NAME, &ret);
  if (ret != CL_SUCCESS) {
    std::cerr << "error : clCreateKernel() error" << std::endl;
    return 1;
  }
  return 0;
}

template <typename Type>
int TimeDelayKalmanFilterOCL::createOCLMemoryObjects(void)
{
  x_next_size_ = dim_x_ * sizeof(Type);
  A_size_ = dim_x_ * dim_x_ * sizeof(Type);
  x_size_ = dim_x_ex_ * sizeof(Type);
  P_size_ = dim_x_ex_ * dim_x_ex_ * sizeof(Type);
  cl_int ret;

  OCL_CREATE_BUFFER_CHECK(d_x_next_, context_, CL_MEM_READ_WRITE, x_next_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_A_, context_, CL_MEM_READ_WRITE, A_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_A_transpose_, context_, CL_MEM_READ_WRITE, A_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_Q_, context_, CL_MEM_READ_WRITE, A_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_x_in_, context_, CL_MEM_READ_WRITE, x_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_x_out_, context_, CL_MEM_READ_WRITE, x_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_P_in_, context_, CL_MEM_READ_WRITE, P_size_, NULL, ret);
  OCL_CREATE_BUFFER_CHECK(d_P_out_, context_, CL_MEM_READ_WRITE, P_size_, NULL, ret);

  return 0;
}

template <typename Type>
int TimeDelayKalmanFilterOCL::copyToOCLMemoryObjects(
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  Type ocl_x_next[DIM_X];
  Type ocl_A[DIM_X][DIM_X];
  Type ocl_A_transpose[DIM_X][DIM_X];
  Type ocl_Q[DIM_X][DIM_X];
  std::unique_ptr<Type[]> ocl_x_in(new Type[dim_x_ex_]);
  std::unique_ptr<Type[]> ocl_P_in(new Type[dim_x_ex_ * dim_x_ex_]);
  auto A_transpose = A.transpose();

  // copy data
  for (int i = 0; i < DIM_X; i++) {
    ocl_x_next[i] = x_next(i);
  }
  for (int i = 0; i < DIM_X; i++) {
    for (int j = 0; j < DIM_X; j++) {
      ocl_A[i][j] = A(i,j);
      ocl_A_transpose[i][j] = A_transpose(i,j);
      ocl_Q[i][j] = Q(i,j);
    }
  }
  for (int i = 0; i < dim_x_ex_; i++) {
    ocl_x_in[i] = x_(i);
  }
  for (int i = 0; i < dim_x_ex_; i++) {
    for (int j = 0; j < dim_x_ex_; j++) {
      ocl_P_in[i * dim_x_ex_ + j] = P_(i,j);
    }
  }

  OCL_WRITE_BUFFER_CHECK(Queue_, d_x_next_, CL_TRUE, 0, x_next_size_, ocl_x_next, 0, NULL, NULL);
  OCL_WRITE_BUFFER_CHECK(Queue_, d_A_, CL_TRUE, 0, A_size_, ocl_A, 0, NULL, NULL);
  OCL_WRITE_BUFFER_CHECK(Queue_, d_A_transpose_, CL_TRUE, 0, A_size_, ocl_A_transpose, 0, NULL, NULL);
  OCL_WRITE_BUFFER_CHECK(Queue_, d_Q_, CL_TRUE, 0, A_size_, ocl_Q, 0, NULL, NULL);
  OCL_WRITE_BUFFER_CHECK(Queue_, d_x_in_, CL_TRUE, 0, x_size_, ocl_x_in.get(), 0, NULL, NULL);
  OCL_WRITE_BUFFER_CHECK(Queue_, d_P_in_, CL_TRUE, 0, P_size_, ocl_P_in.get(), 0, NULL, NULL);

  return 0;
}

int TimeDelayKalmanFilterOCL::predictWithDelayCL(void)
{
  const size_t local_item_size = MAX_THREAD_NUM;
  size_t global_item_size = local_item_size;
  cl_int ret;

  // set arguments
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 0, sizeof(cl_mem), (void *)&d_x_next_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 1, sizeof(cl_mem), (void *)&d_A_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 2, sizeof(cl_mem), (void *)&d_A_transpose_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 3, sizeof(cl_mem), (void *)&d_Q_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 4, sizeof(cl_mem), (void *)&d_x_in_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 5, sizeof(cl_mem), (void *)&d_x_out_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 6, sizeof(cl_mem), (void *)&d_P_in_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 7, sizeof(cl_mem), (void *)&d_P_out_);
  OCL_SET_KERNEL_ARG_CHECK(k_predict_with_delay_, 8, sizeof(int), (void *)&dim_x_ex_);

  // kicking the kernel
  ret = clEnqueueNDRangeKernel(Queue_, k_predict_with_delay_, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);
  if (CL_SUCCESS != ret) {
    std::cerr << "error : clEnqueueNDRangeKernel error code " << ret << std::endl;
    return -1;
  }
  return 0;
}

template <typename Type>
int TimeDelayKalmanFilterOCL::readOCLMemoryObjects()
{
  std::unique_ptr<Type[]> ocl_x_out(new Type[dim_x_ex_]);
  std::unique_ptr<Type[]> ocl_P_out(new Type[dim_x_ex_ * dim_x_ex_]);

  OCL_READ_BUFFER_CHECK(Queue_, d_x_out_, CL_TRUE, 0, x_size_, ocl_x_out.get(), 0, NULL, NULL);
  OCL_READ_BUFFER_CHECK(Queue_, d_P_out_, CL_TRUE, 0, P_size_, ocl_P_out.get(), 0, NULL, NULL);

  for (int i = 0; i < dim_x_ex_; i++) {
    x_(i) = ocl_x_out[i];
  }
  for (int i = 0; i < dim_x_ex_; i++) {
    for (int j = 0; j < dim_x_ex_; j++) {
      P_(i,j) = ocl_P_out[i * dim_x_ex_ + j];
    }
  }

  return 0;
}

void TimeDelayKalmanFilterOCL::releaseOCLMemoryObjects()
{
  OCL_RELEASE_MEMORY_CHECK(d_x_next_);
  OCL_RELEASE_MEMORY_CHECK(d_A_);
  OCL_RELEASE_MEMORY_CHECK(d_A_transpose_);
  OCL_RELEASE_MEMORY_CHECK(d_Q_);
  OCL_RELEASE_MEMORY_CHECK(d_x_in_);
  OCL_RELEASE_MEMORY_CHECK(d_x_out_);
  OCL_RELEASE_MEMORY_CHECK(d_P_in_);
  OCL_RELEASE_MEMORY_CHECK(d_P_out_);
}

void TimeDelayKalmanFilterOCL::finalizeOCL()
{
  releaseOCLMemoryObjects();
  OCL_RELEASE_KERNEL_CHECK(k_predict_with_delay_);
  OCL_RELEASE_QUEUE_CHECK(Queue_);
  OCL_RELEASE_PROGRAM_CHECK(program_);
  OCL_RELEASE_CONTEXT_CHECK(context_);
}
