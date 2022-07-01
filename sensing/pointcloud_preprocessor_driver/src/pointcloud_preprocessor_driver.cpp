// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "pointcloud_preprocessor_driver/pointcloud_preprocessor_driver.hpp"
#include <string>

/** \brief For parameter service callback */
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

/** \brief PointcloudPreprocessorDriver constructor */
PointcloudPreprocessorDriver::PointcloudPreprocessorDriver(const std::string & node_name,
  const rclcpp::NodeOptions & node_options) : Node(node_name, node_options)
{
  RCLCPP_INFO(this->get_logger(),
    "This node is only tested for VLP16. Use other models at your own risk.");

  rcl_interfaces::msg::ParameterDescriptor min_range_desc;
  min_range_desc.name = "min_range";
  min_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_range_desc.description = "minimum range to publish";
  rcl_interfaces::msg::FloatingPointRange min_range_range;
  min_range_range.from_value = 0.1;
  min_range_range.to_value = 10.0;
  min_range_desc.floating_point_range.push_back(min_range_range);
  config_.min_range = this->declare_parameter("min_range", 0.9, min_range_desc);

  rcl_interfaces::msg::ParameterDescriptor max_range_desc;
  max_range_desc.name = "max_range";
  max_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_range_desc.description = "maximum range to publish";
  rcl_interfaces::msg::FloatingPointRange max_range_range;
  max_range_range.from_value = 0.1;
  max_range_range.to_value = 250.0;
  max_range_desc.floating_point_range.push_back(max_range_range);
  config_.max_range = this->declare_parameter("max_range", 130.0, max_range_desc);

  rcl_interfaces::msg::ParameterDescriptor scan_phase_desc;
  scan_phase_desc.name = "scan_phase";
  scan_phase_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  scan_phase_desc.description = "start/end phase for the scan (in degrees)";
  rcl_interfaces::msg::FloatingPointRange scan_phase_range;
  scan_phase_range.from_value = 0.0;
  scan_phase_range.to_value = 359.0;
  scan_phase_desc.floating_point_range.push_back(scan_phase_range);
  config_.scan_phase = this->declare_parameter("scan_phase", 0.0, scan_phase_desc);

  max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 5));

  // publish
  pub_output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_));

  // set parameter callback
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PointcloudPreprocessorDriver::paramCallback, this, _1));

  // subscribe
  velodyne_scan_ =
    this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&PointcloudPreprocessorDriver::processScan, this, std::placeholders::_1));
  velocity_report_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 10,
    std::bind(&PointcloudPreprocessorDriver::processVelocityReport, this, std::placeholders::_1));

  /* request param data setting */
  if (!requestParamDataSetting()) {
    return;
  }
}

/** @brief Callback for raw scan messages. */
void PointcloudPreprocessorDriver::processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg)
{
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);

  /* request point cloud preprocessing */
  if (!requestPointCloudPreprocessing(scanMsg, points_xyziradt)) {
    return;
  }

  auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*points_xyziradt, *ros_pc_msg_ptr);
  pub_output_->publish(std::move(ros_pc_msg_ptr));
}

/** \brief Callback for velocity report message */
void PointcloudPreprocessorDriver::processVelocityReport(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr velocity_report_msg)
{
  /* request twist data setting */
  if (!requestTwistDataSetting(velocity_report_msg)) {
    return;
  }
}

/** \brief Callback for parameter setting */
rcl_interfaces::msg::SetParametersResult PointcloudPreprocessorDriver::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  RCLCPP_INFO(this->get_logger(), "Reconfigure Request");

  Config new_config;

  if(get_param(p, "min_range", new_config.min_range) ||
     get_param(p, "max_range", new_config.max_range))
  {
    config_.min_range = new_config.min_range;
    config_.max_range = new_config.max_range;
  }
  get_param(p, "scan_phase", config_.scan_phase);

  rcl_interfaces::msg::SetParametersResult result;

  /* request param data setting */
  if (!requestParamDataSetting()) {
    result.successful = false;
    result.reason = "param data request error";
  } else {
    result.successful = true;
    result.reason = "success";
  }
  return result;
}


/** \brief Request for point cloud preprocessing to accelerator. */
bool PointcloudPreprocessorDriver::requestPointCloudPreprocessing(
  const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg,
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr points_xyziradt)
{
  /* handshake with accelerator */
  if (!handshakeWithAccelerator()) {
    return false;
  }

  /* create send packet */
  char exec_mode = static_cast<char>(EXEC_MODE::PREPROC_POINTCLOUD);
  uint32_t velodyne_packet_count = scanMsg->packets.size();
  uint32_t velodyne_packet_time_stamp_32;
  uint32_t send_pkt_len = sizeof(exec_mode) + sizeof(velodyne_packet_count) +
    (velodyne_packet_count * (sizeof(velodyne_packet_time_stamp_32) + DATA_SIZE_RAW_DATA));
  char* send_buf = new char[send_pkt_len];
  char* send_ptr = send_buf;
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &velodyne_packet_count);
  for (uint32_t i = 0; i < velodyne_packet_count; i++) {
    char* velodyne_packet_data = reinterpret_cast<char *>(&scanMsg->packets[i].data[0]);
    uint64_t time_stamp_64 =
      convSecToNanosec(scanMsg->packets[i].stamp.sec) + scanMsg->packets[i].stamp.nanosec;
    velodyne_packet_time_stamp_32 = static_cast<uint32_t>(time_stamp_64 / 100U); /* 1[ns] -> 100[ns] */
    send_ptr = setToSendPkt(send_ptr, &velodyne_packet_time_stamp_32);
    send_ptr = setToSendPktBySize(send_ptr, velodyne_packet_data, DATA_SIZE_RAW_DATA);
  }

  /* send message */
  int32_t ret = send(sockfd_, send_buf, send_pkt_len, 0);
  delete[] send_buf;
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP send error : %s", std::strerror(errno));
    close(sockfd_);
    return false;
  }

  /* receive message */
  uint32_t point_num;
  char* recv_buf = new char[DATA_SIZE_POINT_DATA];
  /* get point num from stream */
  ret = recv(sockfd_, recv_buf, sizeof(point_num), MSG_WAITALL);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP receive error : %s", std::strerror(errno));
    close(sockfd_);
    delete[] recv_buf;
    return false;
  }
  char* recv_ptr = recv_buf;
  recv_ptr = getFromRecvPkt(&point_num, recv_ptr);

  fix16_t fix16_x;
  fix16_t fix16_y;
  fix16_t fix16_z;
  fix16_t fix16_intensity;
  fix16_t fix16_distance;
  velodyne_pointcloud::PointXYZIRADT point_data_tmp;
  uint32_t time_stamp_32;
  for (uint32_t i = 0; i < point_num; i++) {
    /* get point data from stream */
    ret = recv(sockfd_, recv_buf, DATA_SIZE_POINT_DATA, MSG_WAITALL);
    if (ret < 0) {
      RCLCPP_ERROR(get_logger(), "[PC] TCP/IP receive error : %s", std::strerror(errno));
      close(sockfd_);
      delete[] recv_buf;
      return false;
    }
    recv_ptr = recv_buf;
    recv_ptr = getFromRecvPkt(&fix16_x, recv_ptr);
    recv_ptr = getFromRecvPkt(&fix16_y, recv_ptr);
    recv_ptr = getFromRecvPkt(&fix16_z, recv_ptr);
    recv_ptr = getFromRecvPkt(&fix16_intensity, recv_ptr);
    recv_ptr = getFromRecvPkt(&point_data_tmp.azimuth, recv_ptr);
    recv_ptr = getFromRecvPkt(&fix16_distance, recv_ptr);
    recv_ptr = getFromRecvPkt(&point_data_tmp.ring, recv_ptr);
    recv_ptr = getFromRecvPkt(&point_data_tmp.return_type, recv_ptr);
    recv_ptr = getFromRecvPkt(&time_stamp_32, recv_ptr);
    point_data_tmp.x          = convFix16ToFloat<float>(fix16_x);
    point_data_tmp.y          = convFix16ToFloat<float>(fix16_y);
    point_data_tmp.z          = convFix16ToFloat<float>(fix16_z);
    point_data_tmp.intensity  = convFix16ToFloat<float>(fix16_intensity);
    point_data_tmp.distance   = convFix16ToFloat<float>(fix16_distance);
    point_data_tmp.time_stamp = 
      static_cast<double>(time_stamp_32) / 1000.0 / 1000.0 / 10.0; /* 100[ns] -> 1[s] */
    points_xyziradt->points.push_back(point_data_tmp);
  }
  delete[] recv_buf;

  points_xyziradt->header = pcl_conversions::toPCL(scanMsg->header);
  // Find timestamp from first/last point (and maybe average)?
  double first_point_timestamp = points_xyziradt->points.front().time_stamp;
  points_xyziradt->header.stamp =
    pcl_conversions::toPCL(rclcpp::Time(toChronoNanoSeconds(first_point_timestamp).count()));
  points_xyziradt->height = 1;
  points_xyziradt->width = points_xyziradt->points.size();

  close(sockfd_);

  return true;
}

/** \brief Request for twist data setting to accelerator. */
bool PointcloudPreprocessorDriver::requestTwistDataSetting(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr velocity_report_msg)
{
  /* handshake with accelerator */
  if (!handshakeWithAccelerator()) {
    return false;
  }

  /* create send packet */
  char exec_mode = static_cast<char>(EXEC_MODE::SET_TWIST);
  uint32_t send_pkt_len = sizeof(exec_mode) + DATA_SIZE_TWIST_DATA;
  char* send_buf = new char[send_pkt_len];
  uint64_t velocity_report_msg_header_stamp_64 =
    convSecToNanosec(velocity_report_msg->header.stamp.sec) + velocity_report_msg->header.stamp.nanosec;
  uint32_t velocity_report_msg_header_stamp_32 =
    static_cast<uint32_t>(velocity_report_msg_header_stamp_64 / 100U); /* 1[ns] -> 100[ns] */
  fix16_t fix16_longitudinal_velocity =
    convFloatToFix16(velocity_report_msg->longitudinal_velocity);
  fix16_t fix16_heading_rate = convFloatToFix16(velocity_report_msg->heading_rate);
  char* send_ptr = send_buf;
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &velocity_report_msg_header_stamp_32);
  send_ptr = setToSendPkt(send_ptr, &fix16_longitudinal_velocity);
  send_ptr = setToSendPkt(send_ptr, &fix16_heading_rate);

  /* send message */
  int32_t ret = send(sockfd_, send_buf, send_pkt_len, 0);
  delete[] send_buf;
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP send error : %s", std::strerror(errno));
    close(sockfd_);
    return false;
  }

  /* receive message */
  /* synchronize the timing with accelerator */
  char recv_buf;
  ret = recv(sockfd_, &recv_buf, DATA_SIZE_EMPTY, MSG_WAITALL);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP receive error : %s", std::strerror(errno));
    close(sockfd_);
    return false;
  }

  close(sockfd_);

  return true;
}

/** \brief Request for parameter setting to accelerator. */
bool PointcloudPreprocessorDriver::requestParamDataSetting(void)
{
  /* handshake with accelerator */
  if (!handshakeWithAccelerator()) {
    return false;
  }

  /* create send packet */
  char exec_mode = static_cast<char>(EXEC_MODE::SET_PARAM);
  uint32_t send_pkt_len = sizeof(exec_mode) + DATA_SIZE_PARAM_DATA;
  char* send_buf = new char[send_pkt_len];
  fix16_t fix16_min_range  = convFloatToFix16(config_.min_range);
  fix16_t fix16_max_range  = convFloatToFix16(config_.max_range);
  fix16_t fix16_scan_phase = convFloatToFix16(config_.scan_phase);

  char* send_ptr = send_buf;
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &fix16_min_range);
  send_ptr = setToSendPkt(send_ptr, &fix16_max_range);
  send_ptr = setToSendPkt(send_ptr, &fix16_scan_phase);

  /* send message */
  int32_t ret = send(sockfd_, send_buf, send_pkt_len, 0);
  delete[] send_buf;
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP send error : %s", std::strerror(errno));
    close(sockfd_);
    return false;
  }

  /* receive message */
  /* synchronize the timing with accelerator */
  char recv_buf;
  ret = recv(sockfd_, &recv_buf, DATA_SIZE_EMPTY, MSG_WAITALL);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP receive error : %s", std::strerror(errno));
    close(sockfd_);
    return false;
  }

  close(sockfd_);

  return true;
}

/** \brief Handshake with accelerator. */
bool PointcloudPreprocessorDriver::handshakeWithAccelerator(void)
{
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd_ < 0){
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP socket error : %s", std::strerror(errno));
    return false;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(struct sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORTNO);
  addr.sin_addr.s_addr = inet_addr(IPADDR);

  int retry_cnt = CONNECT_RETRY_CNT;
  while (retry_cnt > 0) {
    int32_t ret = connect(sockfd_, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
      retry_cnt--;
      RCLCPP_WARN(get_logger(), "[PC] TCP/IP connect error : %s. retry connection.", std::strerror(errno));
    } else {
      break;
    }
    rclcpp::sleep_for(std::chrono::microseconds(CONNECT_WAIT_TIME_US));
  }
  if (retry_cnt <= 0) {
    RCLCPP_ERROR(get_logger(), "[PC] TCP/IP connect error : %s", std::strerror(errno));
    close(sockfd_);
    return false;
  }

  return true;
}

/** \brief set to send packet. */
template<typename T>
inline char* PointcloudPreprocessorDriver::setToSendPkt(char* outptr, const T* inptr)
{
  std::memcpy(outptr, inptr, sizeof(T));
  return (outptr + sizeof(T));
}

/** \brief set data to send packet by size specification. */
template<typename T>
inline char* PointcloudPreprocessorDriver::setToSendPktBySize(
  char* outptr, const T* inptr, const uint32_t size)
{
  std::memcpy(outptr, inptr, size);
  return (outptr + size);
}

/** \brief get from receive packet. */
template<typename T>
inline char* PointcloudPreprocessorDriver::getFromRecvPkt(T* outptr, const char* inptr)
{
  std::memcpy(outptr, inptr, sizeof(T));
  return (const_cast<char*>(inptr) + sizeof(T));
}

/** \brief type conversion from fix16 to float. */
template<typename T>
inline T PointcloudPreprocessorDriver::convFix16ToFloat(fix16_t a)
{
  return static_cast<T>(a) / FIX16_ONE;
}

/** \brief type conversion from float to fix16. */
template<typename T>
inline PointcloudPreprocessorDriver::fix16_t
  PointcloudPreprocessorDriver::convFloatToFix16(T a)
{
  T temp = a * FIX16_ONE;
  temp += (temp >= 0) ? 0.5f : -0.5f;
  return (fix16_t)temp;
}

/** \brief unit conversion from second to nanosecond. */
inline uint64_t PointcloudPreprocessorDriver::convSecToNanosec(int32_t sec)
{
  return static_cast<uint64_t>(sec) * 1000U * 1000U * 1000U;
}

/** \brief unit conversion from nanosecond to second. */
inline std::chrono::nanoseconds PointcloudPreprocessorDriver::toChronoNanoSeconds(
  const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(seconds));
}

