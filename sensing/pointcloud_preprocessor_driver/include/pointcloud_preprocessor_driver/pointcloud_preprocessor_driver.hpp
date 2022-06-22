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

#ifndef POINTCLOUD_PREPROCESSOR_DRIVER__POINTCLOUD_PREPROCESSOR_DRIVER_HPP_
#define POINTCLOUD_PREPROCESSOR_DRIVER__POINTCLOUD_PREPROCESSOR_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>
#include <velodyne_pointcloud/rawdata.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>

class PointcloudPreprocessorDriver : public rclcpp::Node
{
public:
  PointcloudPreprocessorDriver(
    const std::string & node_name, const rclcpp::NodeOptions & options);
private:
  using fix16_t = int32_t;

  /* parameters for TCP/IP communication */
  static constexpr uint32_t PORTNO = 1235;             //!< @brief port number
  static constexpr char* IPADDR = (char*)"127.0.0.1";  //!< @brief IP address

  static constexpr uint32_t DATA_SIZE_RAW_DATA = 1206; //!< @brief size of raw data
  static constexpr uint32_t DATA_SIZE_POINT_DATA = 29; //!< @brief size of point data
  static constexpr uint32_t DATA_SIZE_TWIST_DATA = 12; //!< @brief size of twist data
  static constexpr uint32_t DATA_SIZE_PARAM_DATA = 12; //!< @brief size of parameter
  static constexpr uint32_t DATA_SIZE_EMPTY = 1;       //!< @brief size of empty packet

  static constexpr uint8_t CONNECT_RETRY_CNT = 5;      //!< @brief retry count of tcp/ip connection
  static constexpr uint8_t CONNECT_WAIT_TIME_US = 1;   //!< @brief waiting time(us) for retry connection

  static constexpr fix16_t FIX16_ONE = 0x00010000; //!< @brief fix16_t value of 1

  enum EXEC_MODE {
    NONE = 0,
    PREPROC_POINTCLOUD = 1,
    SET_TWIST = 2,
    SET_PARAM = 3
  };

  typedef struct
  {
    double min_range;
    double max_range;
    double scan_phase;
  } Config;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_interpolate_ex_pub_;
  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr velodyne_scan_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  int32_t sockfd_;  //!< @brief socket for TCP/IP communication
  Config config_;   //!< @brief config settings

  /**
   * @brief callback for raw scan messages
   */
  void processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg);

  /**
   * @brief callback for velocity report message
   */
  void processVelocityReport(
    const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr velocity_report_msg);

  /**
   * @brief callback for parameter setting
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

  /**
   * @brief request for point cloud preprocessing to accelerator
   */
  bool requestPointCloudPreprocessing(
    const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg,
    pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_points_xyziradt);

  /**
   * @brief request for twist data setting to accelerator
   */
  bool requestTwistDataSetting(
    const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr velocity_report_msg);

  /**
   * @brief Request for parameter setting to accelerator
   */
  bool requestParamDataSetting(void);

  /**
   * @brief handshake with accelerator
   */
  bool handshakeWithAccelerator(void);

  /**
   * @brief set data to send packet
   */
  template<typename T>
  inline char* setToSendPkt(char* outptr, const T* inptr);

  /*
   * @brief set data to send packet by size specification
   */
  template<typename T>
  inline char* setToSendPktBySize(char* outptr, const T* inptr, const uint32_t size);

  /**
   * @brief get data from receive packet
   */
  template<typename T>
  inline char* getFromRecvPkt(T* outptr, const char* inptr);

  /**
   * @brief type conversion from fix16 to float
   */
  template<typename T>
  inline T convFix16ToFloat(fix16_t a);

  /**
   * @brief type conversion from float to fix16
   */
  template<typename T>
  inline fix16_t convFloatToFix16(T a);

  /**
   * @brief unit conversion from second to nanosecond
   */
  inline uint64_t convSecToNanosec(int32_t sec);

  /**
   * @brief unit conversion from nanosecond to second
   */
  inline std::chrono::nanoseconds toChronoNanoSeconds(const double seconds);
};

#endif  // POINTCLOUD_PREPROCESSOR_DRIVER__POINTCLOUD_PREPROCESSOR_DRIVER_HPP_
