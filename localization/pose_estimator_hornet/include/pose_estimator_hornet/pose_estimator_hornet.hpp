// Copyright 2021 Tier IV inc. All rights reserved.
//
// This class is also licensed under the Apache License, Version 2.0.
//
// ORIGINAL LICENSE
//
// Copyright 2018-2019 Autoware Foundation
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

#ifndef EKF_LOCALIZER__EKF_LOCALIZER_OCL_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_OCL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>

class PoseEstimatorHornet : public rclcpp::Node
{
public:
  PoseEstimatorHornet(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  /* parameters for TCP/IP communication */
  static constexpr unsigned int PORTNO = 1234;        //!< @brief port number
  static constexpr char* IPADDR = (char*)"127.0.0.1"; //!< @brief IP address

  //!< @brief length of send packet
  static constexpr unsigned int SEND_PKT_LEN = 129;
  //!< @brief length of receive packet for EKF calcuration's result
  static constexpr unsigned int RECV_PKT_LEN_EKFCALC = 152;
  //!< @brief length of receive packet for empty packet
  static constexpr unsigned int RECV_PKT_LEN_EMPTY = 1;

  //!< @brief size of send buffer
  static constexpr unsigned int SEND_BUF_SIZE = SEND_PKT_LEN;
  //!< @brief size of receive buffer
  static constexpr unsigned int RECV_BUF_SIZE = RECV_PKT_LEN_EKFCALC;

  //!< @brief ekf estimated pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  //!< @brief estimated ekf pose with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  //!< @brief estimated ekf odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  //!< @brief ekf estimated twist publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  //!< @brief ekf estimated twist with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  //!< @brief debug info publisher
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr pub_debug_;
  //!< @brief debug measurement pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_measured_pose_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr pub_yaw_bias_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_no_yawbias_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_pose_cov_no_yawbias_;
  //!< @brief initial pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;
  //!< @brief time for ekf calculation callback
  rclcpp::TimerBase::SharedPtr timer_control_;
  //!< @brief timer to send transform
  rclcpp::TimerBase::SharedPtr timer_tf_;
  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  /* parameters */
  bool show_debug_info_;
  double ekf_rate_;                  //!< @brief  EKF predict rate
  double ekf_dt_;                    //!< @brief  = 1 / ekf_rate_
  double tf_rate_;                   //!< @brief  tf publish rate
  bool enable_yaw_bias_estimation_;  //!< @brief for LiDAR mount error.
                                     //!< if true,publish /estimate_yaw_bias
  std::string pose_frame_id_;

  int dim_x_;              //!< @brief  dimension of EKF state
  int extend_state_step_;  //!< @brief  for time delay compensation

  /* Pose */
  double pose_additional_delay_;          //!< @brief  compensated pose delay time =
                                          //!< (pose.header.stamp - now) + additional_delay [s]
  double pose_rate_;  //!< @brief  pose rate [s], used for covariance calculation
  //!< @brief  the mahalanobis distance threshold to ignore pose measurement
  double pose_gate_dist_;

  /* twist */
  double twist_additional_delay_;  //!< @brief  compensated delay = (twist.header.stamp - now)
                                   //!< + additional_delay [s]
  double twist_rate_;              //!< @brief  rate [s], used for covariance calculation
  //!< @brief  measurement is ignored if the mahalanobis distance is larger than this value.
  double twist_gate_dist_;

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
  double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
  double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

  Eigen::MatrixXd x_;  //!< @brief current estimated state
  Eigen::MatrixXd P_;  //!< @brief covariance of estimated state

  char send_buf_[SEND_BUF_SIZE]; //!< @brief send buffer for TCP/IP communication
  char recv_buf_[RECV_BUF_SIZE]; //!< @brief receive buffer for TCP/IP communication

  enum IDX {
    X = 0,
    Y = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };

  enum EXEC_MODE {
    NONE = 0,
    CALC_EKF = 1,
    SET_INITIAL_POSE = 2,
    SET_POSE_WITH_COVARIANCE = 3,
    SET_TWIST_WITH_COVARIANCE = 4
  };

  /* for model prediction */
  geometry_msgs::msg::TwistStamped::SharedPtr
    current_twist_ptr_;                                          //!< @brief current measured twist
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ptr_;  //!< @brief current measured pose
  geometry_msgs::msg::PoseStamped current_ekf_pose_;             //!< @brief current estimated pose
  geometry_msgs::msg::PoseStamped
    current_ekf_pose_no_yawbias_;  //!< @brief current estimated pose w/o yaw bias
  geometry_msgs::msg::TwistStamped current_ekf_twist_;  //!< @brief current estimated twist
  std::array<double, 36ul> current_pose_covariance_;
  std::array<double, 36ul> current_twist_covariance_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback();

  /**
   * @brief publish tf for tf_rate [Hz]
   */
  void timerTFCallback();

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPoseWithCovariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovariance(geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callbackInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief get transform from frame_id
   */
  bool getTransformFromTF(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief set current EKF estimation result to current_ekf_pose_ & current_ekf_twist_
   */
  void setCurrentResult();

  /**
   * @brief publish current EKF estimation result
   */
  void publishEstimateResult();

  /**
   * @brief communicate with EKF Accelerator
   */
  bool communicateWithEKFAccelerator(
    const unsigned int send_pkt_len, const unsigned int recv_pkt_len);

  /**
   * @brief set data to send packet
   */
  template<typename T>
  inline char* setToSendPkt(char* outptr, const T* inptr);

  /**
   * @brief get data from receive packet
   */
  template<typename T>
  inline char* getFromRecvPkt(T* outptr, const char* inptr);

  /**
   * @brief unit conversion from seconds to nanoseconds
   */
  inline unsigned long convSecToNanosec(unsigned int sec);

  /**
   * @brief for debug
   */
  void showCurrentX();

  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_; //!< @brief for debug
};
#endif
