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

#include "pose_estimator_hornet/pose_estimator_hornet.hpp"

#include <rclcpp/logging.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on

using std::placeholders::_1;

PoseEstimatorHornet::PoseEstimatorHornet(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
{
  show_debug_info_ = declare_parameter("show_debug_info", false);
  ekf_rate_ = declare_parameter("predict_frequency", 50.0);
  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  tf_rate_ = declare_parameter("tf_rate", 10.0);
  enable_yaw_bias_estimation_ = declare_parameter("enable_yaw_bias_estimation", true);
  extend_state_step_ = declare_parameter("extend_state_step", 50);
  pose_frame_id_ = declare_parameter("pose_frame_id", std::string("map"));

  /* pose measurement */
  pose_additional_delay_ = declare_parameter("pose_additional_delay", 0.0);
  pose_rate_ = declare_parameter("pose_rate", 10.0);  // used for covariance calculation
  pose_gate_dist_ = declare_parameter("pose_gate_dist", 10000.0);  // Mahalanobis limit

  /* twist measurement */
  twist_additional_delay_ = declare_parameter("twist_additional_delay", 0.0);
  twist_rate_ = declare_parameter("twist_rate", 10.0);  // used for covariance calculation
  twist_gate_dist_ = declare_parameter("twist_gate_dist", 10000.0);  // Mahalanobis limit

  /* process noise */
  double proc_stddev_yaw_c, proc_stddev_yaw_bias_c, proc_stddev_vx_c, proc_stddev_wz_c;
  proc_stddev_yaw_c = declare_parameter("proc_stddev_yaw_c", 0.005);
  proc_stddev_yaw_bias_c = declare_parameter("proc_stddev_yaw_bias_c", 0.001);
  proc_stddev_vx_c = declare_parameter("proc_stddev_vx_c", 5.0);
  proc_stddev_wz_c = declare_parameter("proc_stddev_wz_c", 1.0);
  if (!enable_yaw_bias_estimation_) {
    proc_stddev_yaw_bias_c = 0.0;
  }

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c * ekf_dt_, 2.0);
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c * ekf_dt_, 2.0);

  /* initialize ros system */
  auto period_control_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ekf_dt_));
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), period_control_ns, std::bind(&PoseEstimatorHornet::timerCallback, this));

  const auto period_tf_ns = rclcpp::Rate(tf_rate_).period();
  timer_tf_ = rclcpp::create_timer(
    this, get_clock(), period_tf_ns, std::bind(&PoseEstimatorHornet::timerTFCallback, this));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_yaw_bias_ = create_publisher<tier4_debug_msgs::msg::Float64Stamped>("estimated_yaw_bias", 1);
  pub_pose_no_yawbias_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose_without_yawbias", 1);
  pub_pose_cov_no_yawbias_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance_without_yawbias", 1);
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&PoseEstimatorHornet::callbackInitialPose, this, _1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1, std::bind(&PoseEstimatorHornet::callbackPoseWithCovariance, this, _1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1, std::bind(&PoseEstimatorHornet::callbackTwistWithCovariance, this, _1));

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  x_ = Eigen::MatrixXd::Zero(dim_x_, 1);
  P_ = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  /* create send packet */
  char* send_ptr = &send_buf_[0];
  send_ptr = setToSendPkt(send_ptr, &ekf_rate_);
  send_ptr = setToSendPkt(send_ptr, &ekf_dt_);
  send_ptr = setToSendPkt(send_ptr, &pose_additional_delay_);
  send_ptr = setToSendPkt(send_ptr, &pose_rate_);
  send_ptr = setToSendPkt(send_ptr, &pose_gate_dist_);
  send_ptr = setToSendPkt(send_ptr, &twist_additional_delay_);
  send_ptr = setToSendPkt(send_ptr, &twist_rate_);
  send_ptr = setToSendPkt(send_ptr, &twist_gate_dist_);
  send_ptr = setToSendPkt(send_ptr, &proc_cov_yaw_d_);
  send_ptr = setToSendPkt(send_ptr, &proc_cov_yaw_bias_d_);
  send_ptr = setToSendPkt(send_ptr, &proc_cov_vx_d_);
  send_ptr = setToSendPkt(send_ptr, &proc_cov_wz_d_);

  /* send and receive packet */
  /* when receiving, synchronize the timing with EKF Accelerator */
  if (!communicateWithEKFAccelerator(SEND_PKT_LEN, RECV_PKT_LEN_EMPTY)) {
    return;
  }

  /* debug */
  pub_debug_ = create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>("debug", 1);
  pub_measured_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);
}

/*
 * timerCallback
 */
void PoseEstimatorHornet::timerCallback()
{
  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* create send packet */
  char* send_ptr = &send_buf_[0];
  char exec_mode = (char)EXEC_MODE::CALC_EKF;
  unsigned long t_curr = this->now().nanoseconds();
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &t_curr);

  /* send and receive packet */
  if (!communicateWithEKFAccelerator(SEND_PKT_LEN, RECV_PKT_LEN_EKFCALC)) {
    return;
  }

  /* parse receive packet */
  char* recv_ptr = &recv_buf_[0];
  recv_ptr = getFromRecvPkt(&x_(IDX::X), recv_ptr);
  recv_ptr = getFromRecvPkt(&x_(IDX::Y), recv_ptr);
  recv_ptr = getFromRecvPkt(&x_(IDX::YAW), recv_ptr);
  recv_ptr = getFromRecvPkt(&x_(IDX::YAWB), recv_ptr);
  recv_ptr = getFromRecvPkt(&x_(IDX::VX), recv_ptr);
  recv_ptr = getFromRecvPkt(&x_(IDX::WZ), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::X, IDX::X), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::X, IDX::Y), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::X, IDX::YAW), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::Y, IDX::X), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::Y, IDX::Y), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::Y, IDX::YAW), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::YAW, IDX::X), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::YAW, IDX::Y), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::YAW, IDX::YAW), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::VX, IDX::VX), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::VX, IDX::WZ), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::WZ, IDX::VX), recv_ptr);
  recv_ptr = getFromRecvPkt(&P_(IDX::WZ, IDX::WZ), recv_ptr);

  /* set current pose, twist */
  setCurrentResult();

  /* publish ekf result */
  publishEstimateResult();
}

void PoseEstimatorHornet::showCurrentX()
{
  if (show_debug_info_) {
    DEBUG_PRINT_MAT(x_.transpose());
  }
}

/*
 * setCurrentResult
 */
void PoseEstimatorHornet::setCurrentResult()
{
  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = x_(IDX::X);
  current_ekf_pose_.pose.position.y = x_(IDX::Y);

  tf2::Quaternion q_tf;
  double roll = 0.0, pitch = 0.0;
  if (current_pose_ptr_ != nullptr) {
    current_ekf_pose_.pose.position.z = current_pose_ptr_->pose.position.z;
    tf2::fromMsg(current_pose_ptr_->pose.orientation, q_tf); /* use Pose pitch and roll */
    double yaw_tmp;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_tmp);
  }
  double yaw = x_(IDX::YAW) + x_(IDX::YAWB);
  current_ekf_pose_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);

  current_ekf_pose_no_yawbias_ = current_ekf_pose_;
  current_ekf_pose_no_yawbias_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, x_(IDX::YAW));

  current_ekf_twist_.header.frame_id = "base_link";
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = x_(IDX::VX);
  current_ekf_twist_.twist.angular.z = x_(IDX::WZ);
}

/*
 * timerTFCallback
 */
void PoseEstimatorHornet::timerTFCallback()
{
  if (current_ekf_pose_.header.frame_id == "") {
    return;
  }

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = current_ekf_pose_.header.frame_id;
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = current_ekf_pose_.pose.position.x;
  transformStamped.transform.translation.y = current_ekf_pose_.pose.position.y;
  transformStamped.transform.translation.z = current_ekf_pose_.pose.position.z;

  transformStamped.transform.rotation.x = current_ekf_pose_.pose.orientation.x;
  transformStamped.transform.rotation.y = current_ekf_pose_.pose.orientation.y;
  transformStamped.transform.rotation.z = current_ekf_pose_.pose.orientation.z;
  transformStamped.transform.rotation.w = current_ekf_pose_.pose.orientation.w;

  tf_br_->sendTransform(transformStamped);
}

/*
 * getTransformFromTF
 */
bool PoseEstimatorHornet::getTransformFromTF(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  if (parent_frame.front() == '/') {
    parent_frame.erase(0, 1);
  }
  if (child_frame.front() == '/') {
    child_frame.erase(0, 1);
  }

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return false;
}

/*
 * callbackInitialPose
 */
void PoseEstimatorHornet::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!getTransformFromTF(pose_frame_id_, initialpose->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
      initialpose->header.frame_id.c_str());
  }

  current_ekf_pose_.pose.position.z =
    initialpose->pose.pose.position.z + transform.transform.translation.z;
  current_pose_ptr_ = nullptr;

  /* create send packet */
  char* send_ptr = &send_buf_[0];
  char exec_mode = (char)EXEC_MODE::SET_INITIAL_POSE;
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.pose.position.x));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.pose.position.y));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.pose.orientation.x));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.pose.orientation.y));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.pose.orientation.z));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.pose.orientation.w));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.covariance[0]));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.covariance[6 + 1]));
  send_ptr = setToSendPkt(send_ptr, &(initialpose->pose.covariance[6 * 5 + 5]));
  send_ptr = setToSendPkt(send_ptr, &(transform.transform.translation.x));
  send_ptr = setToSendPkt(send_ptr, &(transform.transform.translation.y));
  send_ptr = setToSendPkt(send_ptr, &(transform.transform.rotation.x));
  send_ptr = setToSendPkt(send_ptr, &(transform.transform.rotation.y));
  send_ptr = setToSendPkt(send_ptr, &(transform.transform.rotation.z));
  send_ptr = setToSendPkt(send_ptr, &(transform.transform.rotation.w));

  /* send and receive packet */
  /* when receiving, synchronize the timing with EKF Accelerator */
  (void)communicateWithEKFAccelerator(SEND_PKT_LEN, RECV_PKT_LEN_EMPTY);
}

/*
 * callbackPoseWithCovariance
 */
void PoseEstimatorHornet::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
  current_pose_covariance_ = msg->pose.covariance;

  /* create send packet */
  char* send_ptr = &send_buf_[0];
  char exec_mode = (char)EXEC_MODE::SET_POSE_WITH_COVARIANCE;
  unsigned long current_pose_ptr_header_stamp = 
    convSecToNanosec(current_pose_ptr_->header.stamp.sec) + current_pose_ptr_->header.stamp.nanosec;
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &current_pose_ptr_header_stamp);
  send_ptr = setToSendPkt(send_ptr, &(current_pose_ptr_->pose.orientation.x));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_ptr_->pose.orientation.y));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_ptr_->pose.orientation.z));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_ptr_->pose.orientation.w));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_ptr_->pose.position.x));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_ptr_->pose.position.y));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(0)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(1)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(5)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(6)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(7)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(11)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(30)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(31)));
  send_ptr = setToSendPkt(send_ptr, &(current_pose_covariance_.at(35)));

  /* send and receive packet */
  /* when receiving, synchronize the timing with EKF Accelerator */
  (void)communicateWithEKFAccelerator(SEND_PKT_LEN, RECV_PKT_LEN_EMPTY);
}

/*
 * callbackTwistWithCovariance
 */
void PoseEstimatorHornet::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(twist);
  current_twist_covariance_ = msg->twist.covariance;

  /* create send packet */
  char* send_ptr = &send_buf_[0];
  char exec_mode = (char)EXEC_MODE::SET_TWIST_WITH_COVARIANCE;
  unsigned long current_twist_ptr_header_stamp = 
    convSecToNanosec(current_twist_ptr_->header.stamp.sec) + current_twist_ptr_->header.stamp.nanosec;
  send_ptr = setToSendPkt(send_ptr, &exec_mode);
  send_ptr = setToSendPkt(send_ptr, &current_twist_ptr_header_stamp);
  send_ptr = setToSendPkt(send_ptr, &(current_twist_ptr_->twist.linear.x));
  send_ptr = setToSendPkt(send_ptr, &(current_twist_ptr_->twist.angular.z));
  send_ptr = setToSendPkt(send_ptr, &(current_twist_covariance_.at(0)));
  send_ptr = setToSendPkt(send_ptr, &(current_twist_covariance_.at(5)));
  send_ptr = setToSendPkt(send_ptr, &(current_twist_covariance_.at(30)));
  send_ptr = setToSendPkt(send_ptr, &(current_twist_covariance_.at(35)));

  /* send and receive packet */
  /* when receiving, synchronize the timing with EKF Accelerator */
  (void)communicateWithEKFAccelerator(SEND_PKT_LEN, RECV_PKT_LEN_EMPTY);
}

/*
 * publishEstimateResult
 */
void PoseEstimatorHornet::publishEstimateResult()
{
  rclcpp::Time current_time = this->now();

  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose_);
  pub_pose_no_yawbias_->publish(current_ekf_pose_no_yawbias_);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose_.pose;
  pose_cov.pose.covariance[0] = P_(IDX::X, IDX::X);
  pose_cov.pose.covariance[1] = P_(IDX::X, IDX::Y);
  pose_cov.pose.covariance[5] = P_(IDX::X, IDX::YAW);
  pose_cov.pose.covariance[6] = P_(IDX::Y, IDX::X);
  pose_cov.pose.covariance[7] = P_(IDX::Y, IDX::Y);
  pose_cov.pose.covariance[11] = P_(IDX::Y, IDX::YAW);
  pose_cov.pose.covariance[30] = P_(IDX::YAW, IDX::X);
  pose_cov.pose.covariance[31] = P_(IDX::YAW, IDX::Y);
  pose_cov.pose.covariance[35] = P_(IDX::YAW, IDX::YAW);
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_no_yawbias = pose_cov;
  pose_cov_no_yawbias.pose.pose = current_ekf_pose_no_yawbias_.pose;
  pub_pose_cov_no_yawbias_->publish(pose_cov_no_yawbias);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist_);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist_.twist;
  twist_cov.twist.covariance[0] = P_(IDX::VX, IDX::VX);
  twist_cov.twist.covariance[5] = P_(IDX::VX, IDX::WZ);
  twist_cov.twist.covariance[30] = P_(IDX::WZ, IDX::VX);
  twist_cov.twist.covariance[35] = P_(IDX::WZ, IDX::WZ);
  pub_twist_cov_->publish(twist_cov);

  /* publish yaw bias */
  tier4_debug_msgs::msg::Float64Stamped yawb;
  yawb.stamp = current_time;
  yawb.data = x_(IDX::YAWB);
  pub_yaw_bias_->publish(yawb);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = current_ekf_pose_.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);

  /* debug measured pose */
  if (current_pose_ptr_ != nullptr) {
    geometry_msgs::msg::PoseStamped p;
    p = *current_pose_ptr_;
    p.header.stamp = current_time;
    pub_measured_pose_->publish(p);
  }

  /* debug publish */
  double pose_yaw = 0.0;
  if (current_pose_ptr_ != nullptr) {
    pose_yaw = tf2::getYaw(current_pose_ptr_->pose.orientation);
  }

  tier4_debug_msgs::msg::Float64MultiArrayStamped msg;
  msg.stamp = current_time;
  msg.data.push_back(tier4_autoware_utils::rad2deg(x_(IDX::YAW)));   // [0] ekf yaw angle
  msg.data.push_back(tier4_autoware_utils::rad2deg(pose_yaw));      // [1] measurement yaw angle
  msg.data.push_back(tier4_autoware_utils::rad2deg(x_(IDX::YAWB)));  // [2] yaw bias
  pub_debug_->publish(msg);
}

/*
 * setToSendPkt
 */
template<typename T>
inline char* PoseEstimatorHornet::setToSendPkt(char* outptr, const T* inptr)
{
  std::memcpy(outptr, (char*)inptr, sizeof(T));
  return (outptr + sizeof(T));
}

/*
 * getFromRecvPkt
 */
template<typename T>
inline char* PoseEstimatorHornet::getFromRecvPkt(T* outptr, const char* inptr)
{
  std::memcpy((char*)outptr, (char*)inptr, sizeof(T));
  return ((char*)inptr + sizeof(T));
}

/*
 * convSecToNanosec
 */
inline unsigned long PoseEstimatorHornet::convSecToNanosec(unsigned int sec)
{
  return (unsigned long)sec * 1000U * 1000U * 1000U;
}

/*
 * communicateWithEKFAccelerator
 */
bool PoseEstimatorHornet::communicateWithEKFAccelerator(
  const unsigned int send_pkt_len, const unsigned int recv_pkt_len)
{
  /* handshake TCP/IP communication */
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0){
    RCLCPP_ERROR(get_logger(), "[EKF] TCP/IP socket error : %s", std::strerror(errno));
    return false;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(struct sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORTNO);
  addr.sin_addr.s_addr = inet_addr(IPADDR);

  int ret = connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[EKF] TCP/IP connect error : %s", std::strerror(errno));
    close(sockfd);
    return false;
  }

  /* send message */
  ret = send(sockfd, &send_buf_[0], send_pkt_len, 0);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[EKF] TCP/IP send error : %s", std::strerror(errno));
    close(sockfd);
    return false;
  }

  /* receive message */
  ret = recv(sockfd, &recv_buf_[0], recv_pkt_len, MSG_WAITALL);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "[EKF] TCP/IP receive error : %s", std::strerror(errno));
    close(sockfd);
    return false;
  }

  close(sockfd);

  return true;
}

