// MIT License

// Copyright (c) 2023 Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "lx16a/motor_controller.hpp"
#include "motor_controller/controller_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rover_msgs/msg/motors_command.hpp"

using std::placeholders::_1;
using namespace motor_controller;

ControllerNode::ControllerNode() : rclcpp::Node("controller_node") {

  // declaring params
  this->declare_parameter<std::string>("motor_controller_device",
                                       "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);

  // getting params
  std::string motor_controller_device;
  this->get_parameter("motor_controller_device", motor_controller_device);

  int baud_rate;
  this->get_parameter("baud_rate", baud_rate);

  // Check if the device exists and is accessible
  struct stat device_stat;
  if (stat(motor_controller_device.c_str(), &device_stat) != 0) {
    std::string error_msg = "Motor controller device '" +
                            motor_controller_device +
                            "' does not exist. Please check if the device is "
                            "connected and the path is correct.";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Check if we have read/write permissions
  if (access(motor_controller_device.c_str(), R_OK | W_OK) != 0) {
    std::string error_msg =
        "No read/write permission for motor controller device '" +
        motor_controller_device +
        "'. Please check device permissions or add user to dialout group.";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Check if it's a character device (serial devices are character devices)
  if (!S_ISCHR(device_stat.st_mode)) {
    std::string error_msg =
        "Motor controller device '" + motor_controller_device +
        "' is not a character device. Expected a serial device.";
    RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  RCLCPP_INFO(this->get_logger(),
              "Motor controller device '%s' found and accessible",
              motor_controller_device.c_str());

  this->motor_controller = std::make_unique<lx16a::MotorController>(
      lx16a::MotorController(motor_controller_device, baud_rate));

  // Odometry geometry params
  this->declare_parameter<double>("wheel_radius_m", 0.065);
  this->declare_parameter<double>("wheel_base_m", 0.260);
  this->declare_parameter<int>("speed_max_raw", 1000);
  this->declare_parameter<double>("speed_max_ms", 0.50);

  this->get_parameter("wheel_radius_m", wheel_radius_m_);
  this->get_parameter("wheel_base_m", wheel_base_m_);
  this->get_parameter("speed_max_raw", speed_max_raw_);
  this->get_parameter("speed_max_ms", speed_max_ms_);

  // Initialise accumulated pose
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  last_time_ = this->now();

  // Initialise commanded speeds to zero (6 drive motors)
  last_drive_commands_ = {0, 0, 0, 0, 0, 0};

  // Odometry publisher
  odom_publisher =
      this->create_publisher<nav_msgs::msg::Odometry>("odom_wheel", 10);

  // 20 Hz odometry timer
  odom_timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ControllerNode::publish_odometry, this));

  // sub
  this->subscription =
      this->create_subscription<rover_msgs::msg::MotorsCommand>(
          "motors_command", 10, std::bind(&ControllerNode::callback, this, _1));
}

void ControllerNode::callback(
    const rover_msgs::msg::MotorsCommand::SharedPtr msg) {
  this->motor_controller->corner_to_position(msg->corner_motor);
  this->motor_controller->send_motor_duty(msg->drive_motor);
  // Cache commanded duties for odometry (avoids blocking serial read-back)
  last_drive_commands_ = msg->drive_motor;
}

void ControllerNode::publish_odometry() {

  auto now = this->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;
  if (dt <= 0.0)
    return;

  // Use last commanded duty values — avoids any blocking serial read-back.
  // The LX16A wait_for_response() is a while(true) loop; calling it from a
  // timer callback would hang if the bus is idle between drive commands.
  // Left-side: index 0-2 (positive duty = forward)
  // Right-side: index 3-5 (positive duty = backward due to mirrored mount)
  double scale = speed_max_ms_ / static_cast<double>(speed_max_raw_);

  double v_left  = (last_drive_commands_[0] +
                    last_drive_commands_[1] +
                    last_drive_commands_[2]) / 3.0 * scale;
  double v_right = (last_drive_commands_[3] +
                    last_drive_commands_[4] +
                    last_drive_commands_[5]) / 3.0 * -scale;

  double v     = (v_left + v_right) / 2.0;
  double omega = (v_right - v_left) / wheel_base_m_;

  // Integrate pose (simple Euler)
  theta_ += omega * dt;
  x_     += v * std::cos(theta_) * dt;
  y_     += v * std::sin(theta_) * dt;

  // --- Build nav_msgs/Odometry (standard nav-stack format) ---
  nav_msgs::msg::Odometry msg;
  msg.header.stamp    = now;
  msg.header.frame_id = "odom";
  msg.child_frame_id  = "base_link";

  // Pose in the odom frame
  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0.0;

  // Yaw-only quaternion (planar rover)
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
  msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

  // Pose covariance (6x6 row-major).  Diagonal: [x, y, z, roll, pitch, yaw]
  // z/roll/pitch are 0 for a planar rover (set near-zero, not exactly 0,
  // so EKF filters don't treat them as perfectly known).
  msg.pose.covariance[0]  = 1e-3;   // x
  msg.pose.covariance[7]  = 1e-3;   // y
  msg.pose.covariance[14] = 1e-9;   // z  (constrained to ground)
  msg.pose.covariance[21] = 1e-9;   // roll
  msg.pose.covariance[28] = 1e-9;   // pitch
  msg.pose.covariance[35] = 1e-3;   // yaw

  // Twist in the child (base_link) frame
  msg.twist.twist.linear.x  = v;
  msg.twist.twist.linear.y  = 0.0;
  msg.twist.twist.linear.z  = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = omega;

  // Twist covariance (6x6 row-major).  Diagonal: [vx, vy, vz, wx, wy, wz]
  msg.twist.covariance[0]  = 1e-3;  // vx
  msg.twist.covariance[7]  = 1e-9;  // vy (non-holonomic: near-zero)
  msg.twist.covariance[14] = 1e-9;  // vz
  msg.twist.covariance[21] = 1e-9;  // wx
  msg.twist.covariance[28] = 1e-9;  // wy
  msg.twist.covariance[35] = 1e-3;  // wz

  odom_publisher->publish(msg);
}

void ControllerNode::shutdown() { this->motor_controller->kill_motors(); }