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

#include "lx16a/lx16a_consts.hpp"
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
}

void ControllerNode::publish_odometry() {

  auto now = this->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;
  if (dt <= 0.0)
    return;

  // Convert raw duty to m/s; right-side motors are mirrored so negate them
  double scale = speed_max_ms_ / static_cast<double>(speed_max_raw_);

  double lf = motor_controller->get_motor_speed(lx16a::MOTOR_LEFT_FRONT) * scale;
  double lm = motor_controller->get_motor_speed(lx16a::MOTOR_LEFT_MIDDLE) * scale;
  double lb = motor_controller->get_motor_speed(lx16a::MOTOR_LEFT_BACK) * scale;
  double rf = motor_controller->get_motor_speed(lx16a::MOTOR_RIGHT_FRONT) * -scale;
  double rm = motor_controller->get_motor_speed(lx16a::MOTOR_RIGHT_MIDDLE) * -scale;
  double rb = motor_controller->get_motor_speed(lx16a::MOTOR_RIGHT_BACK) * -scale;

  double v_left = (lf + lm + lb) / 3.0;
  double v_right = (rf + rm + rb) / 3.0;

  double v = (v_left + v_right) / 2.0;
  double omega = (v_right - v_left) / wheel_base_m_;

  // Integrate pose
  theta_ += omega * dt;
  x_ += v * std::cos(theta_) * dt;
  y_ += v * std::sin(theta_) * dt;

  // Build and publish nav_msgs/Odometry
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = now;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0.0;

  // Yaw-only quaternion
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
  msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

  msg.twist.twist.linear.x = v;
  msg.twist.twist.angular.z = omega;

  odom_publisher->publish(msg);
}

void ControllerNode::shutdown() { this->motor_controller->kill_motors(); }