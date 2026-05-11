// ============================================================================
//  rover_motion – imu_node.cpp
//
//  MPU-9250 driver + ROS 2 Jazzy node.
//  Publishes: /imu/data  (sensor_msgs/msg/Imu)
//             /imu/temp  (sensor_msgs/msg/Temperature)   [optional]
//
//  Topic frame_id defaults to "imu_link" and is configurable via parameter.
// ============================================================================

#include "rover_motion/mpu9250.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

// ────────────────────────────────────────────────────────────────────────────
//  MPU9250 driver implementation
// ────────────────────────────────────────────────────────────────────────────
namespace mpu9250
{

MPU9250::MPU9250(const std::string & i2c_bus, int i2c_addr)
: bus_path_(i2c_bus), addr_(i2c_addr)
{}

MPU9250::~MPU9250()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

void MPU9250::init()
{
  // Open bus
  fd_ = open(bus_path_.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Cannot open I²C bus: " + bus_path_);
  }

  // Set slave address
  if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
    throw std::runtime_error("Cannot set I²C slave address");
  }

  // WHO_AM_I check
  uint8_t who = read_reg(REG_WHO_AM_I);
  if (who != WHO_AM_I_VAL) {
    throw std::runtime_error(
      "MPU-9250 WHO_AM_I mismatch: expected 0x71, got 0x" +
      std::to_string(static_cast<int>(who)));
  }

  // Wake the device (clear sleep bit), use PLL gyro clock
  write_reg(REG_PWR_MGMT_1, 0x01);
  usleep(100'000);  // 100 ms stabilisation

  // Accel: ±2 g  (bits [4:3] = 00)
  write_reg(REG_ACCEL_CONFIG, 0x00);

  // Gyro: ±250 °/s  (bits [4:3] = 00)
  write_reg(REG_GYRO_CONFIG, 0x00);
}

ImuData MPU9250::read()
{
  ImuData d;

  d.accel_x = read_word(REG_ACCEL_XOUT_H)     * ACCEL_SCALE_MPS2;
  d.accel_y = read_word(REG_ACCEL_XOUT_H + 2) * ACCEL_SCALE_MPS2;
  d.accel_z = read_word(REG_ACCEL_XOUT_H + 4) * ACCEL_SCALE_MPS2;

  d.gyro_x  = read_word(REG_GYRO_XOUT_H)      * GYRO_SCALE_RADPS;
  d.gyro_y  = read_word(REG_GYRO_XOUT_H + 2)  * GYRO_SCALE_RADPS;
  d.gyro_z  = read_word(REG_GYRO_XOUT_H + 4)  * GYRO_SCALE_RADPS;

  // Temperature: T(°C) = (raw / 333.87) + 21.0  (datasheet formula)
  int16_t raw_temp = read_word(REG_TEMP_OUT_H);
  d.temperature = static_cast<double>(raw_temp) / 333.87 + 21.0;

  return d;
}

// ── Private helpers ──────────────────────────────────────────────────────────

void MPU9250::write_reg(uint8_t reg, uint8_t value)
{
  if (i2c_smbus_write_byte_data(fd_, reg, value) < 0) {
    throw std::runtime_error("I²C write failed for register 0x" +
                             std::to_string(static_cast<int>(reg)));
  }
}

uint8_t MPU9250::read_reg(uint8_t reg)
{
  int val = i2c_smbus_read_byte_data(fd_, reg);
  if (val < 0) {
    throw std::runtime_error("I²C read failed for register 0x" +
                             std::to_string(static_cast<int>(reg)));
  }
  return static_cast<uint8_t>(val);
}

int16_t MPU9250::read_word(uint8_t reg_high)
{
  uint8_t hi = read_reg(reg_high);
  uint8_t lo = read_reg(static_cast<uint8_t>(reg_high + 1));
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

}  // namespace mpu9250

// ────────────────────────────────────────────────────────────────────────────
//  ROS 2 Node
// ────────────────────────────────────────────────────────────────────────────

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("imu_node")
  {
    // ── Parameters ───────────────────────────────────────────────────────────
    declare_parameter<std::string>("i2c_bus",    "/dev/i2c-1");
    declare_parameter<int>        ("i2c_addr",   0x68);
    declare_parameter<std::string>("frame_id",   "imu_link");
    declare_parameter<double>     ("frequency",  50.0);   // Hz
    declare_parameter<bool>       ("publish_temp", true);

    auto bus    = get_parameter("i2c_bus").as_string();
    auto addr   = static_cast<uint8_t>(get_parameter("i2c_addr").as_int());
    frame_id_   = get_parameter("frame_id").as_string();
    double freq = get_parameter("frequency").as_double();
    publish_temp_ = get_parameter("publish_temp").as_bool();

    RCLCPP_INFO(get_logger(), "IMU bus=%s  addr=0x%02X  frame=%s  %.1f Hz",
      bus.c_str(), addr, frame_id_.c_str(), freq);

    // ── Driver init ──────────────────────────────────────────────────────────
    imu_ = std::make_unique<mpu9250::MPU9250>(bus, addr);
    imu_->init();
    RCLCPP_INFO(get_logger(), "MPU-9250 initialised");

    // ── Publishers ───────────────────────────────────────────────────────────
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    if (publish_temp_) {
      temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/imu/temp", 10);
    }

    // ── Timer ────────────────────────────────────────────────────────────────
    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / freq));

    timer_ = create_wall_timer(period_ms, [this]() { publish_imu(); });
  }

private:
  void publish_imu()
  {
    mpu9250::ImuData data;
    try {
      data = imu_->read();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "IMU read error: %s", e.what());
      return;
    }

    auto stamp = now();

    // ── sensor_msgs/Imu ──────────────────────────────────────────────────────
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp    = stamp;
    imu_msg.header.frame_id = frame_id_;

    imu_msg.linear_acceleration.x = data.accel_x;
    imu_msg.linear_acceleration.y = data.accel_y;
    imu_msg.linear_acceleration.z = data.accel_z;

    imu_msg.angular_velocity.x = data.gyro_x;
    imu_msg.angular_velocity.y = data.gyro_y;
    imu_msg.angular_velocity.z = data.gyro_z;

    // Orientation unknown – fill covariance[0] = -1 per REP-145
    imu_msg.orientation_covariance[0] = -1.0;

    // Conservative diagonal covariances (tune with real noise measurements)
    // Accel: ~0.002 (m/s²)²   Gyro: ~0.0001 (rad/s)²
    imu_msg.linear_acceleration_covariance[0] = 0.002;
    imu_msg.linear_acceleration_covariance[4] = 0.002;
    imu_msg.linear_acceleration_covariance[8] = 0.002;

    imu_msg.angular_velocity_covariance[0] = 0.0001;
    imu_msg.angular_velocity_covariance[4] = 0.0001;
    imu_msg.angular_velocity_covariance[8] = 0.0001;

    imu_pub_->publish(imu_msg);

    // ── sensor_msgs/Temperature ──────────────────────────────────────────────
    if (publish_temp_ && temp_pub_) {
      sensor_msgs::msg::Temperature temp_msg;
      temp_msg.header.stamp    = stamp;
      temp_msg.header.frame_id = frame_id_;
      temp_msg.temperature     = data.temperature;
      temp_msg.variance        = 0.0;
      temp_pub_->publish(temp_msg);
    }
  }

  // driver
  std::unique_ptr<mpu9250::MPU9250> imu_;

  // ros
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  bool        publish_temp_{true};
};

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
