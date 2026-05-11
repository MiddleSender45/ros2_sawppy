#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cmath>

class MPU6500Node : public rclcpp::Node
{
public:
    MPU6500Node() : Node("rover_imu")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MPU6500Node::update, this));

        init_i2c();
        init_sensor();
    }

private:
    int file_ = -1;
    bool initialized_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_i2c()
    {
        const char *device = "/dev/i2c-1";
        file_ = open(device, O_RDWR);

        if (file_ < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open I2C device '%s'. Node will not publish IMU data.", device);
            return;
        }

        if (ioctl(file_, I2C_SLAVE, 0x68) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address.");
            close(file_);
            file_ = -1;
            return;
        }

        initialized_ = true;
    }

    void write_reg(uint8_t reg, uint8_t val)
    {
        uint8_t buffer[2] = {reg, val};
        write(file_, buffer, 2);
    }

    uint8_t read_reg(uint8_t reg)
    {
        write(file_, &reg, 1);
        uint8_t val;
        read(file_, &val, 1);
        return val;
    }

    int16_t read_word(uint8_t reg)
    {
        uint8_t high = read_reg(reg);
        uint8_t low  = read_reg(reg + 1);
        return (int16_t)((high << 8) | low);
    }

    void init_sensor()
    {
        if (!initialized_) return;

        // Wake up device
        write_reg(0x6B, 0x00);

        // Gyro config ±250 dps
        write_reg(0x1B, 0x00);

        // Accel config ±2g
        write_reg(0x1C, 0x00);
    }

    void update()
    {
        if (!initialized_) return;
        sensor_msgs::msg::Imu msg;

        // raw data
        int16_t ax = read_word(0x3B);
        int16_t ay = read_word(0x3D);
        int16_t az = read_word(0x3F);

        int16_t gx = read_word(0x43);
        int16_t gy = read_word(0x45);
        int16_t gz = read_word(0x47);

        // scale (basic, uncalibrated)
        double accel_scale = 9.81 / 16384.0;
        double gyro_scale  = (3.14159 / 180.0) / 131.0;

        msg.linear_acceleration.x = ax * accel_scale;
        msg.linear_acceleration.y = ay * accel_scale;
        msg.linear_acceleration.z = az * accel_scale;

        msg.angular_velocity.x = gx * gyro_scale;
        msg.angular_velocity.y = gy * gyro_scale;
        msg.angular_velocity.z = gz * gyro_scale;

        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6500Node>());
    rclcpp::shutdown();
    return 0;
}