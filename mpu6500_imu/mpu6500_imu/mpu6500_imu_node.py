#!/usr/bin/env python3
"""
ROS2 Jazzy IMU node for the MPU6500 (I2C).

Publishes:
  ~/imu/raw        sensor_msgs/Imu          (accel + gyro, no mag)
  ~/imu/temp       sensor_msgs/Temperature  (die temperature)

Parameters (all declared with defaults):
  i2c_bus         int    1       Linux I2C bus number (/dev/i2c-N)
  i2c_address     int    0x68    MPU6500 I2C address (0x68 or 0x69)
  frame_id        str   "imu_link"
  publish_rate    float  200.0   Hz
  accel_range     int    0       0=±2g 1=±4g 2=±8g 3=±16g
  gyro_range      int    0       0=±250 1=±500 2=±1000 3=±2000 dps
  dlpf_bandwidth  int    2       0-7, see MPU6500 datasheet Table 2
  calibrate       bool   true    run bias calibration on startup
  calib_samples   int    500     samples averaged during calibration
"""

import smbus2
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Header

import numpy as np


# ── Register map ──────────────────────────────────────────────────────────────
REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A   # DLPF config
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG2= 0x1D   # Accel DLPF (MPU6500-specific)
REG_INT_PIN_CFG  = 0x37
REG_INT_ENABLE   = 0x38
REG_ACCEL_XOUT_H = 0x3B   # 6 bytes accel, 2 temp, 6 gyro
REG_TEMP_OUT_H   = 0x41
REG_GYRO_XOUT_H  = 0x43
REG_PWR_MGMT_1   = 0x6B
REG_PWR_MGMT_2   = 0x6C
REG_WHO_AM_I     = 0x75

WHO_AM_I_MPU6500 = 0x70
WHO_AM_I_MPU6000 = 0x68   # also accepted

# Scale factors
ACCEL_SCALE = [2.0, 4.0, 8.0, 16.0]   # g per LSB range
GYRO_SCALE  = [250.0, 500.0, 1000.0, 2000.0]  # dps per LSB range
GRAVITY     = 9.80665                  # m/s²
DEG_TO_RAD  = np.pi / 180.0


class MPU6500Node(Node):

    def __init__(self):
        super().__init__('mpu6500_imu_node')

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter('i2c_bus',        1)
        self.declare_parameter('i2c_address',    0x68)
        self.declare_parameter('frame_id',       'imu_link')
        self.declare_parameter('publish_rate',   200.0)
        self.declare_parameter('accel_range',    0,
            ParameterDescriptor(description='0=±2g 1=±4g 2=±8g 3=±16g',
                                integer_range=[IntegerRange(from_value=0, to_value=3, step=1)]))
        self.declare_parameter('gyro_range',     0,
            ParameterDescriptor(description='0=±250 1=±500 2=±1000 3=±2000 dps',
                                integer_range=[IntegerRange(from_value=0, to_value=3, step=1)]))
        self.declare_parameter('dlpf_bandwidth', 2,
            ParameterDescriptor(description='0-7 per MPU6500 datasheet Table 2',
                                integer_range=[IntegerRange(from_value=0, to_value=7, step=1)]))
        self.declare_parameter('calibrate',      True)
        self.declare_parameter('calib_samples',  500)

        # ── Read parameters ───────────────────────────────────────────────────
        self.bus_num    = self.get_parameter('i2c_bus').value
        self.address    = self.get_parameter('i2c_address').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.rate_hz    = self.get_parameter('publish_rate').value
        self.accel_rng  = self.get_parameter('accel_range').value
        self.gyro_rng   = self.get_parameter('gyro_range').value
        self.dlpf       = self.get_parameter('dlpf_bandwidth').value
        self.do_calib   = self.get_parameter('calibrate').value
        self.n_calib    = self.get_parameter('calib_samples').value

        # Derived scale factors (raw int16 → SI)
        self.accel_lsb  = ACCEL_SCALE[self.accel_rng] * GRAVITY / 32768.0
        self.gyro_lsb   = GYRO_SCALE[self.gyro_rng] * DEG_TO_RAD / 32768.0

        # Bias offsets (filled by calibration)
        self.accel_bias = np.zeros(3)
        self.gyro_bias  = np.zeros(3)

        # ── Open I2C bus and configure sensor ────────────────────────────────
        try:
            self.bus = smbus2.SMBus(self.bus_num)
        except Exception as e:
            self.get_logger().fatal(f'Cannot open I2C bus {self.bus_num}: {e}')
            raise

        self._init_sensor()

        if self.do_calib:
            self._calibrate()

        # ── Publishers ────────────────────────────────────────────────────────
        self.imu_pub  = self.create_publisher(Imu,         '~/imu/raw',  10)
        self.temp_pub = self.create_publisher(Temperature, '~/imu/temp', 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f'MPU6500 node ready  bus={self.bus_num}  addr=0x{self.address:02X}  '
            f'accel=±{ACCEL_SCALE[self.accel_rng]}g  '
            f'gyro=±{GYRO_SCALE[self.gyro_rng]}dps  '
            f'rate={self.rate_hz}Hz'
        )

    # ── Sensor initialisation ─────────────────────────────────────────────────

    def _write(self, reg: int, val: int) -> None:
        self.bus.write_byte_data(self.address, reg, val)

    def _read_bytes(self, reg: int, length: int) -> bytes:
        return bytes(self.bus.read_i2c_block_data(self.address, reg, length))

    def _init_sensor(self) -> None:
        # Verify identity
        who = self.bus.read_byte_data(self.address, REG_WHO_AM_I)
        if who not in (WHO_AM_I_MPU6500, WHO_AM_I_MPU6000):
            self.get_logger().warn(
                f'Unexpected WHO_AM_I=0x{who:02X} (expected 0x70). Continuing anyway.')

        # Reset device, then wake with PLL clock auto-select
        self._write(REG_PWR_MGMT_1, 0x80)   # H_RESET
        time.sleep(0.1)
        self._write(REG_PWR_MGMT_1, 0x01)   # CLKSEL=1 (PLL)
        self._write(REG_PWR_MGMT_2, 0x00)   # enable all axes

        # DLPF + sample rate
        # With DLPF enabled the gyro output rate is 1 kHz.
        # SMPLRT_DIV = (1000 / desired_rate) - 1  (capped at 255)
        smplrt = max(0, min(255, int(1000.0 / self.rate_hz) - 1))
        self._write(REG_SMPLRT_DIV,   smplrt)
        self._write(REG_CONFIG,        self.dlpf & 0x07)
        self._write(REG_ACCEL_CONFIG2, self.dlpf & 0x07)

        # Full-scale ranges
        self._write(REG_GYRO_CONFIG,   (self.gyro_rng  & 0x03) << 3)
        self._write(REG_ACCEL_CONFIG,  (self.accel_rng & 0x03) << 3)

        # Disable FSYNC, enable I2C bypass (not strictly needed for I2C-only)
        self._write(REG_INT_PIN_CFG, 0x02)
        self._write(REG_INT_ENABLE,  0x00)

        time.sleep(0.05)

    # ── Calibration ───────────────────────────────────────────────────────────

    def _read_raw_all(self):
        """Return (accel_xyz, gyro_xyz) as raw int16 arrays."""
        data = self._read_bytes(REG_ACCEL_XOUT_H, 14)
        ax, ay, az, _, gx, gy, gz = struct.unpack('>7h', data)
        return np.array([ax, ay, az], dtype=float), np.array([gx, gy, gz], dtype=float)

    def _calibrate(self) -> None:
        self.get_logger().info(
            f'Calibrating over {self.n_calib} samples — keep sensor still …')

        accel_sum = np.zeros(3)
        gyro_sum  = np.zeros(3)
        for _ in range(self.n_calib):
            a, g = self._read_raw_all()
            accel_sum += a
            gyro_sum  += g
            time.sleep(1.0 / self.rate_hz)

        accel_mean = accel_sum / self.n_calib
        gyro_mean  = gyro_sum  / self.n_calib

        # Remove gravity from Z axis (assuming sensor flat, Z pointing up)
        gravity_lsb = GRAVITY / self.accel_lsb
        accel_mean[2] -= gravity_lsb

        self.accel_bias = accel_mean
        self.gyro_bias  = gyro_mean

        self.get_logger().info(
            f'Calibration done.  '
            f'accel_bias={self.accel_bias * self.accel_lsb}  '
            f'gyro_bias={self.gyro_bias * self.gyro_lsb * 1/DEG_TO_RAD} dps'
        )

    # ── Publish callback ──────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        try:
            data = self._read_bytes(REG_ACCEL_XOUT_H, 14)
        except Exception as e:
            self.get_logger().error(f'I2C read error: {e}', throttle_duration_sec=1.0)
            return

        ax, ay, az, temp_raw, gx, gy, gz = struct.unpack('>7h', data)

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.frame_id)

        # ── IMU message ───────────────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header = header

        # Apply bias and convert to SI
        imu_msg.linear_acceleration.x = (ax - self.accel_bias[0]) * self.accel_lsb
        imu_msg.linear_acceleration.y = (ay - self.accel_bias[1]) * self.accel_lsb
        imu_msg.linear_acceleration.z = (az - self.accel_bias[2]) * self.accel_lsb

        imu_msg.angular_velocity.x = (gx - self.gyro_bias[0]) * self.gyro_lsb
        imu_msg.angular_velocity.y = (gy - self.gyro_bias[1]) * self.gyro_lsb
        imu_msg.angular_velocity.z = (gz - self.gyro_bias[2]) * self.gyro_lsb

        # Orientation unknown — set covariance[0] = -1 per REP-145
        imu_msg.orientation_covariance[0] = -1.0

        # Diagonal covariance estimates (tune to your sensor + DLPF setting)
        accel_var = (0.05 * GRAVITY) ** 2   # ~50 mg noise std dev
        gyro_var  = (0.01 * DEG_TO_RAD) ** 2
        for i in (0, 4, 8):
            imu_msg.linear_acceleration_covariance[i] = accel_var
            imu_msg.angular_velocity_covariance[i]    = gyro_var

        self.imu_pub.publish(imu_msg)

        # ── Temperature message ───────────────────────────────────────────────
        # Formula from MPU6500 datasheet §4.18:  T(°C) = (TEMP_OUT / 333.87) + 21.0
        temp_msg = Temperature()
        temp_msg.header = header
        temp_msg.temperature = temp_raw / 333.87 + 21.0
        temp_msg.variance    = 0.0   # unknown
        self.temp_pub.publish(temp_msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPU6500Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()