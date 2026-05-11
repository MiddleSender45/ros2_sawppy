#pragma once

#include <cstdint>
#include <string>
#include <stdexcept>

// ────────────────────────────────────────────────────────────────────────────
//  MPU-9250 register map (subset used by this driver)
// ────────────────────────────────────────────────────────────────────────────
namespace mpu9250
{

// I²C default address (AD0 = LOW)
constexpr uint8_t DEFAULT_I2C_ADDR = 0x68;

// Power / clock
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2   = 0x6C;

// Accelerometer full-scale config
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;

// Gyroscope full-scale config
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;

// Sensor data – 6 bytes each, big-endian
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_GYRO_XOUT_H  = 0x43;
constexpr uint8_t REG_TEMP_OUT_H   = 0x41;

// WHO_AM_I: should read 0x71 for MPU-9250
constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr uint8_t WHO_AM_I_VAL     = 0x71;

// ────────────────────────────────────────────────────────────────────────────
//  Scale factors
// ────────────────────────────────────────────────────────────────────────────

// Accel full-scale = ±2 g  →  16384 LSB/g  →  ×9.80665 m/s²
constexpr double ACCEL_SCALE_MPS2   = 9.80665 / 16384.0;

// Gyro full-scale  = ±250 °/s  →  131 LSB/(°/s)  →  ×π/180 rad/s
constexpr double GYRO_SCALE_RADPS   = (1.0 / 131.0) * (3.14159265358979323846 / 180.0);

// ────────────────────────────────────────────────────────────────────────────
//  Driver class
// ────────────────────────────────────────────────────────────────────────────

struct ImuData
{
  // m/s²
  double accel_x{0}, accel_y{0}, accel_z{0};
  // rad/s
  double gyro_x{0}, gyro_y{0}, gyro_z{0};
  // °C
  double temperature{0};
};

class MPU9250
{
public:
  /**
   * @param i2c_bus   I²C bus device path, e.g. "/dev/i2c-1"
   * @param i2c_addr  Sensor I²C address (default 0x68)
   */
  explicit MPU9250(const std::string & i2c_bus = "/dev/i2c-1",
                   int i2c_addr = DEFAULT_I2C_ADDR);

  ~MPU9250();

  /** Initialise the sensor (wake-up + self-check). Throws on failure. */
  void init();

  /** Read one sample from the sensor. Throws on I²C error. */
  ImuData read();

private:
  std::string bus_path_;
  int         addr_;
  int         fd_{-1};

  void     write_reg(uint8_t reg, uint8_t value);
  uint8_t  read_reg(uint8_t reg);
  int16_t  read_word(uint8_t reg_high);   // big-endian signed 16-bit
};

}  // namespace mpu9250
