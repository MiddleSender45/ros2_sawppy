from smbus2 import SMBus
import time

# MPU9250 I2C address
MPU9250_ADDR = 0x68

# Registers
PWR_MGMT_1   = 0x6B

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

bus = SMBus(1)

# Wake up MPU9250
bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0)

time.sleep(0.1)


def read_word(reg):
    high = bus.read_byte_data(MPU9250_ADDR, reg)
    low  = bus.read_byte_data(MPU9250_ADDR, reg + 1)

    value = (high << 8) + low

    # Convert to signed value
    if value > 32767:
        value -= 65536

    return value


while True:
    # Accelerometer raw data
    accel_x = read_word(ACCEL_XOUT_H)
    accel_y = read_word(ACCEL_XOUT_H + 2)
    accel_z = read_word(ACCEL_XOUT_H + 4)

    # Gyroscope raw data
    gyro_x = read_word(GYRO_XOUT_H)
    gyro_y = read_word(GYRO_XOUT_H + 2)
    gyro_z = read_word(GYRO_XOUT_H + 4)

    print("Accelerometer:")
    print(f"X={accel_x}  Y={accel_y}  Z={accel_z}")

    print("Gyroscope:")
    print(f"X={gyro_x}  Y={gyro_y}  Z={gyro_z}")

    print("-" * 40)

    time.sleep(0.5)