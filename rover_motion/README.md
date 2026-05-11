# rover_motion

ROS 2 **Jazzy** C++ package for the **MPU-9250** IMU over I²C.  
Designed to slot into a rover odometry stack alongside wheel encoders / EKF.

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/msg/Imu` | Accel (m/s²) + Gyro (rad/s), REP-145 covariances |
| `/imu/temp` | `sensor_msgs/msg/Temperature` | Die temperature in °C (disable with `publish_temp: false`) |

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `/dev/i2c-1` | I²C bus device |
| `i2c_addr` | `104` (0x68) | Sensor I²C address |
| `frame_id` | `imu_link` | TF frame stamped on messages |
| `frequency` | `50.0` | Publish rate (Hz) |
| `publish_temp` | `true` | Enable temperature topic |

---

## Build

```bash
# 1 – install system dependency
sudo apt install libi2c-dev

# 2 – place this package in your workspace
cd ~/ros2_ws/src
cp -r <path/to/rover_motion> .

# 3 – build
cd ~/ros2_ws
colcon build --packages-select rover_motion
source install/setup.bash
```

---

## Run

```bash
# Default parameters (reads from config/imu_params.yaml)
ros2 launch rover_motion imu.launch.py

# Override on the CLI
ros2 launch rover_motion imu.launch.py frequency:=100.0 frame_id:=base_imu
```

---

## Wiring (Raspberry Pi / Jetson)

| MPU-9250 pin | RPi pin |
|--------------|---------|
| VCC | 3.3 V (pin 1) |
| GND | GND (pin 6) |
| SDA | GPIO 2 – SDA (pin 3) |
| SCL | GPIO 3 – SCL (pin 5) |
| AD0 | GND → address 0x68 |

Enable I²C on Raspberry Pi: `sudo raspi-config` → Interface Options → I2C → Enable.

---

## Integration with robot_localization EKF

Add this to your `ekf.yaml`:

```yaml
imu0: /imu/data
imu0_config: [false, false, false,   # x, y, z position
               false, false, false,   # roll, pitch, yaw
               false, false, false,   # vx, vy, vz
               true,  true,  true,    # vroll, vpitch, vyaw  ← gyro
               true,  true,  true]    # ax, ay, az           ← accel
imu0_differential: false
imu0_remove_gravitational_acceleration: true
```

---

## Tuning covariances

The default covariance values are conservative estimates.  
To measure real noise, record a static bag and compute Allan variance:

```bash
ros2 bag record /imu/data -o imu_static
# then analyse with imu_utils or similar
```
