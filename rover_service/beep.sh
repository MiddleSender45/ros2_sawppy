#!/bin/bash
# Simple beep script on GPIO 4
# Requires simple piezo buzzer (such as https://www.adafruit.com/product/160) connected to GPIO 4 and GND
GPIO=4
echo "$GPIO" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio$GPIO/direction

# Beep 3 times
for i in {1..3}; do
  echo 1 > /sys/class/gpio/gpio$GPIO/value
  sleep 0.2
  echo 0 > /sys/class/gpio/gpio$GPIO/value
  sleep 0.2
done

# Unexport GPIO
echo "$GPIO" > /sys/class/gpio/unexport
