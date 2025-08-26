#!/bin/bash
# Simple beep script on GPIO 4
# Requires simple piezo buzzer (such as https://www.adafruit.com/product/160) connected to GPIO 4 and GND
GPIO=4
# Beep 3 times
for i in {1..3}; do
  gpioset gpiochip0 4=1
  sleep 0.2
  gpioset gpiochip0 4=0
  sleep 0.2
done