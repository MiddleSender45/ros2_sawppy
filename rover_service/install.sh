#!/bin/bash -x

if [ "$EUID" -ne 0 ]; then
    echo "must use sudo to install"
    exit 1
fi

# dependencies
echo "--------INSTALLING DEPENDENCIES"
apt install ros-jazzy-joy-linux ros-jazzy-teleop-twist-joy ros-jazzy-urg-node -y >>/dev/null
apt install gpiod -y >>/dev/null

# Change to project directory
cd "$(dirname "$0")"

echo "--------COPYING SH FILES"
cp rover.sh /usr/local/bin/rover.sh >>/dev/null
cp beep.sh /usr/local/bin/beep.sh >>/dev/null

echo "--------COPYING SERVICE FILES"
cp rover.service /etc/systemd/system/rover.service
cp beep.service /etc/systemd/system/beep.service

echo "--------ADDING PERMISSIONS"
chmod 744 /usr/local/bin/rover.sh
chmod 744 /usr/local/bin/beep.sh
chmod 664 /etc/systemd/system/rover.service
chmod 664 /etc/systemd/system/beep.service

echo "--------ENABLING SERVICES"
systemctl daemon-reload
systemctl enable rover.service
systemctl enable beep.service

echo "--------STARTING SERVICES"
systemctl start rover.service
systemctl start beep.service
