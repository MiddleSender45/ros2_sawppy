# dependencies
echo "--------INSTALLING DEPENDENCIES"
apt install ros-jazzy-joy-linux ros-jazzy-teleop-twist-joy ros-jazzy-urg-node -y >>/dev/null

# copy rover project
echo "--------COPYING SH FILES"
cp rover.sh /usr/local/bin/rover.sh >>/dev/null
cp beep.sh /usr/local/bin/beep.sh >>/dev/null

# copy rover service
echo "--------COPYING SERVICE FILES"
cp rover.service /etc/systemd/system/rover.service
cp beep.service /etc/systemd/system/beep.service

# add permissions
echo "--------ADDING PERMISSIONS"
chmod 744 /usr/local/bin/rover.sh
chmod 744 /usr/local/bin/beep.sh
chmod 664 /etc/systemd/system/rover.service
chmod 664 /etc/systemd/system/beep.service

# enable service
echo "--------ENABLING SERVICES"
systemctl daemon-reload
systemctl enable rover.service
systemctl enable beep.service

echo "--------STARTING SERVICES"
# start service
systemctl start rover.service
systemctl start beep.service
