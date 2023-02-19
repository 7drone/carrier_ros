#!/bin/bash

echo ""
echo "This script copies Carrier_ros udev rules to /etc/udev/rules.d/"
echo ""

echo ""
echo "Carrier_ros IMU (USB Serial) : /dev/ttyUSBx to /dev/ttyIMU :"
if [ -f "/etc/udev/rules.d/Carrier_ros_imu.rules" ]; then
    echo "Carrier_ros_imu.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty*", ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU" ' > /etc/udev/rules.d/Carrier_ros_imu.rules

    echo 'Carrier_ros_imu.rules created'
fi

echo ""
echo "Carrier_ros LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/Carrier_ros_lidar.rules" ]; then
    echo "Carrier_ros_lidar.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="1111", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLIDAR"' >/etc/udev/rules.d/Carrier_ros_lidar.rules    

    echo 'Carrier_ros_lidar.rules created'
fi

echo ""
echo "Carrier_ros dynamixel (USB Serial) : /dev/ttyUSBx to /dev/ttyDYNAMIXEL :"
if [ -f "/etc/udev/rules.d/Carrier_ros_dynamixel.rules" ]; then
    echo "Carrier_ros_dynamixel.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT78LEKI", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyDYNAMIXEL"' >/etc/udev/rules.d/Carrier_ros_dynamixel.rules    

    echo 'Carrier_ros_dynamixel.rules created'
fi

echo ""
echo "Carrier_ros arduino (USB Serial) : /dev/ttyACMx to /dev/ttyARDUINO :"
if [ -f "/etc/udev/rules.d/Carrier_ros_arduino.rules" ]; then
    echo "Carrier_ros_arduino.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", ATTRS{serial}=="9593132303235180C0A1", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyARDUINO"' >/etc/udev/rules.d/Carrier_ros_arduino.rules    

    echo 'Carrier_ros_arduino.rules created'
fi


echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
