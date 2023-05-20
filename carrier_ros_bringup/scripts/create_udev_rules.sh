#!/bin/bash

echo ""
echo "This script copies Carrier_ros udev rules to /etc/udev/rules.d/"
echo ""


echo "Carrier_ros motor (USB Serial) : /dev/ttyUSBx to /dev/ttyMotor :"
if [ -f "/etc/udev/rules.d/Carrier_ros_motor.rules" ]; then
    echo "Carrier_ros_motor.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyMotor"' >/etc/udev/rules.d/Carrier_ros_motor.rules    

    echo 'Carrier_ros_lidar.rules created'
fi

echo ""
echo "Carrier_ros LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLEFTLiDAR :"
if [ -f "/etc/udev/rules.d/Carrier_ros_lidar.rules" ]; then
    echo "Carrier_ros_lidar.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0002", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLEFTLiDAR"' >/etc/udev/rules.d/Carrier_ros_leftlidar.rules    

    echo 'Carrier_ros_lidar.rules created'
fi

echo "Carrier_ros LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyRIGHTLiDAR :"
if [ -f "/etc/udev/rules.d/Carrier_ros_lidar.rules" ]; then
    echo "Carrier_ros_lidar.rules file already exist."
else 
    echo  'KERNEL=="tty*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyRIGHTLiDAR"' >/etc/udev/rules.d/Carrier_ros_rightlidar.rules    

    echo 'Carrier_ros_lidar.rules created'
fi

echo ""
echo "Carrier_ros IMU (USB Serial) : /dev/ttyUSBx to /dev/ttyIMU :"
if [ -f "/etc/udev/rules.d/Carrier_ros_imu.rules" ]; then
    echo "Carrier_ros_imu.rules file already exist."
else 
    echo 'SUBSYSTEM=="tty*", ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", ATTRS{serial}=="0004", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU" ' > /etc/udev/rules.d/Carrier_ros_imu.rules

    echo 'Carrier_ros_imu.rules created'
fi


echo ""
echo "Carrier_ros_gps (USB Serial) : /dev/ttyACMx to /dev/ttyGPS :"
if [ -f "/etc/udev/rules.d/Carrier_ros_gps.rules" ]; then
    echo "Carrier_ros_gps.rules file already exist."
else 
    echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0424", ATTRS{idProduct}=="2514", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyGPS"' >/etc/udev/rules.d/Carrier_ros_gps.rules    

    echo 'Carrier_ros_gps.rules created'
fi


echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
