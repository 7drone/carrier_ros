#!/usr/bin/env python3

import rospy
import roslaunch
import os
from std_msgs.msg import Bool

def excuteOutdoorCallback(data):
    if (data.data == True):
        os.system("roslaunch carrier_ros_bringup carrier_ros_outdoor.launch")

if __name__ == "__main__":
    rospy.init_node('excute_outdoor_launch')
    rospy.Subscriber('/excute/outdoor', Bool, excuteOutdoorCallback)
    rospy.spin()