#!/usr/bin/env python3

import rospy
import roslaunch
import os
from std_msgs.msg import Bool

def excuteIndoorCallback(data):
    if (data.data == True):
        os.system("roslaunch carrier_ros_bringup carrier_ros_indoor.launch")

if __name__ == "__main__":
    rospy.init_node('excute_indoor_launch')
    # srv = rospy.Service('/excute/indoor', Trigger, excute_indoor_server)
    rospy.Subscriber('/excute/indoor', Bool, excuteIndoorCallback)
    rospy.spin()