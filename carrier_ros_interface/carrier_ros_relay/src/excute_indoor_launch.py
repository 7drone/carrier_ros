#!/usr/bin/env python3

import rospy
import roslaunch
import os
from std_srvs.srv import Trigger, TriggerResponse

def excute_indoor_server(req):
    os.system("roslaunch carrier_ros_bringup carrier_ros_indoor.launch")

    return TriggerResponse(success=True, message="True")


if __name__ == "__main__":
    rospy.init_node('excute_indoor_launch')
    srv = rospy.Service('/excute/indoor', Trigger, excute_indoor_server)
    rospy.spin()