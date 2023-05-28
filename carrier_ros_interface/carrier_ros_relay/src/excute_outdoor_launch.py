#!/usr/bin/env python3

import rospy
import roslaunch
import os
from std_srvs.srv import Trigger, TriggerResponse

def excute_outdoor_server(req):
    os.system("roslaunch carrier_ros_bringup carrier_ros_outdoor.launch")

    return TriggerResponse(success=True, message="True")


if __name__ == "__main__":
    rospy.init_node('excute_outdoor_launch')
    srv = rospy.Service('/excute/outdoor', Trigger, excute_outdoor_server)
    rospy.spin()