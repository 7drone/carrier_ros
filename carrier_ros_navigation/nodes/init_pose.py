#!/usr/bin/env python3
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose
from std_srvs.srv import Trigger, TriggerResponse
 
def callback(msg):
    init_pose = SetPose()
    init_pose.pose = msg
    print(init_pose.pose)
    initialize_service(init_pose.pose)

def shutdown_node(req):
    rospy.signal_shutdown('All processes have finished.')
    return TriggerResponse(success=True, message="True")
 
rospy.init_node('pose_initializer', anonymous=True)
 
init_pose_srv = rospy.Service('/init_pose/shutdown_node', Trigger, shutdown_node)
sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped,callback)
rospy.wait_for_service('/set_pose')
 
initialize_service = rospy.ServiceProxy('/set_pose', SetPose)
 
 
rospy.spin()
