#!/usr/bin/env python3

import rospy
import roslaunch
import os
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Trigger, TriggerRequest

def outdoor_server(req):
    # indoor -> outdoor
    # shutdown indoor node
    amcl_client()
    move_base_client()
    map_server_client()

    rospy.sleep(10)

    # excute outdoor launch
    excute_outdoor_client()

    return TriggerResponse(success=True, message="True")

def indoor_server(req):
    # outdoor -> indoor
    # shutdown outdoor node
    # amcl_client()
    # move_base_client()
    # map_server_client()

    # rospy.sleep(10)

    # excute outdoor launch
    excute_indoor_client()

    return TriggerResponse(success=True, message="True")

def excute_outdoor_client():
    try:
        outdoor = rospy.ServiceProxy('/excute/outdoor', Trigger)
        rospy.loginfo("Excute outdoor.launch")
        return outdoor()
    except rospy.ServiceException as e:
        rospy.loginfo("Excute outdoor.launch call failed ")

def excute_indoor_client():
    try:
        indoor = rospy.ServiceProxy('/excute/indoor', Trigger)
        rospy.loginfo("Excute indoor.launch")
        return indoor()
    except rospy.ServiceException as e:
        rospy.loginfo("Excute indoor.launch call failed ")

def amcl_client():
    try:
        amcl = rospy.ServiceProxy('/amcl/shutdown_node', Trigger)
        rospy.loginfo("amcl shutdown")
        return amcl()
    except rospy.ServiceException as e:
        rospy.loginfo("amcl service call failed ")

def move_base_client():
    try:
        move_base = rospy.ServiceProxy('/move_base/shutdown_node', Trigger)
        rospy.loginfo("move_base shutdown")
        return move_base()
    except rospy.ServiceException as e:
        rospy.loginfo("move_base service call failed ")
    
def map_server_client():
    try:
        map_server = rospy.ServiceProxy('/map_server/shutdown_node', Trigger)
        rospy.loginfo("map_server shutdown")
        return map_server()
    except rospy.ServiceException as e:
        rospy.loginfo("map_server service call failed ")


if __name__ == '__main__':
    rospy.init_node('node_shutdown')
    outdoor_srv = rospy.Service('/carrier_ros/outdoor', Trigger, outdoor_server)
    indoor_srv = rospy.Service('/carrier_ros/indoor', Trigger, indoor_server)
    rospy.spin()