#!/usr/bin/env python3

import rospy
import roslaunch
import math
import os
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

def outdoor_server(req):
    # indoor -> outdoor
    # shutdown indoor node
    amcl_client()
    init_pose_client()
    move_base_client()
    map_server_client()
    ekf_localization_node_client()

    rospy.sleep(10)

    # excute outdoor launch
    excute_outdoor_pub.publish(True)

    rospy.loginfo("--------------------------------")

    return TriggerResponse(success=True, message="True")

def indoor_server(req):
    # outdoor -> indoor
    # shutdown outdoor node
    # amcl_client()
    move_base_client()
    outdoor_map_server_client()
    ekf_localization_node_client()
    navsat_transform_node_client()

    rospy.sleep(10)

    # excute outdoor launch
    excute_indoor_pub.publish(True)

    publish_outdoor_to_indoor_pose()

    rospy.loginfo("--------------------------------")

    return TriggerResponse(success=True, message="True")

def amcl_client():
    try:
        amcl = rospy.ServiceProxy('/amcl/shutdown_node', Trigger)
        rospy.loginfo("amcl shutdown")
        return amcl()
    except rospy.ServiceException as e:
        rospy.loginfo("amcl service call failed ")

def init_pose_client():
    try:
        init_pose = rospy.ServiceProxy('/init_pose/shutdown_node', Trigger)
        rospy.loginfo("init_pose shutdown")
        return init_pose()
    except rospy.ServiceException as e:
        rospy.loginfo("init_pose service call failed ")

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

def outdoor_map_server_client():
    try:
        outdoor_map_server = rospy.ServiceProxy('/outdoor_map_server/shutdown_node', Trigger)
        rospy.loginfo("outdoor_map_server shutdown")
        return outdoor_map_server()
    except rospy.ServiceException as e:
        rospy.loginfo("outdoor_map_server service call failed ")

def ekf_localization_node_client():
    try:
        ekf_localization_node = rospy.ServiceProxy('/ekf_localization_node/shutdown_node', Trigger)
        rospy.loginfo("ekf_localization_node shutdown")
        return ekf_localization_node()
    except rospy.ServiceException as e:
        rospy.loginfo("ekf_localization_node service call failed ")

def navsat_transform_node_client():
    try:
        navsat_transform_node = rospy.ServiceProxy('/navsat_transform/shutdown_node', Trigger)
        rospy.loginfo("navsat_transform_node shutdown")
        return navsat_transform_node()
    except rospy.ServiceException as e:
        rospy.loginfo("navsat_transform_node service call failed ")


def publish_outdoor_to_indoor_pose():
    # 메시지 객체 생성
    initial_pose_msg = PoseWithCovarianceStamped()

    # 메시지 필드 설정
    # 예제로 x = 1.0, y = 2.0, z = 0.0의 좌표를 설정합니다.
    initial_pose_msg.pose.pose.position.x = 1.0
    initial_pose_msg.pose.pose.position.y = 2.0
    yaw=30*math.py/180
    initial_pose_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
    
    # 메시지 퍼블리시
    initial_pose_publisher.publish(initial_pose_msg)


if __name__ == '__main__':
    rospy.init_node('node_shutdown')

    outdoor_srv = rospy.Service('/carrier_ros/outdoor', Trigger, outdoor_server)
    indoor_srv = rospy.Service('/carrier_ros/indoor', Trigger, indoor_server)

    excute_indoor_pub = rospy.Publisher('/excute/indoor', Bool, queue_size=1)
    excute_outdoor_pub = rospy.Publisher('/excute/outdoor', Bool, queue_size=1)
    initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    rospy.spin()