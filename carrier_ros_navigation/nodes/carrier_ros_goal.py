#!/usr/bin/env python3


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import Empty, EmptyRequest

class Dest():
    start=(36.63743929986959, 8.153148086866908,2.3464171)
    final=(2.151869773864746,24.02182960510254,-0.0257961)

class MoveClient():
    def __init__(self):
        self.cnt=0
        self.srvs = [] 
        self.srvs.append(rospy.Service('robot/indoor/recall', Trigger, self.start)) 
        # self.srvs.append(rospy.Service('middle', Trigger, self.middle)) 
        self.srvs.append(rospy.Service('robot/indoor/start', Trigger, self.final)) 
        self.actionclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.timer=rospy.Timer(rospy.Duration(5), self.clear_callback)

    def euler2quat(self):
        self.qx = np.sin(0) * np.cos(0) * np.cos(self.theta) - np.cos(0) * np.sin(0) * np.sin(self.theta)
        self.qy = np.cos(0) * np.sin(0) * np.cos(self.theta) + np.sin(0) * np.cos(0) * np.sin(self.theta)
        self.qz = np.cos(0) * np.cos(0) * np.sin(self.theta) - np.sin(0) * np.sin(0) * np.cos(self.theta)
        self.qw = np.cos(0) * np.cos(0) * np.cos(self.theta) + np.sin(0) * np.sin(0) * np.sin(self.theta)
        self.movebase_client()

    def movebase_client(self):
        # self.actionclient.wait_for_server()final
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = self.qx
        goal.target_pose.pose.orientation.y = self.qy
        goal.target_pose.pose.orientation.z = self.qz
        goal.target_pose.pose.orientation.w = self.qw
        self.actionclient.send_goal(goal)
        wait = self.actionclient.wait_for_result()

        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        
        # else:
        #     state= self.actionclient.get_state()web_client_1

        #     if state == 3:
        #         # self.costmap_clear()
        #         # rospy.loginfo("self.cnt")
        #         # rospy.loginfo(self.cnt)

        #         if self.cnt==0:
        #             self.cnt+=1
        #             self.web_client_1()
                    
        #         elif self.cnt==1: 
        #             self.cnt+=1
        #             self.web_client_2()
                    
        #         elif self.cnt==2: 
        #             self.web_client_3()

            # else:
            #     return self.restart()

    # def restart(self):3.11
    #     rospy.loginfo("restart")
    #     rospy.sleep(5)
    #     self.movebase_client()

    def clear_callback(self, timer):
        self.costmap_clear()

    # def web_client_1(self):
    #     self.speakclient(3)
    #     web_service=rospy.ServiceProxy('middle_arrive', Trigger)
    #     return web_service()
        
    # def web_client_2(self):
    #     self.speakclient(5)
    #     web_service=rospy.ServiceProxy('final_arrive', Trigger)
    #     return web_service()

    # def web_client_3(self):
    #     self.speakclient(6)
    #     web_service=rospy.ServiceProxy('start_arrive', Trigger)
    #     return web_service()

    def costmap_clear(self):
        try:    
            start_clear = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
            return start_clear()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def indoor_finish(self):
        try:    
            indoorfinish = rospy.ServiceProxy('carrier_ros/outdoor', Trigger)
            rospy.loginfo("indoor finish")
            return indoorfinish()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def start(self,req):
        self.x=Dest.start[0]
        self.y=Dest.start[1]
        self.theta=Dest.start[2]
        self.euler2quat()
        return TriggerResponse(True,'first finish')

    # def middle(self, req):
    #     self.speakclient(2)
    #     self.x=Dest.middle[0]
    #     self.y=Dest.middle[1]
    #     self.theta=Dest.middle[2]
    #     self.euler2quat()
    #     return TriggerResponse(True,'middle finish')

    def final(self, req):
        self.x=Dest.final[0]
        self.y=Dest.final[1]
        self.theta=Dest.final[2]
        self.euler2quat()
        self.indoor_finish()
        return TriggerResponse(True,'final finish')



if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    cls_=MoveClient()
    rospy.spin()