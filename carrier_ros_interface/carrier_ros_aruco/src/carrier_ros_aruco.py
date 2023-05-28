#!/usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerRequest
import math
import roslib; roslib.load_manifest('teleop_ardrone')

class aruco():

	def __init__(self):
		
		self.subs=[]
		self.subs.append(rospy.Subscriber('fiducial_transforms', FiducialTransformArray, self.aruco_callback))

		self.four_first = True
		self.six_first = True

	def aruco_callback(self, data):

		if len(data.transforms):
			
			self.quar_to_euler(
				data.transforms[0].transform.rotation.x,\
				data.transforms[0].transform.rotation.y,\
				data.transforms[0].transform.rotation.z,\
				data.transforms[0].transform.rotation.w
			)

			if data.transforms[0].fiducial_id == 3: 
				
				if self.four_first:
					self.indoorclient()				
					self.four_first = False 

				self.x = data.transforms[0].transform.translation.x
				self.y = data.transforms[0].transform.translation.y
				self.z = data.transforms[0].transform.translation.z

			elif data.transforms[0].fiducial_id == 4:
				
				if self.six_first:
					self.outdoorclient()				
					self.six_first = False 

				self.x = data.transforms[0].transform.translation.x
				self.y = data.transforms[0].transform.translation.y
				self.z = data.transforms[0].transform.translation.z


	def quar_to_euler(self, x, y, z, w):

		t0 = +2 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		self.roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		self.pitch_y = math.asin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		self.yaw_z = math.atan2(t3, t4)


	def indoorclient(self):
	    
		try:
			
			detect=rospy.ServiceProxy('/carrier_ros/indoor', Trigger)
			rospy.loginfo("Indoor")
			return detect()

		
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: ")

	def outdoorclient(self):
	    
		try:
			
			detect=rospy.ServiceProxy('/carrier_ros/outdoor', Trigger)
			rospy.loginfo("Outdoor")
			return detect()

		
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: ")
	

if __name__ == '__main__':

	rospy.init_node('carrier_ros_aruco_detect')
	cls_=aruco()
	rospy.spin()
