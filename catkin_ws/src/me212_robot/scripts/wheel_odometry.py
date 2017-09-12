#!/usr/bin/python

import rospy
import tf
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class wheelOdometry(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.br = tf.TransformBroadcaster()
		self.x = 0 # robot position x in meter
		self.y = 0 # robot position y in meter
		self.yaw = 0 # robot pose theta in radian
        	self.roll = 0
		self.pitch = 0
		self.frame_id = 0
		# publisher
		#self.pubOdometry = rospy.Publisher("/odometry", Odometry, queue_size = 10)
		#self.pubMarker = rospy.Publisher("/odom_marker", Marker, queue_size =10)
		# subscriber
		#self.sub_RP = rospy.Subscriber("~orientationRP", Float64MultiArray, self.cbOrientation)
		self.sub_odom = rospy.Subscriber("/serial_node/odometry", Float64MultiArray, self.cbPose)
		rospy.on_shutdown(self.custom_shutdown) # shutdown method
		#self.update_timer = rospy.Timer(rospy.Duration.from_sec(1), self.update) # timer for update robot pose, update rate = 1 Hz

		rospy.loginfo("[%s] Initialized " %self.node_name)
		self.now = rospy.get_time() # start

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

	'''def update(self, event): # update robot pose method, which will execute repeatedly
		dt = rospy.get_time() - self.now # in sec
		self.dphi_R = float(self.encoder_pre_R - self.encoder_pos_R) / self.CPR * (2 * pi) * self.radius  # without convert to float, you will get zero since int division! in meter
		self.dphi_L = float(self.encoder_pre_L - self.encoder_pos_L) / self.CPR * (2 * pi) * self.radius
		linear = (self.dphi_R + self.dphi_L) / 2 / dt # linear velocity in meter per second
		self.yaw += (self.dphi_R - self.dphi_L) / self.width
		angular = (self.dphi_R - self.dphi_L) / self.width / dt # angular velocity in radian per second
		self.x += cos(self.yaw - (self.dphi_R - self.dphi_L) / (2 * self.width)) * (self.dphi_R + self.dphi_L) / 2
		self.y += sin(self.yaw - (self.dphi_R - self.dphi_L) / (2 * self.width)) * (self.dphi_R + self.dphi_L) / 2
		self.yaw = self.yaw % (2 * pi) # normalized to [0, 2pi)
		#print "dphi_R:", self.dphi_R," dphi_L:", self.dphi_L # uncomment to print the distance wheels traveled
		self.encoder_pos_L = self.encoder_pre_L
		self.encoder_pos_R = self.encoder_pre_R # encoder information refrash
		# send tf from base_link to world
		self.br.sendTransform((self.x, self.y, 0), # to 3d translation
                                tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw), # to 3d rotation
                                rospy.Time.now(), # timestamp
                                "base_link", # robot frame
                                "map") # base frame
		print "x=", self.x," y=", self.y, " theta=" ,self.yaw  # uncomment to print the position and pose of the robot
		quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "map"
		odom.child_frame_id = "wheel_odometry"
		# pose
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0
		odom.pose.pose.orientation.x = quaternion[0]
		odom.pose.pose.orientation.y = quaternion[1]
		odom.pose.pose.orientation.z = quaternion[2]
		odom.pose.pose.orientation.w = quaternion[3]
		# twist
		odom.twist.twist.linear.x = linear * cos(self.yaw)
		odom.twist.twist.linear.y = linear * sin(self.yaw)
		odom.twist.twist.linear.z = 0
		odom.twist.twist.angular.x = 0
		odom.twist.twist.angular.y = 0
		odom.twist.twist.angular.z = angular
		self.pubOdometry.publish(odom)
		
		marker = Marker()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = "map"
		marker.type = Points
		marker.action = ADD
		marker.pose.position.x = 0
		marker.pose.position.y = 0
		marker.pose.position.z = 0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1 
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		rgba = [0.6, 0.6, 0.6, 1]
		marker.color.
		self.now = rospy.get_time()'''
	'''def cbOrientation(self, msg):
		self.roll = msg.data[0]
		self.pitch = msg.data[1]'''
	def cbPose(self, msg):
		self.x = msg.data[0]
		self.y = msg.data[1]
		self.yaw = msg.data[2]
		self.br.sendTransform((self.x, self.y, 0), # to 3d translation
                                      tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw), # to 3d rotation
                                      rospy.Time.now(), # timestamp
                                      "base_link", # robot frame
                                      "map") # base frame
		'''marker = Marker()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = "map"
		marker.type = marker.POINTS
		marker.action = marker.ADD
		marker.pose.position.x = self.x
		marker.pose.position.y = self.y
		marker.pose.position.z = 0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.r = 0.6
		marker.color.g = 0.6
		marker.color.b = 0.6
		marker.color.a = 1
		self.pubMarker.publish(marker)'''

if __name__ == "__main__":
	rospy.init_node("odometry", anonymous = False)
	odometry_node = wheelOdometry()
	rospy.spin()
