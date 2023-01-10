#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from unitysim.msg import BoundingBox3d
from numpy import mean
import threading
from basic_behavior.msg import Thought
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class safe_wanderer:
	def __init__(self):	
		self.lock = threading.Lock()
		rospy.init_node('safe_wanderer', anonymous=True)

		self.pub = rospy.Publisher("/ferna/cmd_vel",Twist,queue_size=10)
		#rospy.Subscriber("/ferna/thoughts", Thought, self.brain_callback)
		rospy.Subscriber("/ferna/scan", LaserScan, self.scan_callback)
		rospy.Subscriber("/ferna/speed", Float32, self.speed_callback)
		rospy.Subscriber("/ferna/angular", Float32, self.angular_callback)

		self.scan_data = None
		self.speed_data = None
		self.angular_data = None
		self.ranges = []

		self.rate = rospy.Rate(10)

	#def brain_callback(self, listen):
	#	self.lock.acquire()
	#	self.brain_data = listen
	#	self.lock.release()

	def scan_callback(self, data):
		self.lock.acquire()
		self.scan_data = data
		self.lock.release()

	def speed_callback(self, data):
		self.lock.acquire()
		self.speed_data = data
		self.lock.release()

	def angular_callback(self, data):
		self.lock.acquire()
		self.angular_data = data
		self.lock.release()

	def run(self):
		while not rospy.is_shutdown():
			if self.scan_data != None and self.speed_data != None and self.angular_data != None:
				self.ranges = []
				for i in range(len(self.scan_data.ranges)):
					if math.isinf(self.scan_data.ranges[i]):
						self.ranges.append(self.scan_data.range_max)
					else:
						self.ranges.append(self.scan_data.ranges[i])
				self.wander_safely()

				#if self.brain_data != None:
					#print(self.brain_data)
				#	if self.brain_data.blob_direction != 0:
				#		print("GOING FOR IT")
				#		self.head_for_blob()
				#	else:
				#		print("Taking it easy")
				#		self.wander_safely()
				#	if self.brain_data.value == 2:
				#		self.turn_around()
				#else:
				#	print("no brain data... wandering safely")
				
			self.rate.sleep()

	def turn_around(self):
		t = Twist()
		t.angular.z = 0.5
		self.pub.publish(t)

	def wander_safely(self):
		forward = 0		
		lateral = 0
		clearance_array = []

		for i in range(len(self.ranges)):
			forward += math.cos(self.indexToAngle(i)) * (self.ranges[i] / self.scan_data.range_max)
			lateral += math.sin(self.indexToAngle(i)) * (self.ranges[i] / self.scan_data.range_max)
			if i < self.angleToIndex(math.pi/4) and i > self.angleToIndex(-math.pi/4):
				clearance_array.append(self.ranges[i])

		forward /= len(self.ranges)
		lateral /= len(self.ranges)

		clearance = mean(clearance_array)
		angular_velocity = math.atan2(lateral, forward)

		ratio = self.speed_data.data/clearance
		pcVel = 1 - ratio + 1
		pcAng = self.angular_data.data/angular_velocity

		t = Twist()
		print(clearance / self.scan_data.range_max)
		print(angular_velocity)
		t.angular.z = pcAng * angular_velocity
		t.linear.x = pcVel * (clearance / self.scan_data.range_max)
		
		self.pub.publish(t)

	def wall_follow_left(self, a_range, b_range):
		print("Now is when I'd wall follow left")

	def wall_follow_right(self, a_range, b_range):
		print("Now is when I'd wall follow right")

	def angleToIndex(self, angle):
		return int(math.floor((angle - self.scan_data.angle_min)/self.scan_data.angle_increment))

	def indexToAngle(self, i):
		return i * self.scan_data.angle_increment + self.scan_data.angle_min

if __name__=='__main__':
	main = safe_wanderer()
	main.run()
