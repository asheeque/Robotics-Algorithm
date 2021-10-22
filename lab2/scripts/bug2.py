#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,PoseStamped, PoseWithCovarianceStamped, Pose2D,Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import numpy as np
import random


roll = pitch = yaw = 0.0
mode = None

direction_map ={
	'left':1,
	'right':-1
}

class Bug2:

	def __init__(self):
		self.velocity_pub = None
		self.mode = None
		self.front_to_wall = None
		self.right_to_wall = None
		self.left_to_wall = None

		self.check_wall_seek = False
		self.current_wall_slope = None

		self.is_direction_decided = False
		self.current_wall_seek_dir = None

		#Direction of car to be rotated
		self.target = None


		#pd controller
		self.kp = None
		self.kd = None
		self.required_distance = None
		self.current_error = 0
		self.previous_error = 0
		self.tot_gain = None
		self.min_distance_to_wall = None


		self.min_distance_index = None
		self.wall_direction = None
		self.checking_index = None
		self.do_rot = True

		self.robot_pos = None
		self.search = False
		self.goal_point = [4.5,9]

		self.direction_mem = None
		self.distance_mem = None


	def distance_from_point(self):
		y = (self.goal_point[1] - self.robot_pos[1]) **2
		x = (self.goal_point[0] - self.robot_pos[0]) **2

		dis = math.sqrt(x+y)
		return dis	

	def odom_callback(self,data):
		self.robot_pos = [data.pose.pose.position.x,data.pose.pose.position.y]
		global roll, pitch, yaw,target_slope
		orientation_q = data.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		target_slope = self.path_to_goal()

		# print(self.distance_from_point())
		print(self.mode)
		if self.mode == 'goal':
			# print(self.dt)
			self.goal_seek()
		elif self.mode == 'wall_seek':
			self.start_wall_seek()


	def goal_seek(self):
		err = target_slope*0.09
		# print(yaw)

		if self.distance_mem != None:
			if self.distance_from_point() > self.distance_mem:
				if self.current_wall_seek_dir == 'right':
					self.calc_wall_seek_angle()
					self.current_wall_seek_dir = 'left'
				else:
					self.current_wall_seek_dir = 'right'
					self.calc_wall_seek_angle()

		if  yaw < target_slope - err:
			self.rotate_robot(0.3,1)
		elif yaw >= target_slope+err :

			self.rotate_robot(0.3,-1)

		else:
			if self.distance_from_point() < 0.9:
				# self.move_robot(1,1)
				self.mode = 'Success'
				print('Success')

			elif self.distance_from_point() < 1.5:
				self.move_robot(1,1)


			elif self.front_to_wall  < 0.8:
				self.mode = 'wall_seek'

			else:
				self.move_robot(1,1)

	def cal_distance_from_line(self,point):
		x = point[0]
		y = point[1]
		line_point_one = [-8,-2]
		line_point_two = [4.5,9]
		numerator = abs((line_point_two[0]-line_point_one[0])*(line_point_one[1]- y)-(line_point_one[0]-x)*(line_point_two[1]-line_point_one[1]))
		denominator = math.sqrt(((line_point_two[0]-line_point_one[0])**2) + ((line_point_two[1]-line_point_one[1]) ** 2))
		distance = numerator/denominator
		return distance

	def choose_random_direction_wrt_wall(self):
		directions = ['left','right']
		self.current_wall_seek_dir = random.choice(directions)
		if self.right_to_wall < 2 or self.left_to_wall < 2:
			if self.right_to_wall < self.left_to_wall:
				self.current_wall_seek_dir = 'left'
			if self.right_to_wall < self.left_to_wall:
				self.current_wall_seek_dir = 'right'
		# self.current_wall_seek_dir = 'left'
		self.distance_mem = self.distance_from_point()
		if self.current_wall_seek_dir == 'right':
			self.target = yaw - 1.57
			if self.target < -math.pi:
				self.target = self.target + 2*math.pi
		else:
			self.target = yaw + 1.57
			if self.target >  math.pi:
				self.target = self.target - 2*math.pi

		self.calc_wall_seek_angle()
		self.is_direction_decided = True

	def calc_wall_seek_angle(self):
		if self.current_wall_seek_dir == 'right':
			self.target = yaw - 1.57
			if self.target < -math.pi:
				self.target = self.target + 2*math.pi
		else:
			self.target = yaw + 1.57
			if self.target >  math.pi:
				self.target = self.target - 2*math.pi


	def start_wall_seek(self):		
		if self.is_direction_decided == False:
			self.choose_random_direction_wrt_wall()
		slope_err = self.target * 0.2
		upper_bound = self.target + slope_err
		lower_bound = self.target - slope_err
		upper_threshold = max(upper_bound,lower_bound)
		lower_threshold = min(upper_bound,lower_bound)

		if (yaw > upper_threshold or yaw < lower_threshold) and self.do_rot:
			if self.current_wall_seek_dir == 'right':
				self.do_wall_follow(.1,0.3,-1)
			elif self.current_wall_seek_dir == 'left':
				self.do_wall_follow(.1,0.3,1)
		else:
			# self.rotate_robot(0.0,1)
			if self.do_rot:
				self.checking_index = self.min_distance_index
				if self.checking_index > 180 :
					self.wall_direction = 1
				else:
					self.wall_direction = -1
				self.do_rot = False
			self.move_along_wall()
		# print(line_x,line_y)

	def move_along_wall(self):

		dis = self.cal_distance_from_line(self.robot_pos)
		if dis > 3:
			self.search = True
		if dis < 1 and self.search:
			self.do_rot = True
			self.mode = 'goal'
			self.search = False

		# print(self.front_to_wall)
		if self.front_to_wall < 2:
		# 	# while self.front_to_wall > 2.5:
			self.do_wall_follow(0,0.3,-1*self.wall_direction)


		elif laserdata[self.checking_index] > 1.5:
			self.do_wall_follow(0.3,0.7,self.wall_direction)
			self.previous_error = 0



		else:
			self.current_error = laserdata[self.checking_index] - self.required_distance
			derivative_error = (self.current_error - self.previous_error)/self.dt
			self.tot_gain = self.kp * self.current_error + self.kd * derivative_error
			self.previous_error = self.current_error
			if self.min_distance_to_wall < 0.7:
				self.do_wall_follow(0.5,self.tot_gain,self.wall_direction)
			else:
				self.do_wall_follow(1,self.tot_gain,self.wall_direction)


	def do_wall_follow(self,speed,rot,direction):
		vel = Twist()
		vel.linear.x = speed
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = rot * direction
		self.velocity_pub.publish(vel)


	def scan_callback(self,data):
		global laserdata
		laserdata =  data.ranges
		front_to_wall = data.ranges[165:196]
		self.front_to_wall = np.mean(front_to_wall)
		print(self.front_to_wall)
		self.right_to_wall = data.ranges[0]
		self.left_to_wall = data.ranges[360]
		self.min_distance_to_wall = min(data.ranges)
		self.min_distance_index = data.ranges.index(self.min_distance_to_wall)
	


	def rotate_robot(self,val,direction):
		vel = Twist()
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = val * direction

		self.velocity_pub.publish(vel)

	def move_robot(self,val,direction):
		vel = Twist()
		vel.linear.x = val * direction
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0

		self.velocity_pub.publish(vel)


	def assign_pd(self,kp,kd,dt,required_distance):

		self.kp = kp
		self.kd = kd
		self.dt = dt
		self.required_distance = required_distance

	def start_bug2(self):
		rospy.init_node('~', anonymous=True)

		if self.mode == None:	
			self.mode = 'goal'
		self.assign_pd(1.3,2.7,0.2,0.7)
		self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		rospy.Subscriber('/base_scan',LaserScan,self.scan_callback)
		rospy.Subscriber('/odom',Odometry,self.odom_callback)
		rospy.spin()



	def path_to_goal(self):

		start_x = -8.0
		start_y = -2.0
		goal_x = 4.5
		goal_y = 9.0
		slope = (goal_y - start_y) / (goal_x - start_x)
		# angle = 180.0 * np.arctan(slope) / 3.14
		# print(slope,angle)
		slope = np.arctan(slope)
		return slope


if __name__ == "__main__":
	bug = Bug2()
	slope = bug.path_to_goal()
	print(slope)
	
	bug.start_bug2()