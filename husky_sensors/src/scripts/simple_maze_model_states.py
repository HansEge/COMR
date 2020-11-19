#!/usr/bin/env python
import rospy
import nav_msgs.msg
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

import plotly.graph_objects as go
import pandas as pd
import numpy as np

import time
import datetime


def quaternion_to_euler(x,y,z,w) :

	t0 = 2*(w*x+y*z)
	t1 = 1-2*(x*x+y*y)
	X = math.degrees(math.atan2(t0,t1))

	t2 = 2*(w*y-z*x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = 2*(w*z+x*y)
	t4 = 1-2*(y*y+z*z)
	Z = math.degrees(math.atan2(t3, t4))

	# X=roll Y=pitch Z=yaw
	X = math.atan2(t0, t1)
	Y = math.asin(t2)
	Z = math.atan2(t3,t4)

	return X, Y, Z

# Used for selecting next goal
goal_index = 0

# Some inspiration found here: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
def huskyOdomCallback(data, args):
	
	pub, cmd_msg, goals = args

	global goal_index

	current_goal = goals[goal_index]
	


	# Controller coefficients
	kp = 0.5
	ka = 0.5
	kb = -0.05

	# Get x and y from gazebo
	x = data.pose[2].position.x
	y = data.pose[2].position.y

	pos_quat = data.pose[2].orientation
	pos_eul = quaternion_to_euler(pos_quat.x, pos_quat.y,
								  pos_quat.z, pos_quat.w)
	
	# Yaw fra pos_eul in rads
	theta = pos_eul[2]
	# Distance to goal
	ro = math.sqrt((current_goal[0]-x)**2+(current_goal[1]-y)**2)
	
	# Angle between axis of the robots reference frame and the vector connecting the center of
	# the axle of the wheels with the final position
	alpha = -theta + math.atan2((current_goal[1]-y), (current_goal[0]-x))
	
	# Comment about beta
	beta = -theta-alpha

	#Velocity and anglular xx
	v = kp*ro
	w = ka*alpha+kb*beta

	# publish new speed and angle
	cmd_msg.linear.x = v
	cmd_msg.angular.z = w

	pub.publish(cmd_msg)

	print("---------------------")
	print("Current goal = " + str(current_goal))
	print("---------------------")
	print("ro = " + str(ro))
	print("---------------------")
	print("aplha = " + str(alpha))


	# Make robot go to the four corner coordinates
	if ro < 0.5:
		print("Setting new goal coordinates")
		goal_index = goal_index + 1
		
		if goal_index == len(goals):
			goal_index = 0
		
		current_goal = goals[goal_index]
	

	# rospy.signal_shutdown("Plot has been made. Shutting down...")



def setup():
	goal_index = 0

	print("Setup ")

	goals = [[2.8,0], [10,-1], [11,5], [4,5.2]]
	goal_index = 0

	# Setup node
	rospy.init_node("basic_subscriber", anonymous=True, disable_signals=True)

	# Message used to send commands to robot
	cmd_msg = Twist()

	# initialize publisher
	pub = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=10)

	# Initialize subscriber
	modelstate_subscriber = rospy.Subscriber('gazebo/model_states',  ModelStates,  huskyOdomCallback, (pub, cmd_msg, goals))


if __name__ =='__main__':
	try:
		setup()


		# This line keeps the subscriber() function alive until a user terminates using CTRL+C.
		rospy.spin()

	except rospy.ROSInterruptException:
		pass