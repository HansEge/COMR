#!/usr/bin/env python
import rospy
import nav_msgs.msg
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

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
current_goal = [2.8,0]

# Some inspiration found here: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
def husky_odo_callback(data, args):
	
	pub, cmd_msg, goals = args

	global goal_index
	global current_goal

	# Goal_index is doing weird stuff. This makes sure stuff doesn't break!
	if goal_index < len(goals):
		current_goal = goals[goal_index]
	
	# Controller coefficients
	kp = 0.3
	ka = 1.5
	kb = -0.001

	# Get x and y from gazebo
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	#print(data.twist.twist)

	pos_quat = data.pose.pose.orientation
	pos_eul = quaternion_to_euler(pos_quat.x, pos_quat.y,
								  pos_quat.z, pos_quat.w)
	
	# Yaw from pos_eul in rads
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

	#print(ro, current_goal)
	
	# Make robot go to the four corner coordinates
	if ro < 0.5:
		print("Setting new goal coordinates")
		goal_index = goal_index + 1

		# Make plots when the robot reaches last goal. 
		if goal_index == len(goals):

			goal_index = 0 # If we want the robot to loop

			# Make plots
			make_traj_plot(trajectory_model, trajectory_odo)
			make_dist_plot(distance_traveled_model, distance_traveled_odo)
			make_yaw_plot(yaw_model, yaw_odo)
		
			current_goal = goals[goal_index]

			# Kill node. Delete if you want to loop
			rospy.signal_shutdown("Plot has been made. Shutting down...")


# Makes plot of trajectory
def make_traj_plot(model, odo):

	print("Printing figure")

	fig1 = go.Figure()

	fig1.add_trace(go.Scatter(
		x=[row[0] for row in model],
		y=[row[1] for row in model],
		name="/gazebo/model_states"
		))

	fig1.add_trace(go.Scatter(
		x=[row[0] for row in odo],
		y=[row[1] for row in odo],
		name="/odometry/filtered"
		))

	fig1.update_layout(
		title="Odometry vs ground-truth position",
		xaxis_title="X-coordinates",
		yaxis_title="Y-coordinates"
		)

	fig1.show()

# Makes plot of distance over time
def make_dist_plot(dist_model, dist_odo):
	
	print("Printing figure")

	fig1 = go.Figure()

	fig1.add_trace(go.Scatter(
		x=[row[0] for row in dist_model],
		y=[row[1] for row in dist_model],
		name="Distance traveled(ground-truth)"
		))

	fig1.add_trace(go.Scatter(
		x=[row[0] for row in dist_odo],
		y=[row[1] for row in dist_odo],
		name="Distance traveled(odometry)"
		))

	fig1.update_layout(
		title="Distance traveled over time",
		xaxis_title="Time",
		yaxis_title="Distance"
		)

	fig1.show()


# Makes plot of yaw over time
def make_yaw_plot(yaw_m, yaw_o):
	
	print("Printing figure")

	fig1 = go.Figure()

	fig1.add_trace(go.Scatter(
		x=[row[1] for row in yaw_m],
		y=[row[0] for row in yaw_m],
		name="yaw(ground-truth)"
		))

	fig1.add_trace(go.Scatter(
		x=[row[1] for row in yaw_o],
		y=[row[0] for row in yaw_o],
		name="yaw(odometry)"
		))

	fig1.update_layout(
		title="yaw over time",
		xaxis_title="Time",
		yaxis_title="yaw"
		)

	fig1.show()


# Global variables used for plotting
yaw_odo = [[0,0]]
distance_traveled_odo =[[0,0]]
trajectory_odo = [[0,0]]
def odometry_callback(data, args):

	t0 = args
	global trajectory_odo
	global distance_traveled_odo
	global yaw_odo

	# Get time between now and start of simulation
	t1 = datetime.datetime.now()
	duration = t1 - t0
	time = float(duration.total_seconds())

	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	pos_quat = data.pose.pose.orientation
	pos_eul = quaternion_to_euler(pos_quat.x, pos_quat.y,
								  pos_quat.z, pos_quat.w)
	
	# Yaw from pos_eul in rads
	theta = pos_eul[2]

	new_pos = [x,y]

	# Calculate distance
	dist = get_euclidean_dist(trajectory_odo[-1], new_pos)
	new_dist = distance_traveled_odo[-1][0] + dist

	# Add stuff to global variables
	distance_traveled_odo.append([new_dist, time])
	yaw_odo.append([theta, time])
	trajectory_odo.append([x,y])


def get_euclidean_dist(cords1, cords2):
	return math.sqrt(math.pow(cords1[0]-cords2[0],2)+(math.pow(cords1[1]-cords2[1],2)))

yaw_model = [[0,0]]
distance_traveled_model = [[0,0]]
trajectory_model = [[0,0]]
def model_state_callback(data, args):

	t0 = args
	global trajectory_model
	global distance_traveled_model
	global yaw_model

	# Get time between now and start of simulation
	t1 = datetime.datetime.now()
	duration = t1 - t0
	time = float(duration.total_seconds())

	x = data.pose[2].position.x
	y = data.pose[2].position.y

	pos_quat = data.pose[2].orientation
	pos_eul = quaternion_to_euler(pos_quat.x, pos_quat.y,
								  pos_quat.z, pos_quat.w)
	
	# Yaw from pos_eul in rads
	theta = pos_eul[2]

	new_pos = [x,y]

	# Calculate distance
	dist = get_euclidean_dist(trajectory_model[-1], new_pos)
	new_dist = distance_traveled_model[-1][0] + dist

	# Add stuff to global variables
	distance_traveled_model.append([new_dist, time])
	yaw_model.append([theta, time])
	trajectory_model.append([x,y])


def setup():
	global goal_index

	print("Setup ")

	# The four corners of the "maze"
	goals = [[2.8,0], [10.5,-1], [11.3,5], [5,5]]

	goal_index = 0
	t0 = datetime.datetime.now()

	# Setup node
	rospy.init_node("basic_subscriber", anonymous=True, disable_signals=True)

	# Message used to send commands to robot
	cmd_msg = Twist()

	# initialize publisher
	pub = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=10)

	# Initialize subscriber
	odometry_subscriber = rospy.Subscriber('/odometry/filtered', Odometry,  husky_odo_callback, (pub, cmd_msg, goals))

	modelstate_plot_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback, (t0))
	odometry_plot_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback, (t0))


if __name__ =='__main__':
	try:
		setup()


		# This line keeps the subscriber() function alive until a user terminates using CTRL+C.
		rospy.spin()

	except rospy.ROSInterruptException:
		pass