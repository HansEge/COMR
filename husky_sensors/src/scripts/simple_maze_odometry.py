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

# Some inspiration found here: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
def husky_odo_callback(data, args):
	
	pub, cmd_msg, goals = args

	global goal_index

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
		
		if goal_index == len(goals):
			goal_index = 0

			make_traj_plot(trajectory_model, trajectory_odo)
			make_dist_plot(distance_traveled_model, distance_traveled_odo)
			make_yaw_plot(yaw_model, yaw_odo)
			
			current_goal = goals[goal_index]
			rospy.signal_shutdown("Plot has been made. Shutting down...")


def make_traj_plot(model, odo):

	print("Printing figure")
	traj_m_df = pd.DataFrame(model, columns=['x_model', 'y_model'])
	traj_o_df = pd.DataFrame(odo, columns=['x_odo','y_odo'])


	# Check to see if dataframes have same length
	if len(traj_m_df) != len(traj_o_df):
		if len(traj_m_df) > len(traj_o_df):
			traj_m_df.drop(traj_m_df.tail(1).index,inplace=True)
			
		elif len(traj_m_df) < len(traj_o_df):
			traj_o_df.drop(traj_o_df.tail(1).index,inplace=True)

	fig1 = go.Figure()

	fig1.add_trace(go.Scatter(
		x=traj_m_df['x_model'],
		y=traj_m_df['y_model'],
		name="/gazebo/model_states"
		))

	fig1.add_trace(go.Scatter(
		x=traj_o_df['x_odo'],
		y=traj_o_df['y_odo'],
		name="/odometry/filtered"
		))

	fig1.update_layout(
		title="Odometry vs ground-truth position",
		xaxis_title="X-coordinates",
		yaxis_title="Y-coordinates"
		)

	fig1.show()

def make_dist_plot(dist_model, dist_odo):
	
	print("Printing figure")
	dist_odo_df = pd.DataFrame(dist_odo, columns=['distance', 'time'])
	dist_model_df = pd.DataFrame(dist_model, columns=['distance', 'time'])

	fig1 = go.Figure()

	fig1.add_trace(go.Scatter(
		x=dist_model_df['time'],
		y=dist_model_df['distance'],
		name="Distance traveled(ground-truth)"
		))

	fig1.add_trace(go.Scatter(
		x=dist_odo_df['time'],
		y=dist_odo_df['distance'],
		name="Distance traveled(odometry)"
		))

	fig1.update_layout(
		title="Distance traveled over time",
		xaxis_title="Time",
		yaxis_title="Distance"
		)

	fig1.show()

def make_yaw_plot(yaw_m, yaw_o):
	
	print("Printing figure")
	yaw_odo_df = pd.DataFrame(yaw_o, columns=['yaw', 'time'])
	yaw_model_df = pd.DataFrame(yaw_m, columns=['yaw', 'time'])

	fig1 = go.Figure()

	fig1.add_trace(go.Scatter(
		x=yaw_model_df['time'],
		y=yaw_model_df['yaw'],
		name="yaw(ground-truth)"
		))

	fig1.add_trace(go.Scatter(
		x=yaw_odo_df['time'],
		y=yaw_odo_df['yaw'],
		name="yaw(odometry)"
		))

	fig1.update_layout(
		title="yaw over time",
		xaxis_title="Time",
		yaxis_title="yaw"
		)

	fig1.show()


yaw_odo = []
distance_traveled_odo =[]
trajectory_odo = []
def odometry_callback(data, args):

	t0, t_total_o = args
	global trajectory_odo
	global distance_traveled_odo
	global yaw_odo

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

	if len(trajectory_odo) != 0: 
		if len(distance_traveled_odo) == 0 and len(yaw_odo) == 0:
			distance_traveled_odo.append([0,0])
			yaw_odo.append([0,0])

		dist = get_euclidean_dist(trajectory_odo[-1], new_pos)
		new_dist = distance_traveled_odo[-1][0] + dist

		distance_traveled_odo.append([new_dist, time])
		yaw_odo.append([theta, time])

		print(distance_traveled_odo[-1])

	trajectory_odo.append([x,y])
	t_total_o.append(time)


def get_euclidean_dist(cords1, cords2):
	return math.sqrt(math.pow(cords1[0]-cords2[0],2)+(math.pow(cords1[1]-cords2[1],2)))

yaw_model = []
distance_traveled_model = []
trajectory_model = []
def model_state_callback(data, args):

	t0, t_total_m = args
	global trajectory_model
	global distance_traveled_model
	global yaw_model

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

	if len(trajectory_model) != 0: 
		if len(distance_traveled_model) == 0 and len(yaw_model) == 0:
			distance_traveled_model.append([0,0])
			yaw_model.append([0,0])

		dist = get_euclidean_dist(trajectory_model[-1], new_pos)
		new_dist = distance_traveled_model[-1][0] + dist

		distance_traveled_model.append([new_dist, time])
		yaw_model.append([theta, time])

		print(distance_traveled_model[-1])

	trajectory_model.append([x,y])
	t_total_m.append(time)


def setup():
	goal_index = 0

	print("Setup ")

	# The four corners of the "maze"
	goals = [[2.8,0], [10.5,-1], [11.3,5], [5,5]]
	goal_index = 0

	t0 = datetime.datetime.now()
	t_total_m = []
	t_total_o = []

	# Setup node
	rospy.init_node("basic_subscriber", anonymous=True, disable_signals=True)

	# Message used to send commands to robot
	cmd_msg = Twist()

	# initialize publisher
	pub = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=10)

	# Initialize subscriber
	odometry_subscriber = rospy.Subscriber('/odometry/filtered', Odometry,  husky_odo_callback, (pub, cmd_msg, goals))

	modelstate_plot_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback, (t0, t_total_m))
	odometry_plot_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback, (t0, t_total_o))


if __name__ =='__main__':
	try:
		setup()


		# This line keeps the subscriber() function alive until a user terminates using CTRL+C.
		rospy.spin()

	except rospy.ROSInterruptException:
		pass