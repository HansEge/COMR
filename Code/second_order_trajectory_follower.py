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

def make_plots(trajectory, measured_pos, distance, time):


	print("Printing figure")
	traj_df = pd.DataFrame(trajectory, columns=['x_d', 'y_d'])
	meas_pos_df = pd.DataFrame(measured_pos, columns=['x_r','y_r'])
	error_df = pd.DataFrame(distance, columns=['ro'])
	time_df = pd.DataFrame(time, columns=['time'])


	fig1 = go.Figure()
	fig2 = go.Figure()


	fig1.add_trace(go.Scatter(
		x=traj_df['x_d'],
		y=traj_df['y_d'],
		name="Desired position of robot"
		))

	fig1.add_trace(go.Scatter(
		x=meas_pos_df['x_r'],
		y=meas_pos_df['y_r'],
		name="Actual position of robot"
		))

	fig1.update_layout(
		title="Desired coordinates vs actual coordinates",
		xaxis_title="X-coordinates",
		yaxis_title="Y-coordinates"
		)

	fig2.add_trace(go.Scatter(
		y=error_df['ro'],
		x=time_df['time']
		))
	fig2.update_layout(
		title="Error vs time",
		xaxis_title="Time(s)",
		yaxis_title="Error(ro)"
		)


	fig1.show()
	fig2.show()



# Some inspiration found here: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
def huskyOdomCallback(data, args):
	
	pub, cmd_msg, goal, trajectory, measured_pos, distance, t_total, t0 = args

	a = 0.1

	goal[0] = goal[0]+0.0003
	goal[1] = a*goal[0]**2

	# Controller coefficients
	kp = 0.5
	ka = 0.5
	kb = -0.05

	# Get x and y from gazebo
	x = data.pose[1].position.x
	y = data.pose[1].position.y

	pos_quat = data.pose[1].orientation
	pos_eul = quaternion_to_euler(pos_quat.x, pos_quat.y,
								  pos_quat.z, pos_quat.w)
	
	# Yaw fra pos_eul in rads
	theta = pos_eul[2]
	# Distance to goal
	ro = math.sqrt((goal[0]-x)**2+(goal[1]-y)**2)
	
	# Angle between axis of the robots reference frame and the vector connecting the center of
	# the axle of the wheels with the final position
	alpha = -theta + math.atan2((goal[1]-y), (goal[0]-x))
	
	# Comment about beta
	beta = -theta-alpha

	#Velocity and anglular xx
	v = kp*ro
	w = ka*alpha+kb*beta

	# publish new speed and angle
	cmd_msg.linear.x = v
	cmd_msg.angular.z = w

	pub.publish(cmd_msg)
	# rate.sleep()

	#Debug
	#print(goal)
	t1 = datetime.datetime.now()
	duration = t1 - t0
	time = float(duration.total_seconds())

	measured_pos.append([x,y])
	trajectory.append([goal[0],goal[1]])
	t_total.append(time)
	distance.insert(0,round(ro,3))
	
	#print(distance)
	#print(duration.total_seconds())

	if len(trajectory) > 20000:
		make_plots(trajectory, measured_pos, distance, t_total)
		del trajectory[:]
		del measured_pos[:]
		del distance[:]
		del t_total[:]


def setup():

	print("Setup ")

	# Let's initialize the node first. Set the name to "basic_subscriber".	
	rospy.init_node("basic_subscriber", anonymous=True)

	
	trajectory = []
	measured_pos = []
	distance = []
	goal = [0,0]
	t_total = []

	t0 = datetime.datetime.now()

	# Message used to send commands to robot
	cmd_msg = Twist()

	# initialize publisher
	pub = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=10)


	# Initialize subscriber
	modelstate_subscriber = rospy.Subscriber('gazebo/model_states',  ModelStates,  huskyOdomCallback, (pub, cmd_msg, goal, trajectory, measured_pos, distance, t_total, t0))


if __name__ =='__main__':
	try:
		setup()


		# This line keeps the subscriber() function alive until a user terminates using CTRL+C.
		rospy.spin()

	except rospy.ROSInterruptException:
		pass