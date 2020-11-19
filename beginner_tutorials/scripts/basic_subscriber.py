#!/usr/bin/env python

import rospy
​
# This is the message type that we want to subscribe.
from gazebo_msgs.msg import ModelStates
​
def callback(data):
	# Read x and y coordinate of robot given in data (gazebo/model_states).
	# You can see the details of the ModelState type message: http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelState.html. 

	x = data.pose[1].position.x
	y = data.pose[1].position.y

	# Try yourself: Read the orientation, as well.
	print(x,y) 



def subscriber():
	# Let's initialize the node first. Set the name to "basic_subscriber".	

	rospy.init_node("basic_subscriber", anonymous=True)

	# Create a subscriber to subscribe the topic of "gazebo/model_states". The type of "gazebo/model_states" is ModelStates.
	# When we execute below line our node will subscribe to "gazebo/model_states" immediately. And It will call callback function when triggered with the arrival of new message.
	modelstate_subscriber = rospy.Subscriber('gazebo/model_states',  ModelStates,  callback)

	# This line keeps the subscriber() function alive until a user terminates using CTRL+X.

	rospy.spin()
​

if __name__=='__main__':

	try:

		subscriber()

	except:

		pass