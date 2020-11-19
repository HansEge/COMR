#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


global first, second, result

first = 0
second = 0
result = 0


def callback(msg):

	if first == 0:
		global first
		first = msg.data
	elif second == 0:
		global second
		second = msg.data

	if first != 0 and second != 0:
		global result
		result = first + second
		rospy.loginfo("Sum of %d and %d is %d", first, second, result)
		first = 0
		second = 0
		result = 0
	rospy.loginfo("Need another number")


	
def listener():
	rospy.init_node("listener", anonymous=True)

	rospy.Subscriber("integer1", Int32, callback)
	rospy.Subscriber("integer2", Int32, callback)


	rospy.spin()

if __name__=="__main__":

	listener()
