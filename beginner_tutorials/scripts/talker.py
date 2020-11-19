#!/usr/bin/env python

from random import randint
import rospy
from std_msgs.msg import Int32


def talker():
	pub1 = rospy.Publisher("integer1", Int32, queue_size=10)
	pub2 = rospy.Publisher("integer2", Int32, queue_size=10)


	rospy.init_node("talker", anonymous=True)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		rand_int1 = randint(0,5000)
		rand_int2 = randint(0,5000)

		rospy.loginfo("First number is %d", rand_int1)
		rospy.loginfo("Second number is %d", rand_int2)

		pub1.publish(rand_int1)
		pub2.publish(rand_int2)

		rate.sleep()


if __name__ == '__main__':
	try:
		talker()

	except rospy.ROSInterruptException:
		pass
	
