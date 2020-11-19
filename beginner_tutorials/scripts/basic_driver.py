#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def talker():
	#Define a publisher node
	pub = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=1000)

	#Let ROS master know the node
	rospy.init_node("husky_command_generator", anonymous=True)

	#Define publish rate
	rate = rospy.Rate(10) #HZ

	while not rospy.is_shutdown():
		#Create a message
		vel_msg = Twist()

		vel_msg.linear.x = 0.5
		vel_msg.linear.y = 0.0
		vel_msg.linear.z = 0
		
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0.5	

		#debug
		rospy.loginfo(vel_msg)

		#Publish a message
		pub.publish(vel_msg)

		rate.sleep()



if __name__ =='__main__':
	try:
		talker()

	except rospy.ROSInterruptException:
		pass