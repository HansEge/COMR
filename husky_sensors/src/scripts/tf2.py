import rospy
import tf2 ros
from tf2 geometry msgs.msg import PoseStamped

if __name__ == '__main__':
 	rospy.init_node('tf2_listener')
 	tfBuffer = tf2_ros.Buffer()
 	listener = tf2_ros.TransformListener(tfBuffer) 
 	rate = rospy.Rate(10) #Hz
 	pose_stamped = PoseStamped()

 	#Import pose here

 	while not rospy.is_shutdown():
 		try:
 			transform = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())

 			pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

 			except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
 				tf2_ros.ExtrapolationException):
 			rate.sleep()
 			continue