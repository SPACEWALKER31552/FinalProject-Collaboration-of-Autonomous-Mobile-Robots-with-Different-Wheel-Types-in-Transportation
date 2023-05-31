#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryToPath():
	def __init__(self):
		rospy.init_node('odometry_to_path',anonymous=True)
		self.odom_sub = rospy.Subscriber('/odom',Odometry,self.odom_callback)
		self.path_pub = rospy.Publisher('/path',Path ,queue_size = 10)
		self.path = Path()
		self.path.header.frame_id = 'odom'
		self.seq = 0
	
	def odom_callback(self, msg):
		pose_stamped = PoseStamped()
		pose_stamped.header.seq = self.seq
		pose_stamped.header.stamp = rospy.Time.now()
		pose_stamped.header.frame_id = 'odom'
		pose_stamped.pose = msg.pose.pose
		self.path.poses.append(pose_stamped)
		self.seq += 1
		self.path_pub.publish(self.path)
		
if __name__ == '__main__':
	try:
		odometry_to_path = OdometryToPath()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
