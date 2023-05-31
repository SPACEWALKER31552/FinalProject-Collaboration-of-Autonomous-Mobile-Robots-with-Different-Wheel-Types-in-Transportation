#!/usr/bin/env python
import rospy
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    w1 = (1/7.25)*(msg.linear.x - msg.linear.y - ((18+17.5)*(msg.angular.z)))
    w2 = (1/7.25)*(msg.linear.x + msg.linear.y + ((18+17.5)*(msg.angular.z)))
    w3 = (1/7.25)*(msg.linear.x + msg.linear.y - ((18+17.5)*(msg.angular.z)))
    w4 = (1/7.25)*(msg.linear.x - msg.linear.y + ((18+17.5)*(msg.angular.z)))
    rospy.loginfo("w1 = %f"%w1)
    rospy.loginfo("w2 = %f"%w2)
    rospy.loginfo("w3 = %f"%w3)
    rospy.loginfo("w4 = %f"%w4)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
