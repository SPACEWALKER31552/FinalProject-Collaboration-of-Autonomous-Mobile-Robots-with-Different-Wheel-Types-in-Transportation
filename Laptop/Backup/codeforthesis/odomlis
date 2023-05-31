#!/usr/bin/env python
import rospy
import tf.transformations
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math 

orientholo1z = 0.0
orientholo2z = 0.0
orientholo3z = 0.0

def odometry1(msg):
    global orientholo1z
    # yaw (z-axis rotation)
    siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y);
    cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
    orientholo1z = math.atan2(siny_cosp, cosy_cosp);
    orientholo1z = orientholo1z * 180/3.14 ;
    
def odometry2(msg):
    global orientholo2z
    siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y);
    cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
    orientholo2z = math.atan2(siny_cosp, cosy_cosp);
    orientholo2z = orientholo2z * 180/3.14 ;
    
def odometry3(msg):
    global orientholo3z
    # yaw (z-axis rotation)
    siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y);
    cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
    orientholo3z = math.atan2(siny_cosp, cosy_cosp);
    orientholo3z = orientholo3z * 180/3.14 ;
             

    
def main():

    global orientholo1z
    global orientholo2z
    global orientholo3z
    
    rospy.init_node('odomcallback', anonymous=True) #make node 
    #rospy.Subscriber('holo1/odom',Odometry,odometry1)
    rospy.Subscriber('non1/odom',Odometry,odometry1)
    rospy.Subscriber('holo2/odom',Odometry,odometry2)
    rospy.Subscriber('holo3/odom',Odometry,odometry3)
    odompy = rospy.Publisher('holo/odompy', Twist, queue_size = 10)
    try :
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
        	odompyvalue = Twist()  
        	odompyvalue.angular.x = orientholo1z
        	odompyvalue.angular.y = orientholo2z
        	odompyvalue.angular.z = orientholo3z
        	odompy.publish(odompyvalue)
        	rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
    	pass

if __name__ == '__main__':
    main()
