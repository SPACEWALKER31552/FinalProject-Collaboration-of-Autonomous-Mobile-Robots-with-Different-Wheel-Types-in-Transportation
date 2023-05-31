#!/usr/bin/env python
import rospy
import tf.transformations
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

linearx = 0.0
lineary = 0.0
linearz = 0.0

angularx = 0.0
angulary = 0.0
angularz = 0.0

def callback(msg):

    global linearx
    global lineary
    global linearz
    global angularx
    global angulary
    global angularz
    
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    linearx = msg.linear.x
    lineary = msg.linear.y
    	
    angularx = msg.angular.x
    angulary = msg.angular.y
    angularz = msg.angular.z



def main():

    global linearx
    global lineary
    global linearz    
    global angularx
    global angulary
    global angularz
    
    rospy.loginfo("Start Main cmd_vel distributor")
    rospy.init_node('MasterKinematic')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    
    #cmdleader = rospy.Publisher('cmd_velleader', Twist, queue_size = 10)
    cmdslave = rospy.Publisher('lis/cmd_vel', Twist, queue_size = 10) 

    
    try :
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            #do 3 robot kinematic here

            #cmdleadervalue = Twist()
            cmdslavevalue = Twist()

            
            cmdslavevalue.linear.x = linearx
            cmdslavevalue.linear.y = lineary
            cmdslavevalue.linear.z = linearz
            cmdslavevalue.angular.x = angularx
            cmdslavevalue.angular.y = angulary
            cmdslavevalue.angular.z = angularz
            
            
            cmdslave.publish(cmdslavevalue)
            
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
    	pass

if __name__ == '__main__':
    main()
