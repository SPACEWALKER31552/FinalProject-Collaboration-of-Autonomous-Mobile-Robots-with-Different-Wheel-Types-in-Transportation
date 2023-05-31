#!/usr/bin/env python
import rospy
import tf.transformations
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Twist


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
    linearz = msg.linear.z	
    	
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
    robot1 = rospy.get_param("robot_name1",12) 
    robot2 = rospy.get_param("robot_name2",12) 
    robot3 = rospy.get_param("robot_name3",12) 
    
    #cmdleader = rospy.Publisher('cmd_velleader', Twist, queue_size = 10)
    #cmdslave = rospy.Publisher('cmd_velslave', Twist, queue_size = 10)
    cmdslave = rospy.Publisher('holo1/cmd_vel', Twist, queue_size = 10)
    cmdleader = rospy.Publisher('holo2/cmd_vel', Twist, queue_size = 10)
    
    try :
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            #do 3 robot kinematic here

            cmdleadervalue = Twist()
            cmdslavevalue = Twist()
            cmdleadervalue.linear.x = linearx
            cmdleadervalue.linear.y = lineary
            cmdleadervalue.linear.z = linearz
            cmdleadervalue.angular.x = angularx
            cmdleadervalue.angular.y = angulary
            cmdleadervalue.angular.z = angularz
            
            cmdslavevalue.linear.x = linearx
            cmdslavevalue.linear.y = lineary
            cmdslavevalue.linear.z = linearz
            cmdslavevalue.angular.x = angularx
            cmdslavevalue.angular.y = angulary
            cmdslavevalue.angular.z = angularz
            
            cmdleader.publish(cmdleadervalue)
            cmdslave.publish(cmdslavevalue)
            #rospy.loginfo("Published")
            
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
    	pass

if __name__ == '__main__':
    main()
