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

offset_x = 2.0
offset_y = -0.25
max_x = 5
max_y = 2

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

    global linearxW
    global lineary
    global linearz    
    global angularx
    global angulary
    global angularz
    
    rospy.loginfo("Start Main cmd_vel distributor")
    rospy.init_node('MasterKinematic')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    
    #cmdleader = rospy.Publisher('cmd_velleader', Twist, queue_size = 10)
    cmdslave = rospy.Publisher('holo1/cmd_vel', Twist, queue_size = 10)  
    cmdslave2 = rospy.Publisher('holo2/cmd_vel', Twist, queue_size = 10)
    cmdslave3 = rospy.Publisher('holo3/cmd_vel', Twist, queue_size = 10)

    
    try :
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            #do 3 robot kinematic here

            #cmdleadervalue = Twist()
            cmdslavevalue = Twist()
            cmdslavevalue2 = Twist()
            cmdslavevalue3 = Twist()

            new_linear_x = (linearx - offset_x) *1.5
            new_linear_y = (lineary - offset_y) *0.3
            
            cmdslavevalue.linear.x = new_linear_x
            cmdslavevalue.linear.y = 0
            cmdslavevalue.linear.z = 0
            cmdslavevalue.angular.x = 0
            cmdslavevalue.angular.y = 0
            cmdslavevalue.angular.z = new_linear_y
            
            
            cmdslavevalue2.linear.x = new_linear_x
            cmdslavevalue2.linear.y = 0
            cmdslavevalue2.linear.z = 0
            cmdslavevalue2.angular.x = 0
            cmdslavevalue2.angular.y = 0
            cmdslavevalue2.angular.z = new_linear_y
            #*(1/0.2792)
            
            
            cmdslavevalue3.linear.x = new_linear_x
            cmdslavevalue3.linear.y = 0
            cmdslavevalue3.linear.z = 0
            cmdslavevalue3.angular.x = 0
            cmdslavevalue3.angular.y = 0
            cmdslavevalue3.angular.z = new_linear_y
            
            
            #cmdleader.publish(cmdleadervalue)
            cmdslave.publish(cmdslavevalue)
            cmdslave2.publish(cmdslavevalue2)
            cmdslave3.publish(cmdslavevalue3)
            #rospy.loginfo("Published")
            
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
    	pass

if __name__ == '__main__':
    main()
