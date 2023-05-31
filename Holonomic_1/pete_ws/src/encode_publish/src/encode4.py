#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

counter4 = 0
current4A = 0
last4A = 0

speed_act4 = 0.00000

encoder_encoder4A = 13
encoder_encoder4B = 16

dir4 = 0

encoder_cpr = 360.00000; 
radius = 0.076; 

pos4 = 0 ;
lastpos4 = 0 ;
lastlastpos4 = 0 ;

def callback(msg):
    global dir4
    dir4 = msg.angular.y
    #rospy.loginfo("dir4 %f"%(dir4))
    
 
def init():
    rospy.loginfo("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_encoder4A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder4B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.add_event_detect(encoder_encoder4A, GPIO.BOTH,callback=rotation_decode4)
    
    return    

    
def rotation_decode4(encoder_encoder4A):
    global counter4
    global last4A
    global current4A
    #sleep(0.0002)
    current4A = GPIO.input(encoder_encoder4A)
    #encoder1B = GPIO.input(encoder_encoder1B)
    #rospy.loginfo("1 A = %d"%(encoder1A))
    #rospy.loginfo("1 B = %d"%(encoder1B))
 
    if current4A != last4A  :
        encoder4A = GPIO.input(encoder_encoder4A)
        encoder4B = GPIO.input(encoder_encoder4B)
        if encoder4A != encoder4B :
            if dir4 == 1.000000 :
                counter4 += 1 
            
        else :
            if dir4 == 2.000000 :
                counter4 -= 1
            #counter1 += 1 
            
        rospy.loginfo("tick4 = %f"%(counter4))
        last4A = current4A
        return


def main():
    global lastpos4
    global counter4
    global encoder_cpr
    global radius
    global speed_act4
    pub = rospy.Publisher('slaveencode4', String, queue_size = 10)
    pubtick = rospy.Publisher('slaveencodetick4', String, queue_size = 10)
    rospy.Subscriber("slavedirection", Twist,callback)
    rospy.init_node('slaveencode4')
    try :
        init()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if abs(counter4 - lastpos4) != 0 :
                speed_act4 = ((((counter4-lastpos4)/encoder_cpr)*2*3.14)*(1/0.1)*radius)
                
                lastpos4 = counter4 
            else :
                speed_act4 = 0
                
            pub.publish(str(speed_act4))
            pubtick.publish(str(counter4))
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
        GPIO.cleanup()


 
if __name__ == '__main__':
    main()




