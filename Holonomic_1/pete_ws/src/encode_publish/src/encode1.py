#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

counter1 = 0
current1A = 0
last1A = 0

speed_act1 = 0.00000

encoder_encoder1A = 17
encoder_encoder1B = 27

dir1 = 0

encoder_cpr = 360.00000; 
radius = 0.076; 

pos1 = 0 ;
lastpos1 = 0 ;
lastlastpos1 = 0 ;

def callback(msg):
    global dir1
    dir1 = msg.linear.x
    #rospy.loginfo("dir1 %f"%(dir1))
 
 
def init():
    rospy.loginfo("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_encoder1A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder1B, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    GPIO.add_event_detect(encoder_encoder1A, GPIO.BOTH,callback=rotation_decode1)
    
    global last1A
    global lastpos1
    lastpos1 = 0
    last1A = GPIO.input(encoder_encoder1A)
    return    

    

def rotation_decode1(encoder_encoder1A):
    global counter1
    global last1A
    global current1A
    #sleep(0.0002)
    current1A = GPIO.input(encoder_encoder1A)
    #encoder1B = GPIO.input(encoder_encoder1B)
    #rospy.loginfo("1 A = %d"%(encoder1A))
    #rospy.loginfo("1 B = %d"%(encoder1B))
 
    if current1A != last1A  :
        encoder1A = GPIO.input(encoder_encoder1A)
        encoder1B = GPIO.input(encoder_encoder1B)
        if encoder1A != encoder1B :
            if dir1 == 1.000000 :
                counter1 += 1 
            
        else :
            if dir1 == 2.000000 :
                counter1 -= 1
            #counter1 += 1 
            
        rospy.loginfo("tick1 = %f"%(counter1))
        last1A = current1A
        return

def main():
    global lastpos1
    global counter1
    global encoder_cpr
    global radius
    global speed_act1
    pub = rospy.Publisher('slaveencode1', String, queue_size = 10)
    pubtick = rospy.Publisher('slaveencodetick1', String, queue_size = 10)
    rospy.Subscriber("slavedirection", Twist,callback)
    rospy.init_node('slaveencode1')
    try :
        init()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if abs(counter1 - lastpos1) != 0 :
                speed_act1 = ((((counter1-lastpos1)/encoder_cpr)*2*3.14)*(1/0.1)*radius)
                
                lastpos1 = counter1 
            else :
                speed_act1 = 0
            
            pub.publish(str(speed_act1))
            pubtick.publish(str(counter1))
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
        GPIO.cleanup()


 
if __name__ == '__main__':
    main()

