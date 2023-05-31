#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

counter2 = 0
current2A = 0
last2A = 0

speed_act2 = 0.00000

encoder_encoder2A = 7
encoder_encoder2B = 8

dir2 = 0

encoder_cpr = 360.00000; 
radius = 0.076; 

pos2 = 0 ;
lastpos2 = 0 ;
lastlastpos2 = 0 ;

def callback(msg):
    global dir2
    dir2 = msg.linear.y
    #rospy.loginfo("dir2 %f"%(dir2))
    
def init():
    rospy.loginfo("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_encoder2A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder2B, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    GPIO.add_event_detect(encoder_encoder2A, GPIO.BOTH,callback=rotation_decode2)
    
    return    

    

def rotation_decode2(encoder_encoder2A):
    global counter2
    global last2A
    global current2A
    #sleep(0.0002)
    current2A = GPIO.input(encoder_encoder2A)
    #encoder1B = GPIO.input(encoder_encoder1B)
    #rospy.loginfo("1 A = %d"%(encoder1A))
    #rospy.loginfo("1 B = %d"%(encoder1B))
 
    if current2A != last2A  :
        encoder2A = GPIO.input(encoder_encoder2A)
        encoder2B = GPIO.input(encoder_encoder2B)
        if encoder2A != encoder2B :
            if dir2 == 1.000000 :
                counter2 += 1 
            
        else :
            if dir2 == 2.000000 :
                counter2 -= 1
            #counter1 += 1 
            
        rospy.loginfo("tick2 = %f"%(counter2))
        last2A = current2A
        return
    


def main():
    global lastpos2
    global counter2
    global encoder_cpr
    global radius
    global speed_act2
    pub = rospy.Publisher('slaveencode2', String, queue_size = 10)
    pubtick = rospy.Publisher('slaveencodetick2', String, queue_size = 10)
    rospy.Subscriber("slavedirection", Twist,callback)
    rospy.init_node('slaveencode2')
    try :
        init()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if abs(counter2 - lastpos2) != 0 :
                speed_act2 = ((((counter2-lastpos2)/encoder_cpr)*2*3.14)*(1/0.1)*radius)
                
                lastpos2 = counter2
            else :
                speed_act2 = 0
                
            pub.publish(str(speed_act2))
            pubtick.publish(str(counter2))
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
        GPIO.cleanup()


 
if __name__ == '__main__':
    main()

