#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

counter3 = 0
current3A = 0
last3A = 0

speed_act3 = 0.00000

encoder_encoder3A = 22
encoder_encoder3B = 25

dir3 = 0

encoder_cpr = 360.00000; 
radius = 0.076; 

pos3 = 0 ;
lastpos3 = 0 ;
lastlastpos3 = 0 ;

def callback(msg):
    global dir3
    dir3 = msg.angular.x
    #rospy.loginfo("dir3 %f"%(dir3))
 
 
 
def init():
    rospy.loginfo("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_encoder3A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder3B, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    GPIO.add_event_detect(encoder_encoder3A, GPIO.BOTH,callback=rotation_decode3)
    
    return    

    

def rotation_decode3(encoder_encoder3A):
    global counter3
    global last3A
    global current3A
    #sleep(0.0002)
    current3A = GPIO.input(encoder_encoder3A)
    #encoder1B = GPIO.input(encoder_encoder1B)
    #rospy.loginfo("1 A = %d"%(encoder1A))
    #rospy.loginfo("1 B = %d"%(encoder1B))
 
    if current3A != last3A  :
        encoder3A = GPIO.input(encoder_encoder3A)
        encoder3B = GPIO.input(encoder_encoder3B)
        if encoder3A != encoder3B :
            if dir3 == 1.000000 :
                counter3 += 1 
            
        else :
            if dir3 == 2.000000 :
                counter3 -= 1
            #counter1 += 1 
            
        rospy.loginfo("tick3 = %f"%(counter3))
        last3A = current3A
        return


def main():
    global lastpos3
    global counter3
    global encoder_cpr
    global radius
    global speed_act3
    pub = rospy.Publisher('slaveencode3', String, queue_size = 10)
    pubtick = rospy.Publisher('slaveencodetick3', String, queue_size = 10)
    rospy.Subscriber("slavedirection", Twist,callback)
    rospy.init_node('slaveencode3')
    try :
        init()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if abs(counter3 - lastpos3) != 0 :
                speed_act3 = ((((counter3-lastpos3)/encoder_cpr)*2*3.14)*(1/0.1)*radius)
                
                lastpos3 = counter3 
            else :
                speed_act3 = 0
                
            pub.publish(str(speed_act3))
            pubtick.publish(str(counter3))
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
        GPIO.cleanup()


 
if __name__ == '__main__':
    main()



