#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import String

counter1 = 0
counter2 = 0
counter3 = 0
counter4 = 0

encoder_encoder1A = 17
encoder_encoder1B = 27

encoder_encoder2A = 7
encoder_encoder2B = 8

encoder_encoder3A = 22
encoder_encoder3B = 25

encoder_encoder4A = 13
encoder_encoder4B = 16


 
 
def init():
    rospy.loginfo("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_encoder1A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder1B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder2A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder2B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder3A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder3B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder4A, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(encoder_encoder4B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    #GPIO.add_event_detect(encoder_encoder1A, GPIO.BOTH,callback=rotation_decode1,bouncetime = 10)
    GPIO.add_event_detect(encoder_encoder2A, GPIO.BOTH,callback=rotation_decode2,bouncetime = 10)
    #GPIO.add_event_detect(encoder_encoder3A, GPIO.BOTH,callback=rotation_decode3,bouncetime = 10)
    #GPIO.add_event_detect(encoder_encoder4A, GPIO.BOTH,callback=rotation_decode4,bouncetime = 10)
   
    return    
    #while not rospy.is_shutdown():
        #encoder1A = GPIO.input(encoder_encoder1A)
        #encoder1B = GPIO.input(encoder_encoder1B)
        #rospy.loginfo("Switch A = %f"%(Switch_A))
        #rospy.loginfo("Switch B = %f"%(encoder1B))
        #state1last = GPIO.input(encoder_encoder1A)
        
        #GPIO.add_event_detect(encoder_encoder1A, GPIO.RISING, callback=rotation_decode)
        
        #GPIO.add_event_detect(encoder_encoder1B, GPIO.RISING, callback=rotation_decoder)
        
    
 
def rotation_decode1(encoder_encoder1A):
    global counter1
    sleep(0.002)
    encoder1A = GPIO.input(encoder_encoder1A)
    encoder1B = GPIO.input(encoder_encoder1B)
    rospy.loginfo("1 A = %d"%(encoder1A))
    rospy.loginfo("1 B = %d"%(encoder1B))
 
    if (encoder1A == 1) and (encoder1B == 0):

        rospy.loginfo("tick1 = %f"%(counter1))
        while encoder1B == 0:
            encoder1B = GPIO.input(encoder_encoder1B)
        while encoder1B == 1:
            encoder1B = GPIO.input(encoder_encoder1B)
        counter1 -= 1
        return
 
    elif (encoder1A == 0) and (encoder1B == 1):
        
        rospy.loginfo("tick1 = %f"%(counter1))
        while encoder1A == 0:
            encoder1A = GPIO.input(encoder_encoder1A)
        while encoder1A == 1:
            encoder1A = GPIO.input(encoder_encoder1A)
        counter1 += 1
        return
    else:
        return


def rotation_decode2(encoder_encoder2A):
    global counter2
    sleep(0.002)
    encoder2A = GPIO.input(encoder_encoder2A)
    encoder2B = GPIO.input(encoder_encoder2B)
    rospy.loginfo("2 A = %d"%(encoder2A))
    rospy.loginfo("2 B = %d"%(encoder2B))
 
    if (encoder2A == 1) and (encoder2B == 0):

        rospy.loginfo("tick2 = %f"%(counter2))
        while encoder2B == 0:
            encoder2B = GPIO.input(encoder_encoder2B)
        while encoder2B == 1:
            encoder2B = GPIO.input(encoder_encoder2B)
        counter2 -= 1
        return
 
    elif (encoder2A == 0) and (encoder2B == 1):
        
        rospy.loginfo("tick2 = %f"%(counter2))
        while encoder2A == 0:
            encoder2A = GPIO.input(encoder_encoder2A)
        while encoder2A == 1:
            encoder2A = GPIO.input(encoder_encoder2A)
        counter2 += 1
        return
    else:
        return

def rotation_decode3(encoder_encoder3A):
    global counter3
    sleep(0.002)
    encoder3A = GPIO.input(encoder_encoder3A)
    encoder3B = GPIO.input(encoder_encoder3B)
    rospy.loginfo("3 A = %d"%(encoder3A))
    rospy.loginfo("3 B = %d"%(encoder3B))
 
    if (encoder3A == 1) and (encoder3B == 0):

        rospy.loginfo("tick3 = %f"%(counter3))
        while encoder3B == 0:
            encoder3B = GPIO.input(encoder_encoder3B)
        while encoder3B == 1:
            encoder3B = GPIO.input(encoder_encoder3B)
        counter3 -= 1
        return
 
    elif (encoder3A == 0) and (encoder3B == 1):
        
        rospy.loginfo("tick3 = %f"%(counter3))
        while encoder3A == 0:
            encoder3A = GPIO.input(encoder_encoder3A)
        while encoder3A == 1:
            encoder3A = GPIO.input(encoder_encoder3A)
        counter3 += 1
        return
    else:
        return

def rotation_decode4(encoder_encoder4A):
    global counter4
    sleep(0.002)
    encoder4A = GPIO.input(encoder_encoder4A)
    encoder4B = GPIO.input(encoder_encoder4B)
    rospy.loginfo("4 A = %d"%(encoder4A))
    rospy.loginfo("4 B = %d"%(encoder4B))
 
    if (encoder4A == 1) and (encoder4B == 0):

        rospy.loginfo("tick4 = %f"%(counter4))
        while encoder4B == 0:
            encoder4B = GPIO.input(encoder_encoder4B)
        while encoder4B == 1:
            encoder4B = GPIO.input(encoder_encoder4B)
        counter4 -= 1
        return
 
    elif (encoder4A == 0) and (encoder4B == 1):
        
        rospy.loginfo("tick4 = %f"%(counter4))
        while encoder4A == 0:
            encoder4A = GPIO.input(encoder_encoder4A)
        while encoder4A == 1:
            encoder4A = GPIO.input(encoder_encoder4A)
        counter4 += 1
        return
    else:
        return

def main():
    pub = rospy.Publisher('encode', String, queue_size = 10)
    rospy.init_node('encode')
    try :
        init()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(str(counter1))
            rate.sleep()
    except KeyboardInterrupt :
        pass
    finally :
        GPIO.cleanup()
 
if __name__ == '__main__':
    main()
