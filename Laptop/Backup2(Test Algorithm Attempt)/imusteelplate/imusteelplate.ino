/* Get tilt angles on X and Y, and rotation angle on Z
    Angles are given in degrees
 License: MIT
 */
 #include <Wire.h>
//#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <MPU6050_light.h>

double imux = 0;
double imuy = 0;
double imuz = 0;



MPU6050 mpu(Wire);
unsigned long timer = 0;

//ros::NodeHandle nh;
//geometry_msgs::Vector3Stamped imusteelplate;                                
//ros::Publisher imusteelplate_msg("imusteelplate", &imusteelplate); 


void setup() 
{
   Wire.begin();
   byte status = mpu.begin();
   mpu.calcOffsets();
   Serial.begin(115200);
   //nh.initNode();                            //init ROS node
   //nh.getHardware()->setBaud(115200);
   //nh.advertise(imusteelplate_msg);

}
void loop() 
{
  //nh.spinOnce();
   if ((millis() - timer) > 100) 
   { // print data every 10ms
     //nh.spinOnce();
     delay(10);
     byte status = mpu.begin();
     //imusteelplate.vector.x = mpu.getAngleX();
     //imusteelplate.vector.y = mpu.getAngleY();
     //imusteelplate.vector.z = mpu.getAngleZ();
     //imusteelplate_msg.publish(&imusteelplate);
     Serial.print("X : ");
     Serial.print(mpu.getAngleX());
     Serial.print("\tY : ");
     Serial.print(mpu.getAngleY());
     Serial.print("\tZ : ");
     Serial.println(mpu.getAngleZ());
     timer = millis();
   }
}
