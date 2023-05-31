#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

double linx = 0;
double liny = 0;
double linz = 0; 

double angx = 0;
double angy = 0;
double angz = 0; 

void handle_speed1(const geometry_msgs::Twist& msg) 
{
   
   linx = msg.linear.x ;
   liny = msg.linear.y ;
   linz = msg.linear.z ;
   angx = msg.angular.x ;
   angy = msg.angular.y ;
   angz = msg.angular.z ;
   
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odomspeed");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("cmd_vel", 1000, handle_speed1);
	ros::Publisher slavespeed_pub = n.advertise<geometry_msgs::TwistStamped>("odomspeed", 50);
	
	//ros::Timer timer = n.createTimer(ros::Duration(0.1),timerCallback);
	//ros::Timer timer2 = n.createTimer(ros::Duration(0.1),timerCallback2);
	//ros::Timer timer3 = n.createTimer(ros::Duration(0.1),timerCallback3);
	//ros::Timer timer4 = n.createTimer(ros::Duration(0.1),timerCallback4);
	
	ros::Rate loop_rate(10);
	
	
	 while (ros::ok())
	{
		geometry_msgs::TwistStamped msg ;
		msg.header.stamp = ros::Time::now();
		msg.twist.linear.x = linx ;
		msg.twist.linear.y = liny ;
		msg.twist.linear.z = linz ;
		msg.twist.angular.x = angx ;
		msg.twist.angular.y = angy ;
		msg.twist.angular.z = angz ;

		slavespeed_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		
		
	}


	return 0;
}
