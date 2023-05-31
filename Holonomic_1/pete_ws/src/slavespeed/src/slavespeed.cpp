#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

double speed_act1 = 0; 
double speed_act2 = 0; 
double speed_act3 = 0; 
double speed_act4 = 0;  

void handle_speed1(const std_msgs::String::ConstPtr& handle_speed1) 
{
   
   speed_act1 = std::stof(handle_speed1->data.c_str());
   //ROS_INFO_STREAM("pos1 :"<<pos1);
   
}

void handle_speed2(const std_msgs::String::ConstPtr& handle_speed2) 
{
   
   speed_act2 = std::stof(handle_speed2->data.c_str());
   //ROS_INFO_STREAM("pos2 :"<<pos2);
   
}

void handle_speed3(const std_msgs::String::ConstPtr& handle_speed3) 
{
   
   speed_act3 = std::stof(handle_speed3->data.c_str());
   //ROS_INFO_STREAM("pos3 :"<<pos3);
   
}

void handle_speed4(const std_msgs::String::ConstPtr& handle_speed4) 
{
   
   speed_act4 = std::stof(handle_speed4->data.c_str());
   //ROS_INFO_STREAM("pos4 :"<<pos4);
   
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slavespeed");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("slaveencode1", 1000, handle_speed1);
	ros::Subscriber sub2 = n.subscribe("slaveencode2", 1000, handle_speed2);
	ros::Subscriber sub3 = n.subscribe("slaveencode3", 1000, handle_speed3);
	ros::Subscriber sub4 = n.subscribe("slaveencode4", 1000, handle_speed4);

	ros::Publisher slavespeed_pub = n.advertise<geometry_msgs::TwistStamped>("slavespeed", 50);
	
	//ros::Timer timer = n.createTimer(ros::Duration(0.1),timerCallback);
	//ros::Timer timer2 = n.createTimer(ros::Duration(0.1),timerCallback2);
	//ros::Timer timer3 = n.createTimer(ros::Duration(0.1),timerCallback3);
	//ros::Timer timer4 = n.createTimer(ros::Duration(0.1),timerCallback4);
	
	ros::Rate loop_rate(10);
	
	
	 while (ros::ok())
	{
		
		speed_act1 = trunc(speed_act1*100)/100;
		speed_act2 = trunc(speed_act2*100)/100;
		speed_act3 = trunc(speed_act3*100)/100;
		speed_act4 = trunc(speed_act4*100)/100;

		geometry_msgs::TwistStamped msg ;
		msg.header.stamp = ros::Time::now();
		msg.twist.linear.x = speed_act1 ;
		msg.twist.linear.y = speed_act2 ;
		msg.twist.angular.x = speed_act3 ;
		msg.twist.angular.y = speed_act4 ;


		slavespeed_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		
		
	}


	return 0;
}
