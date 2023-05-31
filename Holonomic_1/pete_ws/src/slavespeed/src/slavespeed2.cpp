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

const double encoder_cpr = 360; 
const double radius = 0.076; 

volatile float pos1 = 0 ;
volatile float lastpos1 = 0 ;
volatile float lastlastpos1 = 0 ;
volatile float pos2 = 0 ;
volatile float lastpos2 = 0 ;
volatile float lastlastpos2 = 0 ;
volatile float pos3 = 0 ;
volatile float lastpos3 = 0 ;
volatile float lastlastpos3 = 0 ;
volatile float pos4 = 0 ;
volatile float lastpos4 = 0 ;
volatile float lastlastpos4 = 0 ;

ros::Time current_time;
double last_time = 0.0;
double interval;


void handle_pos1(const std_msgs::String::ConstPtr& handle_pos1) 
{
   
   pos1 = std::stof(handle_pos1->data.c_str());
   //ROS_INFO_STREAM("pos1 :"<<pos1);
   
}

void handle_pos2(const std_msgs::String::ConstPtr& handle_pos2) 
{
   
   pos2 = std::stof(handle_pos2->data.c_str());
   //ROS_INFO_STREAM("pos2 :"<<pos2);
   
}

void handle_pos3(const std_msgs::String::ConstPtr& handle_pos3) 
{
   
   pos3 = std::stof(handle_pos3->data.c_str());
   //ROS_INFO_STREAM("pos3 :"<<pos3);
   
}

void handle_pos4(const std_msgs::String::ConstPtr& handle_pos4) 
{
   
   pos4 = std::stof(handle_pos4->data.c_str());
   //ROS_INFO_STREAM("pos4 :"<<pos4);
   
}
void timerCallback(const ros::TimerEvent& event)
{
	lastlastpos1 = pos1 ;
	lastlastpos2 = pos2 ;
	lastlastpos3 = pos3 ;
	lastlastpos4 = pos4 ;
}
/*void timerCallback2(const ros::TimerEvent& event)
{
	if(abs(pos1 - lastpos1)!= 0)
	{
		speed_act1 = ((((pos1-lastpos1)/encoder_cpr)*2*3.14159265358979323846564)*(1/0.1)*radius);
		//ROS_INFO_STREAM("speed wheel 2 :"<<speed_act2);
		ROS_INFO_STREAM("tick wheel 2 :"<<(pos1-lastpos1));
		lastpos1 = pos1 ;
	}
	else
	{
		speed_act1 = 0 ;
		//ROS_INFO_STREAM("speed wheel 2 :"<<speed_act2);
	}
}
void timerCallback3(const ros::TimerEvent& event)
{
	if(abs(pos3 - lastpos3)!= 0)
	{
		speed_act3 = ((((pos3-lastpos3)/encoder_cpr)*2*3.14159265358979323846564)*(1/0.1)*radius);
		//ROS_INFO_STREAM("speed wheel 3 :"<<speed_act3);
		//ROS_INFO_STREAM("tick wheel 3 :"<<(pos3-lastpos3));
		lastpos3 = pos3 ;
	}
	else
	{
		speed_act3 = 0 ;
		//ROS_INFO_STREAM("speed wheel 3 :"<<speed_act3);
	}
}
void timerCallback4(const ros::TimerEvent& event)
{
	if(abs(pos4 - lastpos4)!= 0)
	{
		speed_act4 = ((((pos4-lastpos4)/encoder_cpr)*2*3.14159265358979323846564)*(1/0.1)*radius);
		//ROS_INFO_STREAM("speed wheel 4 :"<<speed_act4);
		//ROS_INFO_STREAM("tick wheel 4 :"<<(pos4-lastpos4));
		lastpos4 = pos4 ;
	}
	else
	{
		speed_act4 = 0 ;
		//ROS_INFO_STREAM("speed wheel 4 :"<<speed_act4);
	}
}*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slavespeed2");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("slaveencodetick1", 1000, handle_pos1);
	ros::Subscriber sub2 = n.subscribe("slaveencodetick2", 1000, handle_pos2);
	ros::Subscriber sub3 = n.subscribe("slaveencodetick3", 1000, handle_pos3);
	ros::Subscriber sub4 = n.subscribe("slaveencodetick4", 1000, handle_pos4);

	ros::Publisher slavespeed_pub = n.advertise<geometry_msgs::TwistStamped>("slavetickdif", 50);
	
	ros::Timer timer = n.createTimer(ros::Duration(0.1),timerCallback);
	//ros::Timer timer2 = n.createTimer(ros::Duration(0.1),timerCallback2);
	//ros::Timer timer3 = n.createTimer(ros::Duration(0.1),timerCallback3);
	//ros::Timer timer4 = n.createTimer(ros::Duration(0.1),timerCallback4);
	
	ros::Rate loop_rate(10);
	
	
	 while (ros::ok())
	{
		
		current_time = ros::Time::now();
		interval = current_time.toSec() - last_time ;
		
		if(abs(pos1 - lastlastpos1)!= 0)
		{
			speed_act1 = (pos1-lastlastpos1);
			ROS_INFO_STREAM("tick 1 diff :"<<speed_act1);
			//ROS_INFO_STREAM("tick wheel 1 :"<<(pos1-lastlastpos1));
			lastpos1 = pos1 ;
		}
		else
		{
			speed_act1 = 0 ;
			//ROS_INFO_STREAM("speed wheel 1 :"<<speed_act1);
		}
		
		if(abs(pos2 - lastlastpos2)!= 0)
		{
			speed_act2 = (pos2-lastlastpos2);
			ROS_INFO_STREAM("speed wheel 2 :"<<speed_act2);
			//ROS_INFO_STREAM("tick wheel 2 :"<<(pos2-lastpos2));
			lastpos2 = pos2 ;
		}
		else
		{
			speed_act2 = 0 ;
			//ROS_INFO_STREAM("speed wheel 2 :"<<speed_act2);
		}
		

		if(abs(pos3 - lastlastpos3)!= 0)
		{
			speed_act3 = (pos3-lastlastpos3);
			ROS_INFO_STREAM("speed wheel 3 :"<<speed_act3);
			//ROS_INFO_STREAM("tick wheel 3 :"<<(pos3-lastpos3));
			lastpos3 = pos3 ;
		}
		else
		{
			speed_act3 = 0 ;
			//ROS_INFO_STREAM("speed wheel 3 :"<<speed_act3);
		}
	
		if(abs(pos4 - lastlastpos4)!= 0)
		{
			speed_act4 = (pos4-lastlastpos4);
			ROS_INFO_STREAM("speed wheel 4 :"<<speed_act4);
			//ROS_INFO_STREAM("tick wheel 4 :"<<(pos4-lastpos4));
			lastpos4 = pos4 ;
		}
		else
		{
			speed_act4 = 0 ;
			//ROS_INFO_STREAM("speed wheel 4 :"<<speed_act4);
		}
		

		last_time = ros::Time::now().toSec() ;
		

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
