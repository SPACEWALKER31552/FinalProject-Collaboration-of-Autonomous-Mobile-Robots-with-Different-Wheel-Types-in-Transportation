#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>


#define LOOPTIME                      100     //Looptime in millisecond
unsigned int noCommLoops = 0;                 //main loop without communication counter

unsigned long lastMilli = 0;

//--- Robot-specific constants ---            
const double radius = 0.076;                   //Wheel radius, in m
double wheelbase = 0.55;                          //Wheelbase, in m
double two_pi = 6.28319;
const double encoder_cpr = 360;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.0035;    // 0.00235 Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.09;          //0.0882 (min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_act1 = 0; 
double speed_act2 = 0; 
double speed_act3 = 0; 
double speed_act4 = 0;            
                 
const double max_speed = 0.4;                 //Max speed in m/s 
         
volatile float pos1 = 0 ;
volatile float lastpos1 = 0 ;
volatile float pos2 = 0 ;
volatile float lastpos2 = 0 ;
volatile float pos3 = 0 ;
volatile float lastpos3 = 0 ;
volatile float pos4 = 0 ;
volatile float lastpos4 = 0 ;


double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_dt = 0.0;
double x_pos = 0.76;
double y_pos = -0.565;
double theta = 0.0;

double imunew = 0.0 ;
double imuold = 0.0 ;
ros::Time current_time;
ros::Time speed_time(0.0);


void handle_speed(const geometry_msgs::TwistStamped& handle_speed) 
{
    speed_act1 = handle_speed.twist.linear.x ;
	speed_act2 = handle_speed.twist.linear.y ;
	speed_act3 = handle_speed.twist.angular.x ;
	speed_act4 = handle_speed.twist.angular.y ;
	speed_time = handle_speed.header.stamp ;
}
void handle_imu(const geometry_msgs::Vector3Stamped& imu)
{
	imunew = (imu.vector.x)*two_pi/360 ;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slave2drive");
	ros::NodeHandle n;
	ros::NodeHandle nh_private_("~");
	ros::Subscriber subspeed = n.subscribe("slave2speed", 1000, handle_speed);
	ros::Subscriber subimu = n.subscribe("slave2IMU",1000,handle_imu);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("slave2odom", 50);
	tf::TransformBroadcaster broadcaster;  
	
	double rate = 10.0;
	double linear_scale_positive = 1.0;
	double linear_scale_negative = 1.0;
	double angular_scale_positive = 1.0;
	double angular_scale_negative = 1.0;
	bool publish_tf = true;
	double dt = 0.0;
	double dx = 0.0;
	double dy = 0.0;
	double dth = 0.0;
	double dxy = 0.0;
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	char base_link[] = "/base_linkslave2";
	char odom[] = "/odom";
	
	speed_dt = 0.1;
	
	ros::Duration d(1.0);
	nh_private_.getParam("publish_rate", rate);
	nh_private_.getParam("publish_tf", publish_tf);
	nh_private_.getParam("linear_scale_positive", linear_scale_positive);
	nh_private_.getParam("linear_scale_negative", linear_scale_negative);
	nh_private_.getParam("angular_scale_positive", angular_scale_positive);
	nh_private_.getParam("angular_scale_negative", angular_scale_negative);

	ros::Rate r(rate);
	while(n.ok())
	{
		ros::spinOnce();
		current_time = speed_time;
		dt = speed_dt;					//Time in s
		//ROS_INFO("dt : %f", dt);
		dx = (speed_act1+speed_act2+speed_act3+speed_act4)*dt/4;
		dy = (speed_act2-speed_act1+speed_act3-speed_act4)*dt/4;
		//ROS_INFO("dx : %f", dx);
		//ROS_INFO("dy : %f", dy);
		//dth = ((speed_act2-speed_act1-speed_act3+speed_act4)*dt)/wheelbase;
		dth = imunew - imuold ;
		if (dth > 0) dth *= angular_scale_positive;
		if (dth < 0) dth *= angular_scale_negative;
		if (dx > 0) dx *= linear_scale_positive;
		if (dx < 0) dx *= linear_scale_negative;
		if (dy > 0) dy *= linear_scale_positive;
		if (dy < 0) dy *= linear_scale_negative;

		//dx = cos(dth) * dxy;
		//dy = sin(dth) * dxy;
		
		x_pos += (cos(theta) * dx - sin(theta) * dy);
		y_pos += (sin(theta) * dx + cos(theta) * dy);
		theta += dth;

		if(theta >= two_pi) theta -= two_pi;
		if(theta <= -two_pi) theta += two_pi;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
		geometry_msgs::Quaternion empty_quat = tf::createQuaternionMsgFromYaw(0);
		
		if(publish_tf) 
		{
			geometry_msgs::TransformStamped t;
      
			t.header.frame_id = odom;
			t.child_frame_id = base_link;
			t.transform.translation.x = x_pos;
			t.transform.translation.y = y_pos;
			t.transform.translation.z = 0.0;
			t.transform.rotation = odom_quat;
			t.header.stamp = current_time;
			
			broadcaster.sendTransform(t);
		}
		
		nav_msgs::Odometry odom_msg;
		geometry_msgs::Pose myodom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = odom;
		odom_msg.pose.pose.position.x = x_pos;
		odom_msg.pose.pose.position.y = y_pos;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;
		
		
		if (speed_act1 == 0 && speed_act2 == 0 && speed_act3 == 0 && speed_act4 == 0) 
		{
		odom_msg.pose.covariance[0] = 1e-9;
		odom_msg.pose.covariance[7] = 1e-3;
		odom_msg.pose.covariance[8] = 1e-9;
		odom_msg.pose.covariance[14] = 1e6;
		odom_msg.pose.covariance[21] = 1e6;
		odom_msg.pose.covariance[28] = 1e6;
		odom_msg.pose.covariance[35] = 1e-9;
		odom_msg.twist.covariance[0] = 1e-9;
		odom_msg.twist.covariance[7] = 1e-3;
		odom_msg.twist.covariance[8] = 1e-9;
		odom_msg.twist.covariance[14] = 1e6;
		odom_msg.twist.covariance[21] = 1e6;
		odom_msg.twist.covariance[28] = 1e6;
		odom_msg.twist.covariance[35] = 1e-9;
		}
		else{
		odom_msg.pose.covariance[0] = 1e-3;
		odom_msg.pose.covariance[7] = 1e-3;
		odom_msg.pose.covariance[8] = 0.0;
		odom_msg.pose.covariance[14] = 1e6;
		odom_msg.pose.covariance[21] = 1e6;
		odom_msg.pose.covariance[28] = 1e6;
		odom_msg.pose.covariance[35] = 1e3;
		odom_msg.twist.covariance[0] = 1e-3;
		odom_msg.twist.covariance[7] = 1e-3;
		odom_msg.twist.covariance[8] = 0.0;
		odom_msg.twist.covariance[14] = 1e6;
		odom_msg.twist.covariance[21] = 1e6;
		odom_msg.twist.covariance[28] = 1e6;
		odom_msg.twist.covariance[35] = 1e3;
		}
		
		vx = (dt == 0)?  0 : (speed_act1+speed_act2+speed_act3+speed_act4)/4;
		vth = (dt == 0)? 0 : (speed_act2-speed_act1-speed_act3+speed_act4)/wheelbase;
		odom_msg.child_frame_id = base_link;
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.linear.y = 0.0;
		odom_msg.twist.twist.angular.z = dth;

		odom_pub.publish(odom_msg);
    		imuold = imunew ;
		r.sleep();
	}	
}	
