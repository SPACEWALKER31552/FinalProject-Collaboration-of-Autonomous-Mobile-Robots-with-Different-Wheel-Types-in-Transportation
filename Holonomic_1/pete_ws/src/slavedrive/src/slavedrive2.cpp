#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>

#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>

#define PI 3.14159265

double encode1 ;
double encode2 ;
double encode3 ;
double encode4 ;
ros::Time current_time ;

/*class OdometryPublisher
{
	public:
		OdometryPublisher();

	private:
		void encoderCallback(const geometry_msgs::TwistStamped& encoders);

		ros::NodeHandle nh;
		ros::Subscriber enc_sub;
		ros::Publisher odom_pub;
		tf::TransformBroadcaster odom_broadcaster;
		
		double x, y, th;
		double scale_x, scale_y, scale_th;
		bool calibration_mode;
};*/


/*OdometryPublisher::OdometryPublisher()
{
	// intialize integrators
	x = y = th = 0;
	
	// load parameters
    ros::NodeHandle nh_priv("~");
	nh_priv.param("scale_x", scale_x, 0.6283185 / 360.0 / 2.15); // loosely base off of 360 counts per rev, 100mm wheels (which is 0.6283185 / 360.0)
	nh_priv.param("scale_y", scale_y, 0.6283185 / 360.0 / 2.7);
	nh_priv.param("scale_th", scale_th, 0.0032);
	nh_priv.param("calibration_mode", calibration_mode, false);
	
	// lets show em what we got
	ROS_INFO_STREAM("param scale_x: " << scale_x);
	ROS_INFO_STREAM("param scale_y: " << scale_y);
	ROS_INFO_STREAM("param scale_th: " << scale_th);
	ROS_INFO_STREAM("param calibration_mode: " << calibration_mode);

	// connects subs and pubs
	enc_sub = nh.subscribe<geometry_msgs::TwistStamped>("slavetickdif", 10, &OdometryPublisher::encoderCallback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
}*/

/*void OdometryPublisher::encoderCallback(const geometry_msgs::TwistStamped& encoders)
{
	
	encode1 = encoders.twist.linear.x ;
	encode2 = encoders.twist.linear.y ;
	encode3 = encoders.twist.angular.x ;
	encode4 = encoders.twist.angular.y ;
	ros::Time current_time = encoders.header.stamp ;
	// unpack the encoder message in base_link frame
	double dx = ((encode1+encode2+encode3+encode4)/4) * scale_x;
	double dy = ((0-encode1+encode2+encode3-encode4)/4) * scale_y;
	double dth = ((0-encode1+encode2-encode3+encode4)/4) * scale_th;
	double dt = 0.1;
	
	// convert to movements in the odom frame
	if(calibration_mode) {
		x += dx;
		y += dy;
		th += dth;
		//th = fmod(th, 2*PI);
	}
    else {
		x += dx * cos(th) - dy * sin(th);
		y += dx * sin(th) + dy * cos(th);
		th += dth;
		th = fmod(th, 2*PI);
    }

	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	// first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_linkslave";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	if(calibration_mode) odom_trans.transform.translation.z = th; // need some way to check this w/o the quaternion
	else odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	// send the transform
	odom_broadcaster.sendTransform(odom_trans);

	// next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	// set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// set the velocity
	odom.child_frame_id = "base_linkslave";
	odom.twist.twist.linear.x = dx / dt;
	odom.twist.twist.linear.y = dy / dt;
	odom.twist.twist.angular.z = dth / dt;

	// publish the message
	odom_pub.publish(odom);
}*/

void encoderCallback(const geometry_msgs::TwistStamped& encoders) 
{
	encode1 = encoders.twist.linear.x ;
	encode2 = encoders.twist.linear.y ;
	encode3 = encoders.twist.angular.x ;
	encode4 = encoders.twist.angular.y ;
	current_time = encoders.header.stamp ;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "slavedrive2");
	ros::NodeHandle nh;
	tf::TransformBroadcaster odom_broadcaster;
		
	double x, y, th;
	double scale_x, scale_y, scale_th;
	bool calibration_mode;
	//OdometryPublisher odom;
	
	// intialize integrators
	x = y = th = 0;
	
	// load parameters
    ros::NodeHandle nh_priv("~");
	nh_priv.param("scale_x", scale_x, (2*PI*0.076) / 360.0 / 2.15); // loosely base off of 360 counts per rev, 100mm wheels (which is 0.6283185 / 360.0)
	nh_priv.param("scale_y", scale_y, (2*PI*0.076) / 360.0 / 2.7);
	nh_priv.param("scale_th", scale_th, 0.0032);
	nh_priv.param("calibration_mode", calibration_mode, false);
	
	// lets show em what we got
	ROS_INFO_STREAM("param scale_x: " << scale_x);
	ROS_INFO_STREAM("param scale_y: " << scale_y);
	ROS_INFO_STREAM("param scale_th: " << scale_th);
	ROS_INFO_STREAM("param calibration_mode: " << calibration_mode);

	// connects subs and pubs
	ros::Subscriber enc_sub = nh.subscribe("slavetickdif", 10, encoderCallback);
	//enc_sub = nh.subscribe<geometry_msgs::TwistStamped>("slavetickdif", 10,encoderCallback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("slaveodom", 50);
	//odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

	while(nh.ok())
	{
		ros::spinOnce();
		double dx = ((encode1+encode2+encode3+encode4)/4) * scale_x;
		double dy = ((0-encode1+encode2+encode3-encode4)/4) * scale_y;
		double dth = ((0-encode1+encode2-encode3+encode4)/4) * scale_th;
		double dt = 0.1;
	
		// convert to movements in the odom frame
		if(calibration_mode) {
			x += dx;
			y += dy;
			th += dth;
			//th = fmod(th, 2*PI);
		}
		else {
			x += dx * cos(th) - dy * sin(th);
			y += dx * sin(th) + dy * cos(th);
			th += dth;
			th = fmod(th, 2*PI);
		}

		// since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		// first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_linkslave";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		if(calibration_mode) odom_trans.transform.translation.z = th; // need some way to check this w/o the quaternion
		else odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		// send the transform
		odom_broadcaster.sendTransform(odom_trans);

		// next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		// set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		// set the velocity
		odom.child_frame_id = "base_linkslave";
		odom.twist.twist.linear.x = dx / dt;
		odom.twist.twist.linear.y = dy / dt;
		odom.twist.twist.angular.z = dth / dt;

		// publish the message
		odom_pub.publish(odom);
		
		// lets show em what we got
		ROS_INFO_STREAM("x: " << x);
		ROS_INFO_STREAM("y: " << y);
	
	}
}
