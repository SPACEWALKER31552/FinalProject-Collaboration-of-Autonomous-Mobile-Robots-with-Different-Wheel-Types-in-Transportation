#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
// define varialble
unsigned long lastmillis = 0;
double currentzeta = 0;
double leaderzeta = 0.0;
double slavezeta = 0.0;
double slave2zeta = 0.0;
double uslave = 0.0;
double uslave2 = 0.0;
double uleader = 0.0;

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 0;
double leaderturnicomponent = 0;
double leaderturnjcomponent = 0;
double leaderturnkcomponent = 0;

double slaveturnicomponent = 0;
double slaveturnjcomponent = 0;
double slaveturnkcomponent = 0;

double slave2turnicomponent = 0;
double slave2turnjcomponent = 0;
double slave2turnkcomponent = 0;

// coordinate for master
double leaderRx = 0;
double leaderRy = 0;
double leaderRz = 0;

// coordinate for slave
double slaveRx = -0.565;
double slaveRy = 1.6;
double slaveRz = 0;

// coordinate for slave2
double slave2Rx = 0.565;
double slave2Ry = 1.6;
double slave2Rz = 0;

double newzeta = 0.0;

#define Looptime 200 // millisec
int sec = 1;
#define l 1.37 // 1.697
#define Pi 3.14159
#define alpha1 2.514
#define alpha2 3.768515
#define alpha3 0
#define d1 0.963133
#define d2 0.963133
#define d3 0.82

double startz = 0.0;
double start2z = 0.0;
double slave2diff = 0.0;
double slavediff = 0.0;
double r = 1.37; // 1.696827923 ;
// double r = 3 ;
double slave2x = 0.0;
double slave2y = 0.0;
double slavex = 0.0;
double slavey = 0.0;
unsigned long timer = 0;
long prevT = 0;
float eprevslave = 0;
float eintegralslave = 0;
double goalangleslave = 0.0;

long prevTslave2 = 0;
float eprevslave2 = 0;
float eintegralslave2 = 0;
double goalangleslave2 = 0.0;

long prevTleader = 0;
float eprevleader = 0;
float eintegralleader = 0;
double goalangleleader = 0.0;
double goalangleleaderturn = 0.0;
double goalangleleaderturnold = 0.0;

int desdegree = 0;

void handle_cmd(const geometry_msgs::Twist &cmd_vel)
{ // Reset the counter for number of main loops without communication

	linx = cmd_vel.linear.x;
	liny = cmd_vel.linear.y;
	linz = cmd_vel.linear.z;
	angx = cmd_vel.angular.x;
	angy = cmd_vel.angular.y;
	angz = cmd_vel.angular.z;
}
void handle_leader(const geometry_msgs::Vector3Stamped &imu)
{ // Reset the counter for number of main loops without communication

	leaderzeta = imu.vector.x;
}
void handle_slave(const geometry_msgs::Vector3Stamped &imu)
{ // Reset the counter for number of main loops without communication

	slavezeta = imu.vector.x;
}
void handle_slave2(const geometry_msgs::Vector3Stamped &imu)
{ // Reset the counter for number of main loops without communication

	slave2zeta = imu.vector.x;
}
void handle_degree(const geometry_msgs::Vector3Stamped &degree)
{ // Reset the counter for number of main loops without communication

	desdegree = degree.vector.x;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "masterkinematic2");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private_("~");

	ros::Subscriber cmd = nh.subscribe("cmd_vel", 50, handle_cmd);
	ros::Subscriber imu = nh.subscribe("slaveIMU", 1000, handle_slave);
	ros::Subscriber imuu = nh.subscribe("slave2IMU", 1000, handle_slave2);
	ros::Subscriber imuleader = nh.subscribe("leaderIMU", 1000, handle_leader);
	ros::Subscriber degree = nh.subscribe("destinationdegree", 50, handle_degree);

	ros::Publisher follower1velocity_msg = nh.advertise<geometry_msgs::Twist>("holo1/cmd_vel", 1000);
	ros::Publisher follower2velocity_msg = nh.advertise<geometry_msgs::Twist>("holo2/cmd_vel", 1000);
	ros::Publisher follower3velocity_msg = nh.advertise<geometry_msgs::Twist>("holo3/cmd_vel", 1000);
	ros::Publisher masterkinematicleader_msg = nh.advertise<geometry_msgs::Vector3Stamped>("leaderposition", 1000);
	ros::Publisher masterkinematicslave_msg = nh.advertise<geometry_msgs::Vector3Stamped>("slaveposition", 1000);
	ros::Publisher masterkinematicslave2_msg = nh.advertise<geometry_msgs::Vector3Stamped>("slave2position", 1000);
	ros::Publisher IMUprogram_msg = nh.advertise<geometry_msgs::Vector3Stamped>("IMUprogram", 1000);

	ros::Rate ra(10); // 10 hz
	geometry_msgs::Twist follower1velocity;
	geometry_msgs::Twist follower2velocity;
	geometry_msgs::Twist follower3velocity;
	geometry_msgs::Vector3Stamped masterkinematicleader;
	geometry_msgs::Vector3Stamped masterkinematicslave;
	geometry_msgs::Vector3Stamped masterkinematicslave2;
	geometry_msgs::Vector3Stamped IMUprogram;
	while (nh.ok())
	{
		ros::spinOnce();
		if (angz != 0 && linx == 0)
		{
			if (startz == 0)
			{
				startz = slavezeta;
			}
			if (start2z == 0)
			{
				start2z = slave2zeta;
			}
			else
			{
			}

			// slave2 --------------------------------------------------------------------------------------

			if ((slave2zeta - start2z + 26.19) > 0) // 19.45
			{
				slave2diff = slave2zeta - start2z + 26.19;
				// Serial.println(slave2diff);
				if (slave2diff > 360)
				{
					slave2diff = int(round(slave2diff)) % 360;
				}
				if (slave2diff >= 270)
				{
					// quotant 4
					slave2diff = slave2diff - 270;
					slave2x = r * cos(slave2diff * 3.14 / 180);
					slave2y = r * sin(slave2diff * 3.14 / 180);
					slave2x = -1 * abs(slave2x);
					slave2y = -1 * abs(slave2y);
				}
				else if (slave2diff >= 180)
				{
					// quotant 3
					slave2diff = slave2diff - 180;
					slave2x = r * sin(slave2diff * 3.14 / 180);
					slave2y = r * cos(slave2diff * 3.14 / 180);
					slave2x = -1 * abs(slave2x);
					slave2y = abs(slave2y);
				}
				else if (slave2diff >= 90)
				{
					// quotant 2
					slave2diff = slave2diff - 90;
					slave2x = r * cos(slave2diff * 3.14 / 180);
					slave2y = r * sin(slave2diff * 3.14 / 180);
					slave2x = abs(slave2x);
					slave2y = abs(slave2y);
				}
				else if (slave2diff < 90)
				{
					// quotant 1
					slave2x = r * sin(slave2diff * 3.14 / 180);
					slave2y = r * cos(slave2diff * 3.14 / 180);
					slave2x = abs(slave2x);
					slave2y = -1 * abs(slave2y);
				}
			}
			else if ((slave2zeta - start2z + 26.19) < 0)
			{
				slave2diff = slave2zeta - start2z + 26.19;
				// Serial.println(slave2diff);
				if (slave2diff < -360)
				{
					slave2diff = int(round(slave2diff)) % 360;
				}
				if (slave2diff <= -270)
				{
					// quotant -4
					slave2diff = slave2diff + 270;
					slave2x = r * cos(slave2diff * 3.14 / 180);
					slave2y = r * sin(slave2diff * 3.14 / 180);
					slave2x = abs(slave2x);
					slave2y = -1 * abs(slave2y);
				}
				else if (slave2diff <= -180)
				{
					// quotant -3
					slave2diff = slave2diff + 180;
					slave2x = r * sin(slave2diff * 3.14 / 180);
					slave2y = r * cos(slave2diff * 3.14 / 180);
					slave2x = abs(slave2x);
					slave2y = abs(slave2y);
				}
				else if (slave2diff <= -90)
				{
					// quotant -2
					slave2diff = slave2diff + 90;
					slave2x = r * cos(slave2diff * 3.14 / 180);
					slave2y = r * sin(slave2diff * 3.14 / 180);
					slave2x = -1 * abs(slave2x);
					slave2y = abs(slave2y);
				}
				else if (slave2diff > -90)
				{
					// quotant -1
					slave2x = r * sin(slave2diff * 3.14 / 180);
					slave2y = r * cos(slave2diff * 3.14 / 180);
					slave2x = -1 * abs(slave2x);
					slave2y = -1 * abs(slave2y);
				}
			}

			// slave --------------------------------------------------------------------------------------
			if ((slavezeta - startz - 26.19) > 0)
			{
				slavediff = slavezeta - startz - 26.19;
				// Serial.println(slavediff);
				if (slavediff > 360)
				{
					slavediff = slavediff - 360;
				}

				if (slavediff >= 270)
				{
					// quotant 4
					slavediff = slavediff - 270;
					slavex = r * cos(slavediff * 3.14 / 180);
					slavey = r * sin(slavediff * 3.14 / 180);
					slavex = -1 * abs(slavex);
					slavey = -1 * abs(slavey);
				}
				else if (slavediff >= 180)
				{
					// quotant 3
					slavediff = slavediff - 180;
					slavex = r * sin(slavediff * 3.14 / 180);
					slavey = r * cos(slavediff * 3.14 / 180);
					slavex = -1 * abs(slavex);
					slavey = abs(slavey);
				}
				else if (slavediff >= 90)
				{
					// quotant 2
					slavediff = slavediff - 90;
					slavex = r * cos(slavediff * 3.14 / 180);
					slavey = r * sin(slavediff * 3.14 / 180);
					slavex = abs(slavex);
					slavey = abs(slavey);
				}
				else if (slavediff < 90)
				{
					// quotant 1
					slavex = r * sin(slavediff * 3.14 / 180);
					slavey = r * cos(slavediff * 3.14 / 180);
					slavex = abs(slavex);
					slavey = -1 * abs(slavey);
				}
			}
			else if ((slavezeta - startz - 26.19) < 0)
			{
				slavediff = slavezeta - startz - 26.19;
				// Serial.println(slavediff);
				if (slavediff < -360)
				{
					slavediff = slavediff + 360;
				}

				if (slavediff <= -270)
				{
					// quotant -4
					slavediff = slavediff + 270;
					slavex = r * cos(slavediff * 3.14 / 180);
					slavey = r * sin(slavediff * 3.14 / 180);
					slavex = abs(slavex);
					slavey = -1 * abs(slavey);
				}
				else if (slavediff <= -180)
				{
					// quotant -3
					slavediff = slavediff + 180;
					slavex = r * sin(slavediff * 3.14 / 180);
					slavey = r * cos(slavediff * 3.14 / 180);
					slavex = abs(slavex);
					slavey = abs(slavey);
				}
				else if (slavediff <= -90)
				{
					// quotant -2
					slavediff = slavediff + 90;
					slavex = r * cos(slavediff * 3.14 / 180);
					slavey = r * sin(slavediff * 3.14 / 180);
					slavex = -1 * abs(slavex);
					slavey = abs(slavey);
				}
				else if (slavediff > -90)
				{
					// quotant -1
					slavex = r * sin(slavediff * 3.14 / 180);
					slavey = r * cos(slavediff * 3.14 / 180);
					slavex = -1 * abs(slavex);
					slavey = -1 * abs(slavey);
				}
			}

			slaveturnicomponent = (slaveRz * angy) - (slavey * angz);
			slaveturnjcomponent = (slavex * angz) - (slaveRz * angx);
			slaveturnkcomponent = (slavey * angx) - (slavex * angy);

			// cross to find omega component slave2
			slave2turnicomponent = (slave2Rz * angy) - (slave2y * angz);
			slave2turnjcomponent = (slave2x * angz) - (slave2Rz * angx);
			slave2turnkcomponent = (slave2y * angx) - (slave2x * angy);

			// cross to find omega component master
			leaderturnicomponent = (leaderRz * angy) - (leaderRy * angz);
			leaderturnjcomponent = (leaderRx * angz) - (leaderRz * angx);
			leaderturnkcomponent = (leaderRy * angx) - (leaderRx * angy);

			masterkinematicslave.vector.x = slavex;
			masterkinematicslave.vector.y = slavey;
			masterkinematicslave.vector.z = int((slavezeta - startz)) % 90;

			masterkinematicslave2.vector.x = slave2x;
			masterkinematicslave2.vector.y = slave2y;
			masterkinematicslave2.vector.z = int((slave2zeta - start2z)) % 90;

			if (goalangleslave == 0)
			{
				goalangleslave = slavezeta;
			}
			if (goalangleleader == 0)
			{
				goalangleleader = leaderzeta;
			}

			int signI = 0;
			if (angz > 0)
			{
				signI = -1;
			}
			else
			{
				signI = 1;
			}
			follower1velocity.linear.x = 0;
			follower1velocity.linear.y = 0;
			follower1velocity.angular.z = signI * 1.5; //1.8// 1.37*uleader;
		
			follower1velocity_msg.publish(follower1velocity);
		
			// slave-------------------------------------------	--------------
			// follower2velocity.linear.x = linx +slaveturnjcomponent+ (slaveturnjcomponent*cos((int((slavezeta - startz))%90)*3.14/180));
			// follower2velocity.linear.y = -slaveturnicomponent+(-slaveturnicomponent*cos((int((slavezeta - startz))%90)*3.14/180));
			
			if ((slavezeta - startz) < 0 && (slavezeta - startz) >= -90)
			{
				follower2velocity.linear.x = -linx - (slaveturnjcomponent * abs(cos((int(abs(slavezeta - startz)) % 90) * 3.14 / 180))) - (slaveturnicomponent * abs(sin((int(abs(slavezeta - startz)) % 90) * 3.14 / 180)));
				follower2velocity.linear.y = -(slaveturnicomponent * abs(cos((int(abs(slavezeta - startz)) % 90) * 3.14 / 180))) + (slaveturnjcomponent * abs(sin((int(abs(slavezeta - startz)) % 90) * 3.14 / 180)));
				follower2velocity.angular.z = -angz / 7;
			}
			

			else if ((slavezeta - startz) <= 90 && (slavezeta - startz) >= 0)
			{
				follower2velocity.linear.x = -linx + (slaveturnjcomponent * abs(cos((int((slavezeta - startz)) % 90) * 3.14 / 180))) - (slaveturnicomponent * abs(sin((int((slavezeta - startz)) % 90) * 3.14 / 180)));
				follower2velocity.linear.y = -(slaveturnicomponent * abs(cos((int((slavezeta - startz)) % 90) * 3.14 / 180))) - (slaveturnjcomponent * abs(sin((int((slavezeta - startz)) % 90) * 3.14 / 180)));
				follower2velocity.angular.z = -angz / 7;
			}

			else
			{
				follower2velocity.linear.x = 0;
				follower2velocity.linear.y = 0;
				follower2velocity.angular.z = 0;
			}

			// slave2---------------------------------------------------------
			if ((slave2zeta - start2z) < 0 && (slave2zeta - start2z) >= -90)
			{
				follower3velocity.linear.x = -linx + (slave2turnjcomponent * abs(cos((int(abs(slave2zeta - start2z)) % 90) * 3.14 / 180))) + (slave2turnicomponent * abs(sin((int(abs(slave2zeta - start2z)) % 90) * 3.14 / 180)));
				follower3velocity.linear.y = -(slave2turnicomponent * abs(cos((int(abs(slave2zeta - start2z)) % 90) * 3.14 / 180))) + (slave2turnjcomponent * abs(sin((int(abs(slave2zeta - start2z)) % 90) * 3.14 / 180)));
				follower3velocity.angular.z = -angz / 8;
			}
			
			else if ((slave2zeta - start2z) <= 90 && (slave2zeta - start2z) >= 0)
			{
				follower3velocity.linear.x = -linx - (slave2turnjcomponent * abs(cos((int((slave2zeta - start2z)) % 90) * 3.14 / 180))) + (slave2turnicomponent * abs(sin((int((slave2zeta - start2z)) % 90) * 3.14 / 180)));
				follower3velocity.linear.y = -(slave2turnicomponent * abs(cos((int((slave2zeta - start2z)) % 90) * 3.14 / 180))) - (slave2turnjcomponent * abs(sin((int((slave2zeta - start2z)) % 90) * 3.14 / 180)));
				follower3velocity.angular.z = -angz / 8;
			}
		
			else
			{
				follower3velocity.linear.x = 0;
				follower3velocity.linear.y = 0;
				follower3velocity.angular.z = 0;
			}
		}

		else if (linx != 0 && angz == 0)
		{

			if (linx != 0)
			{
				if (goalangleslave == 0)
				{
					goalangleslave = slavezeta;
				}
				if (goalangleslave2 == 0)
				{
					goalangleslave2 = slave2zeta;
				}
				if (goalangleleader == 0)
				{
					goalangleleader = leaderzeta;
				}
			}
			else
			{
			}

			// PID constants
			float kp = 0.02;
			float kd = 0.000;
			float ki = 0.00;

			// time difference
			long currT = ros::Time::now().toSec();
			float deltaT = 0.01;
			prevT = currT;

			// errorleader
			int eleader = goalangleleader - leaderzeta;

			// derivative
			float dedtleader = (eleader - eprevleader) / (deltaT);

			// integral
			eintegralleader = eintegralleader + eleader * deltaT;

			// control signal
			float uleader = 0.3 * eleader + kd * dedtleader + 0 * eintegralleader;

			// store previous error
			eprevleader = eleader;

			// error
			int eslave = goalangleslave - slavezeta;

			// derivative
			float dedtslave = (eslave - eprevslave) / (deltaT);

			// integral
			eintegralslave = eintegralslave + eslave * deltaT;

			// control signal
			float uslave = kp * eslave + kd * dedtslave + ki * eintegralslave;

			// store previous error
			eprevslave = eslave;

			// error2
			int eslave2 = goalangleslave2 - slave2zeta;

			// derivative2
			float dedtslave2 = (eslave2 - eprevslave2) / (deltaT);

			// integral2
			eintegralslave2 = eintegralslave2 + eslave2 * deltaT;

			// control signal2
			float uslave2 = 0.05 * eslave2 + kd * dedtslave2 + ki * eintegralslave2;

			// store previous error
			eprevslave2 = eslave2;

			follower1velocity.linear.x = 1.5 * linx;
			follower1velocity.angular.z = uleader;
			follower1velocity_msg.publish(follower1velocity);

			follower2velocity.linear.x = linx;
			follower2velocity.angular.z = uslave;

			follower3velocity.linear.x = linx;
			follower3velocity.angular.z = 0 ;//uslave2;
		}
		else
		{
			startz = 0.0;
			start2z = 0.0;

			leaderturnicomponent = 0;
			leaderturnjcomponent = 0;
			leaderturnkcomponent = 0;

			slaveturnicomponent = 0;
			slaveturnjcomponent = 0;
			slaveturnkcomponent = 0;

			slave2turnicomponent = 0;
			slave2turnjcomponent = 0;
			slave2turnkcomponent = 0;
		}
		if (desdegree != 0)
		{
			// PID constants
			float kp = 0.02;
			float kd = 0.0001;
			float ki = 0.00;

			// time difference
			long currT = ros::Time::now().toSec();
			float deltaT = 0.01;
			prevT = currT;
			
			
			// errorleader
			int eleader = desdegree - leaderzeta;

			// derivative
			float dedtleader = (eleader - eprevleader) / (deltaT);

			// integral
			eintegralleader = eintegralleader + eleader * deltaT;

			// control signal
			float uleader = 0.2 * eleader + 0.0 * dedtleader + 0.0 * eintegralleader;

			// store previous error
			eprevleader = eleader;



			// error
			int eslave = desdegree - slavezeta;

			// derivative
			float dedtslave = (eslave - eprevslave) / (deltaT);

			// integral
			eintegralslave = eintegralslave + eslave * deltaT;

			// control signal
			double uslave = (kp * eslave) + (kd * dedtslave) + (ki * eintegralslave);

			// store previous error
			eprevslave = eslave;



			// error2
			int eslave2 = desdegree - slave2zeta;

			// derivative2
			float dedtslave2 = (eslave2 - eprevslave2) / (deltaT);

			// integral2
			eintegralslave2 = eintegralslave2 + eslave2 * deltaT;

			// control signal2
			float uslave2 = kp * eslave2 + kd * dedtslave2 + ki * eintegralslave2;

			// store previous error
			eprevslave2 = eslave2;


			follower1velocity.linear.x = 0;
			follower1velocity.linear.y = 0;
			follower1velocity.angular.z = uleader;
			follower1velocity_msg.publish(follower1velocity);

			follower2velocity.linear.x = 0;
			follower2velocity.linear.y = 0;
			follower2velocity.angular.z = uslave;

			follower3velocity.linear.x = 0;
			follower3velocity.linear.y = 0;
			follower3velocity.angular.z = uslave2;
		}
		else if (linx == 0 && angz == 0)
		{
			goalangleleader = 0;
			goalangleslave = 0;
			goalangleslave2 = 0;

			uslave = 0;
			uslave2 = 0;
			uleader = 0;

			follower1velocity.linear.x = 0;
			follower1velocity.linear.y = 0;
			follower1velocity.angular.z = 0;
			follower1velocity_msg.publish(follower1velocity);

			follower2velocity.linear.x = 0;
			follower2velocity.linear.y = 0;
			follower2velocity.angular.z = 0;

			follower3velocity.linear.x = 0;
			follower3velocity.linear.y = 0;
			follower3velocity.angular.z = 0;
		}
		
		IMUprogram.vector.x = leaderzeta ;//goalangleleader;
		IMUprogram.vector.y = linx ;//goalangleslave;
		IMUprogram.vector.z = uleader ;//desdegree;
		IMUprogram_msg.publish(IMUprogram);
		follower1velocity_msg.publish(follower1velocity);
		follower2velocity_msg.publish(follower2velocity);
		follower3velocity_msg.publish(follower3velocity);
		// masterkinematicslave_msg.publish(&masterkinematicslave);
		// masterkinematicslave2_msg.publish(&masterkinematicslave2);
		ra.sleep();
	}
}
