# Collaboration_of_Autonomous_Mobile_Robots_with_Different_Wheel_Types_in_Transportation
My Bachelor 's Degree Final Project at KMUTT (PRE496-PRE497)


Abstract: This study proposes a cooperative transportation system, which uses multiple autonomous mobile robots with different wheel types cooperating together to transport a rigid object. The system uses a leader-follower formation control strategy, by using the non-holonomic leading the way while the holonomic AMRs follow. The object is assumed as a constraint between the robots, and the cooperation is tested using a designed destination point, in which each follower robot receives commands from the leader sent through the laptop. To walk straight, all robots receive the same command, but for rotating, leader robot must evaluate the command for the follower robots by the designed method. The Robot Operating System (ROS) is used for the development of the robots and for standard communication. The proposed method is validated through simulation tasks using autonomous mobile robots with different wheel types.

<p align="center">
  <img  src="https://github.com/SPACEWALKER31552/Collaboration_of_Autonomous_Mobile_Robots_with_Different_Wheel_Types_in_Transportation/assets/109845426/a8e6ddb6-1065-4dc0-9810-7afb1c9119a4">
</p>

Introduction: In recent years, the use of autonomous mobile robots (AMRs) has become increasingly widespread across a wide range of industries, especially in transportation and material handling. While AMRs have revolutionized manufacturing, logistics, and other areas, they often operate independently of one another, limiting their effectiveness in certain applications. Cooperative autonomous mobile robots (C-AMRs) have emerged as a promising solution to this problem. Cooperative AMRs are designed to work together in a coordinated fashion to accomplish shared tasks, such as material handling or transportation of a heavy object [1],[2]. This cooperative approach offers numerous advantages over traditional AMRs. Firstly, C-AMRs can improve efficiency and productivity by working together to perform tasks that are difficult or impossible for individual robots to accomplish alone. For example, C-AMRs can work together to move large or heavy objects, or to navigate complex environments. Secondly, C-AMRs can enhance safety by sharing information and coordinating movements, reducing the risk of collisions or other accidents. Additionally, C-AMRs can provide greater flexibility and adaptability by coordinating their movements and tasks in real-time, enabling them to respond quickly to changing conditions.
Although there are many studies about the cooperative AMRs with same type. But there are only a few case studies that study the use of AMRs with different drives together. As the number of utilizing of AMRs in industries is still increasing, it is possible that AMRs with different maneuverability should work together. As the holonomic and non-holonomic AMRs have different degree of freedom, the movement and subsequently the sequence of control of two types of AMRs are different.

Robots design and their kinematics
The group of AMRs in this study consists of two holonomic AMRs with mecanum wheels and one non-holonomic AMRs with the differential-drive. The sensors of the AMRs are an inertial measurement unit (IMU) and encoders. To perform the transportation task, all AMRs are equipped with the gripper to hold the metal sheet. The gripper is supported by a bearing so that the mobile robot can rotate without constraint when they transport the metal sheet. The overall design of the robots is shown in Fig. 1 and 2.

 
<p align="center">
  <img  src="https://github.com/SPACEWALKER31552/Collaboration_of_Autonomous_Mobile_Robots_with_Different_Wheel_Types_in_Transportation/assets/109845426/743a787f-a58c-448d-848a-5ee7d504901a">
Figure 1. The Overall design of the non-holonomic robot
</p>

 
<p align="center">
  <img  src="https://github.com/SPACEWALKER31552/Collaboration_of_Autonomous_Mobile_Robots_with_Different_Wheel_Types_in_Transportation/assets/109845426/400eea55-fbfe-43a9-8811-e44eb573720e">
Figure 2. The Overall design of the non-holonomic robot
</p>


Kinematic Model of differential-drive mobile robot
It is generally known that the differential-drive wheel type is a non-holonomic type of AMR because it can move only two degrees of freedom in plane. The kinematic model of a differential-drive mobile robot is described by the following equations:

[■(x ̇@y ̇@θ ̇ )]=[■((r_L  cos⁡θ)/2&(r_R  cos⁡θ)/2@(r_L  sin⁡θ)/2&(r_R  sin⁡θ)/2@-r_L/2b&r_R/2b)][■(φ ̇_L@φ ̇_R )]	(1)

 

Figure 3. differential-drive mobile robot in parameter
Furthermore, when ordering a command, it must be ordered in the form of linear velocity and then converted through the calculation equation to get angular speed for PWM commands to differential-drive mobile robots. Therefore, distribute variables in an equation and shape the equation to achieve an angular velocity, which is as follows:



Kinematic model of mecanum wheel mobile robot
 
Figure 4. mecanum wheel mobile robot in parameter

For mecanum wheel, it is a special type of wheel that can move up to three degrees of freedom, but there are some limitations, such as moving on rough surfaces, which is not suitable and may result in damaging to the wheels or skidding during some movement. And since there is a degree of freedom that is different from differential-drive mobile robots, the kinematic equation is also different. The kinematic model of a mecanum wheel mobile robot is described by the following equations:



Same as differential-drive wheel model, to command the robot. The command must be ordered in the form of linear velocity. Therefore, the inverse kinematic must be performed from the above equation to achieve an angular velocity, which is as follows:

[■(ω_1@ω_2@ω_3@ω_4 )]=1/r [■(1&-1&-(l_x+l_y)@1&1&(l_x+l_y)@1&1&-(l_x+l_y)@1&-1&(l_x+l_y))][■(v_x@v_y@ω_z )]	(4)
