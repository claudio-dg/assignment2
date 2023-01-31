/**
* \file move_arm_server.cpp
* \brief implements the server service to move robot's arm
* \author Claudio Del Gaizo
* \version 0.1
* \date 21/01/2023
*
*
* \details
*
*
* Publishes to: <BR>
*
* /m2wr/joint1_position_controller/command : to move the joint_1 of robot's arm
*
* /m2wr/joint2_position_controller/command : to move the joint_2 of robot's arm
*
* /m2wr/joint3_position_controller/command : to move the joint_3 of robot's arm
*
* /m2wr/joint4_position_controller/command : to move the joint_4 of robot's arm
*
* Server for Services: <BR>
*
* MY_move_arm : Service to select which robot's arm position to apply
*
*
* Description:
*
* This node implements the server service to move robot's arm : when called it will publish the joint values on the various Joint_position_controllers according to the request number, in order to move the arm to a certain pose, (each related to a certain aruco marker to detect).
**/




#include <ros/ros.h>
#include <stdio.h>

#include <std_msgs/Float64.h>
#include <assignment2/MY_SetPose.h>


//global definitions so that they can be used outside the main aswell
ros::Publisher pub_joint1; ///< global publisher for /m2wr/joint1_position_controller/command
ros::Publisher pub_joint2; ///< global publisher for /m2wr/joint2_position_controller/command
ros::Publisher pub_joint3; ///< global publisher for /m2wr/joint3_position_controller/command
ros::Publisher pub_joint4; ///< global publisher for /m2wr/joint4_position_controller/command


std_msgs::Float64 joint1_value; ///< message containing joint_1 values
std_msgs::Float64 joint2_value; ///< message containing joint_2 values
std_msgs::Float64 joint3_value; ///< message containing joint_3 values
std_msgs::Float64 joint4_value; ///< message containing joint_4 values

//global flag variables
int new_pose = 0; ///< global int variable to flag if a new pose has to be published 
int first_time = 1; ///< global int variable to flag first iteration
int sign = -1;  ///< global int variable to rotate camera clock-wise/anti-clock-wise

/**
* \brief Server Callback for /MY_move_arm Service 
* \param req : assignment2::MY_SetPose::Request : contains an integer specifying which pose the robot should reach 
*
* \param resp : :assignment2::MY_SetPose::Response : response non used here
*
* \return bool
*
* 
* This function takes the value of the request according to which it initializes the joint values necessary to rech the required pose. It presents 9 different poses to select from, 1 for each marker plus a default pose and a pose to rotate the camera of 360 degrees.
*
**/
bool reach(assignment2::MY_SetPose::Request &req, assignment2::MY_SetPose::Response &resp){

switch (req.pose_number){
	case 0:
		
		ROS_INFO("Default pose");
		joint1_value.data =0;
		joint2_value.data =0;
		joint3_value.data =0.1;
		joint4_value.data =0;
		new_pose = 1;
		break;
	case 1: 
		
		ROS_INFO("first marker pose");
		joint1_value.data =0.2;
		joint2_value.data =1.6;
		joint3_value.data =0;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 2:
		
		ROS_INFO("second marker pose");
		joint1_value.data =0.2;
		joint2_value.data =0.8;
		joint3_value.data =0.5;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 3: 
		
		ROS_INFO("third marker pose");
		joint1_value.data =2.15; //1.95
		joint2_value.data =0.8;
		joint3_value.data =0.5;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 4:
		
		ROS_INFO("fourth marker pose");
		joint1_value.data =3.14;//3.14
		joint2_value.data =0.8;
		joint3_value.data =0.5;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 5: 
		
		ROS_INFO("fifth marker pose");
		joint1_value.data =3.14;
		joint2_value.data =1.6;
		joint3_value.data =0;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 6:
	
		ROS_INFO("sixth marker pose");
		joint1_value.data =3.8;
		joint2_value.data =1.6;
		joint3_value.data =0;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 7: 
		
		ROS_INFO("seventh marker pose");
		joint1_value.data =4.6;//4.4
		joint2_value.data =1.3;
		joint3_value.data =0;
		joint4_value.data =0;
		new_pose = 1;
		break;
	case 10: 
		
		ROS_INFO("Rotation request");
		sign = -sign ;  //+-1
		joint1_value.data =6.28*sign; //3.6
		joint2_value.data =0;
		joint3_value.data =0.1;
		joint4_value.data =0;
		
		/*
		pub_joint1.publish(joint1_value);
		pub_joint2.publish(joint2_value);
		pub_joint3.publish(joint3_value);
		pub_joint4.publish(joint4_value);
		ROS_INFO("prima mossa  ...");		
		ros::Duration(1).sleep();
		
		joint1_value.data =3.6;//3.6
		pub_joint1.publish(joint1_value);
		ROS_INFO("seconda mossa  ...");		
		ros::Duration(6).sleep();
		
		joint1_value.data = 6.2;//6.1
		ROS_INFO("terza mossa  ...");*/
		
		new_pose = 1;
		break;
	default:
		ROS_INFO("Non existing Pose");
		
		break;
		}
  return true;
}


/**
* \brief main function
*
*
* Initialize the node, the server for /MY_move_arm and the publishers for joint_position_controllers, then it publshes (only once) the position of the arm to detect the first aruco marker, and waits for a new pose to be set to publish it.
*
**/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_server");
  ros::NodeHandle nh;
  //init service server
  ROS_INFO("Creating Service server to move arm  ...");
  ros::ServiceServer service = nh.advertiseService("MY_move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Define the publishers for robot's joints 
  pub_joint1 = nh.advertise<std_msgs::Float64> ("/m2wr/joint1_position_controller/command", 1);
  pub_joint2 = nh.advertise<std_msgs::Float64> ("/m2wr/joint2_position_controller/command", 1);
  pub_joint3 = nh.advertise<std_msgs::Float64> ("/m2wr/joint3_position_controller/command", 1);
  pub_joint4 = nh.advertise<std_msgs::Float64> ("/m2wr/joint4_position_controller/command", 1);
  
  //At the beginning force the position of first marker, then wait for calls 
  joint1_value.data =0.2;
  joint2_value.data =1.6;
  joint3_value.data =0;
  joint4_value.data =-1.5;
  ROS_INFO("START MOVING towards the first marker pose in 3 sec...");
  int i = 0;
  ros::Duration(3).sleep();
  pub_joint1.publish(joint1_value);
  pub_joint2.publish(joint2_value);
  pub_joint3.publish(joint3_value);
  pub_joint4.publish(joint4_value);

  while(ros::ok()){
  	 
 	 if (new_pose)
 	 {
		
		pub_joint1.publish(joint1_value);
		pub_joint2.publish(joint2_value);
		pub_joint3.publish(joint3_value);
		pub_joint4.publish(joint4_value); 
		ROS_INFO("PUBLISHING: J1= %f** J2 = %f** J3=%f** J4= %f...",joint1_value.data,joint2_value.data,joint3_value.data,joint4_value.data );
		new_pose = 0;
	 }
  }
  return(0);
}
