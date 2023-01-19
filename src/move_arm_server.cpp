#include <ros/ros.h>
#include <stdio.h>

/*
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
//#include <std_srvs/SetBool.h>
*/


#include <std_msgs/Float64.h>

#include <assignment2/SetPose.h>


//global definitions so that they can be used outside the main aswell
ros::Publisher pub_joint1;
ros::Publisher pub_joint2;
ros::Publisher pub_joint3;
ros::Publisher pub_joint4;


std_msgs::Float64 joint1_value;
std_msgs::Float64 joint2_value;
std_msgs::Float64 joint3_value;
std_msgs::Float64 joint4_value;



// qui setta le pose predefinite in base al nome
int new_pose = 0;
int first_time = 1;
//Callback of service server (MA A ME SERVE UN SERVER O NO?..forse si perche devo fare da qualche parte chem quando ho ricevuto l id , 
// chiamo la pose successiva...FORSE PROPRIO DENTRO A marker_server.cpp??? in quella callback metto che quando è scannerizzato un  aruco chiamo questo server per settare
// la pose successiva....per ora provo a lasciare struttura del server e vedo se riesco a muovere robot chiamndo poi sto servizio da shell


bool reach(assignment2::SetPose::Request &req, assignment2::SetPose::Response &resp){

//############################################# IMPORTANTE ####################################
	  /*
	  comem fare poi rotazione 360?*/
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
		joint1_value.data =2.15; //1.95...ma base si sposta
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
	default:
		ROS_INFO("Non existing Pose");
		
		break;
		}
  return true;
}

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
  
  //At the beginning force the position of first marker, then wait for calls ORA FUNZA con la sleep
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
