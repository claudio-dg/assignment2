#include <ros/ros.h>
#include <stdio.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
//#include <std_srvs/SetBool.h>

//custom srv 
#include <assignment2/SetPose.h>

// qui setta le pose predefinite in base al nome
int pose_required = 100;

//Callback of service server (MA A ME SERVE UN SERVER O NO?..forse si perche devo fare da qualche parte chem quando ho ricevuto l id , 
// chiamo la pose successiva...FORSE PROPRIO DENTRO A marker_server.cpp??? in quella callback metto che quando è scannerizzato un  aruco chiamo questo server per settare
// la pose successiva....per ora provo a lasciare struttura del server e vedo se riesco a muovere robot chiamndo poi sto servizio da shell


bool reach(assignment2::SetPose::Request &req, assignment2::SetPose::Response &resp){


switch (req.pose_number){
	case 0:
		pose_required = 0;
		break;
	case 1: 
		pose_required = 1;
		break;
	case 2:
		pose_required = 2;
		break;
	case 3: 
		pose_required = 3;
		break;
	case 4:
		pose_required = 4;
		break;
	case 5: 
		pose_required = 5;
		break;
	case 6:
		pose_required = 6;
		break;
	case 7: 
		pose_required = 7;
		break;
	default:
		printf("Non existing Pose");
		
		break;
		}
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state");
  ros::NodeHandle nh;
  //init service server
  ROS_INFO("Creating Service server to moveit arm controller ...");
  ros::ServiceServer service = nh.advertiseService("MY_move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  group.setStartStateToCurrentState();
  group.setGoalOrientationTolerance(0.01);
  group.setGoalPositionTolerance(0.01);
  
  
  while(ros::ok()){
 	 switch (pose_required){
	case 0:
		group.setNamedTarget("my_default_pose");
	  	group.move();
	  	ROS_INFO("setting to default pose ...");
	  	pose_required=100;		
		break;
	case 1: 
		group.setNamedTarget("first_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the first marker ...");
	  	pose_required=100;
		break;
	case 2:
		group.setNamedTarget("second_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the second marker ...");
	  	pose_required=100;
		break;
	case 3: 
		group.setNamedTarget("third_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the third marker ...");
	  	pose_required=100;
		break;
	case 4:
		group.setNamedTarget("fourth_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the fourth marker ...");
	  	pose_required=100;
		break;
	case 5: 
		group.setNamedTarget("fifth_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the fifth marker ...");
	  	pose_required=100;
		break;
	case 6:
		group.setNamedTarget("sixth_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the sixth marker ...");
	  	pose_required=100;
		break;
	case 7: 
		group.setNamedTarget("seventh_marker_pose");
	  	group.move();
	  	ROS_INFO("Moving towards the seventh marker ...");
	  	pose_required=100;
		break;
	default:	//quando e' 100 non fai niente	
		break;
		}
  }
  return(0);
}
