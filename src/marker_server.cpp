#include <ros/ros.h>
#include <assignment2/RoomConnection.h>
#include <assignment2/RoomInformation.h>

//for my service SetPose.srv
#include "assignment2/SetPose.h"

//define client for /MY_move_arm service
ros::ServiceClient arm_client; 

assignment2::SetPose next_pose;

bool markerCallback(assignment2::RoomInformation::Request &req, assignment2::RoomInformation::Response &res){
	assignment2::RoomConnection conn;
	switch (req.id){
	case 11:
		res.room = "E";
		res.x = 1.5;
		res.y = 8.0;
		conn.connected_to = "C1";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		ROS_INFO(" HO RICEVUTO ID MARKER = 11");
		
		
   		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 5;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	case 12: 
		res.room = "C1";
		res.x = -1.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D5";
		res.connections.push_back(conn);
		conn.connected_to = "C2";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		conn.connected_to = "R2";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		ROS_INFO(" HO RICEVUTO ID MARKER = 12");
		
   		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 6;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	case 13: 
		res.room = "C2";
		res.x = 3.5;
		res.y = 0.0;
		conn.connected_to = "E";
		conn.through_door = "D6";
		res.connections.push_back(conn);
		conn.connected_to = "C1";
		conn.through_door = "D7";
		res.connections.push_back(conn);
		conn.connected_to = "R3";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		conn.connected_to = "R4";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		ROS_INFO(" HO RICEVUTO ID MARKER = 13");
		
		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 4;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	case 14: 
		res.room = "R1";
		res.x = -7.0;
		res.y = 3.0;
		conn.connected_to = "C1";
		conn.through_door = "D1";
		res.connections.push_back(conn);
		ROS_INFO("AAAAAAAA HO RICEVUTO ID MARKER = 14");
		
   		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 0;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	case 15: 
		res.room = "R2";
		res.x = -7.0;
		res.y = -4.0;
		conn.connected_to = "C1";
		conn.through_door = "D2";
		res.connections.push_back(conn);
		ROS_INFO(" HO RICEVUTO ID MARKER = 15");
		
   		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 3;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	case 16: 
		res.room = "R3";
		res.x = 9.0;
		res.y = 3.0;
		conn.connected_to = "C2";
		conn.through_door = "D3";
		res.connections.push_back(conn);
		ROS_INFO(" HO RICEVUTO ID MARKER = 16");
		
   		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 2;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	case 17: 
		res.room = "R4";
		res.x = 9.0;
		res.y = -4.0;
		conn.connected_to = "C2";
		conn.through_door = "D4";
		res.connections.push_back(conn);
		ROS_INFO("HO RICEVUTO ID MARKER = 17");
		
   		//then call the service /MY_move_arm to move the robot towards the next marker 
   		next_pose.request.pose_number = 7;
   		arm_client.waitForExistence(); 
   		arm_client.call(next_pose);
		break;
	default:
		res.room = "no room associated with this marker id";
	}
	return true;
}	





int main(int argc, char **argv)
{
	ros::init(argc, argv, "assignment2");
	ros::NodeHandle nh;
	ros::ServiceServer oracle = nh.advertiseService( "/room_info",markerCallback);
	
	//define the client of my service MY_move_arm
	arm_client = nh.serviceClient<assignment2::SetPose>("/MY_move_arm");
	
	
	
	ros::spin();
	
	ros::shutdown();
	return 0;
}
