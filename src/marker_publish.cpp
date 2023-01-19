/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */


// PER LANCIARLO: rosrun assignment2 marker_publish_node /image:=/robot/camera1/image_raw COME REMAPPARE IL TOPIC DA LAUNH FILE

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//for my service SetPose.srv
//#include "assignment2/SetPose.h"

//for my service RoomInformation.srv
#include <assignment2/RoomInformation.h>

//for my topic /my_Id_topic
#include <std_msgs/UInt32.h>

//for using str
#include <cstring>

//ros::ServiceClient marker_client; //quindi questo lo dovro cancellare poi
//assignment2::RoomInformation aruco_id; //quindi questo lo dovro cancellare poi

ros::Publisher id_pub;
std_msgs::UInt32 id;

int prec = 0;

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  // node params
  double marker_size_;
  bool useCamInfo_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;

  cv::Mat inImage_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;
   
      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

		std::cout << "The id of the detected marker detected is: ";
        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          std::cout << markers_.at(i).id << " ";
          //°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°° IO °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
          int current_id =  markers_.at(i).id;
          
          if(current_id != prec) 
	//if statement to call the service only once when a new aruco is detected instead having multiple calls for the same marker
          {
          ROS_INFO("NUOVO ID E': %i ...",current_id );
          prec = current_id;
          
          id.data = current_id;
          id_pub.publish(id);
          if(prec ==14) // 14terminate this node when last  marker is detected
 	  {
 	  ROS_INFO("All markers detected, terminate the node");
 	  ros::Duration(2).sleep();
 	  abort();
 	  } 
           
           /* parte della client call diretta a marker server
 	//fill the request field with the id of the marker detected
 	  aruco_id.request.id = current_id;
 	  ROS_INFO("CHIAMO MOVE_ARM_SERVER dando come ID: %i ...",aruco_id.request.id );
 	//wait for the existence of the service and then call it
 	  marker_client.waitForExistence(); 
 	  marker_client.call(aruco_id);
 	  if(prec ==14) // 14terminate this node when last  marker is detected
 	  {
 	  ROS_INFO("All markers detected, terminate the node");
 	  abort();
 	  }
          //OK FUNZA MA POI DEVO PRENDERE LA RISPOSTA IN QUESTO SCRIPT CONTENENTE INFO DELLA STANZA E IN QUALCH EMODO COSTRUIRE LA ONTOLOGY...posso fare tutto qua mettendo semplicemente dopo ste righe la parte che scrive nella ontology? boh vediamo
          std::string room_str = aruco_id.response.room;  //COSI FUNZIONA = MODO GIUSTO DID PRENDERE STRINGA IN CPP
          
          float x = aruco_id.response.x;
          float y = aruco_id.response.y;
          
          //roomConnection come le estraggo? vedere poi!!! in python sarebbe piu facile.......
          //ROS_INFO("RISPOSTA RICEVUTA = :room %c x = %f y = %f room connections BOH ...",room,x,y );
          std::cout << room_str << std::endl; // PER STAMPARE STRINGA
          */

	  //############################################# IMPORTANTE ####################################
	  //forse per semplificare il tutto la questione del come prendere connectons ecc conviene cambiare struttura e fare cosi:
	  // dove qui ce il client call...metto in realta un publisher su un topic mio..cioe faccio pubblicare l id detectato da qualche parte
	  //dopodiche faccio uno script python che si subba a quel topic e legge l id,  a quel punto nello script python faccio la chiamata al marker server,
	  //cosi poi in quello script python gestisco la risposta in modo piu facile e soprattutto posso poi interagire con ontologuìy e tutto riusando il codice che avevo usato nell ass1!!!!!!
	  


          }
          //°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°° IO °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
        }
        std::cout << std::endl;

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");
  ros::NodeHandle nh;
  ArucoMarkerPublisher node;
  //define the client of my service /room_info to send the detected aruco_id
  //***********marker_client = nh.serviceClient<assignment2::RoomInformation>("/room_info");
  
  
  // Define the publisher for marker's id 
  id_pub = nh.advertise<std_msgs::UInt32> ("/my_ID_topic", 1);
  /*
  //define the client of my service MY_move_arm SOLO PER FAR MANDARE LA PRIMA POS DA RAGGIUNGERE IN AUTO
   ros::ServiceClient arm_client = nh.serviceClient<assignment2::SetPose>("/MY_move_arm");
   
     

   assignment2::SetPose first_pose;
   first_pose.request.pose_number = 1;
   //then call the service
   arm_client.waitForExistence(); 
   arm_client.call(first_pose);
   ROS_INFO("PUBBLICO LA PRIMA POS  ...");
   */
  ros::spin();
}
