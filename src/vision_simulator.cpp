// Copyright (c) 2016, Adam Allevato
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <fstream>
#include <string>
#include <sstream>

#include <orp/WorldObject.h>
#include <orp/WorldObjects.h>

#include "orp/app/vision_simulator.h"

std::vector<std::string> labelsi;
/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_simulator");
  ros::NodeHandle n;

  if(argc != 3) {
    ROS_FATAL("proper usage is 'vision_simulator input_file object_frame");
    return -1;
  }
  std::string filename = argv[1];
  std::string outputFrame = argv[2];
  ROS_INFO("Starting Vision Simulator");
  
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  VisionSimulator s(n, filename, outputFrame);
  return 1;
} //main

///////////////////////////////////////////////////////////////////////////////////

VisionSimulator::VisionSimulator(ros::NodeHandle nh, std::string filename, std::string outputFrame) :
    n(nh),
    frame(outputFrame),
    simCount(0)
{
  listener = new tf::TransformListener();
  int_markers = std::map<std::string, visualization_msgs::InteractiveMarker>();

  markerServer.reset( new interactive_markers::InteractiveMarkerServer("vision_simulator_objects","",false) );
  objectPoseServer = n.advertiseService("/get_object_pose", &VisionSimulator::getObjectPose, this);
  
  ros::Publisher objectsPub = n.advertise<orp::WorldObjects>("detected_objects", 1);

  std::string line;
  float x, y, z;
  std::string objName;
  tf::Vector3 position;

  orp::WorldObjects theMessage;
    
  std::ifstream infile(filename.c_str());
  while (std::getline(infile, line))
  {
    std::istringstream iss(line);
    if (!(iss >> objName >> x >> y >> z )) { ROS_ERROR("Error while parsing file!"); } // error

    ROS_INFO_STREAM("creating marker for object type " << objName << " at " << x << ", " << y << ", " << z);
    position = tf::Vector3(x,y,z);
    make6DofMarker(visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true, objName );

  
    orp::WorldObject theObject;
    theObject.label = objName;
    theObject.pose.header.frame_id = outputFrame;
    theMessage.objects.push_back(theObject);
  }

  markerServer->applyChanges();
  ROS_INFO("vision simulator loaded");

  ros::Rate loop(2.0f); //2 Hz
  while(ros::ok() && loop.sleep()) {
    for(int i = 0; i < theMessage.objects.size(); ++i) {
      theMessage.objects[i].pose.pose = int_markers.at(labelsi[i]).pose;
    }
    objectsPub.publish(theMessage);
  }
} //VisionSimulator constructor

bool VisionSimulator::getObjectPose(
  orp::GetObjectPose::Request &request,
  orp::GetObjectPose::Response &response)
{
  response.num_found = 1;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "/base_link";
  pose.header.stamp = ros::Time(0);
  ROS_INFO_STREAM("retrieving item with name " << request.name.substr(0, request.name.length()-11));
  pose.pose = int_markers.at(request.name.substr(0, request.name.length()-11)).pose;

  ROS_INFO("marker pose: %f %f %f / %f %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    pose.pose.orientation.x, pose.pose.orientation.y,  pose.pose.orientation.z,  pose.pose.orientation.w);

  ROS_INFO("waiting for transform");
  if(!listener->waitForTransform("/base_link",frame,pose.header.stamp,ros::Duration(4.0))) {
    ROS_ERROR("Couldn't get transform");
  }
  listener->transformPose(frame, pose, pose);

  response.poses.push_back(pose);

  ROS_INFO("output pose: %f %f %f / %f %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    pose.pose.orientation.x, pose.pose.orientation.y,  pose.pose.orientation.z,  pose.pose.orientation.w);
  return true;
} //getObjectPose

void VisionSimulator::make6DofMarker( unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof, std::string name )
{
  float xsize = 0.1;
  float ysize = 0.1; 
  float zsize = 0.1;

  visualization_msgs::InteractiveMarker int_marker;

  int_marker.header.frame_id = frame;
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = std::max(std::max(xsize, ysize), zsize);

  int_marker.name = name + "_" + ORPUtils::zeroPad(simCount, 10);
  labelsi.push_back(int_marker.name);
  int_marker.description = "simulated " + name;
  int_marker.pose.orientation.x = 0;
  int_marker.pose.orientation.y = 0;
  int_marker.pose.orientation.z = 0;
  int_marker.pose.orientation.w = 1;

  ORPUtils::makeBoxControl(int_marker, xsize, ysize, zsize, 0.5, 0.5, 0.5); //does hard confusing stuff to make visible portion of marker

  //now make the controls

  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::InteractiveMarkerControl control;

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  updateCallback = boost::bind(&VisionSimulator::cb_markerFeedback, this, _1);

  ROS_INFO("inserting marker into marker server");
  markerServer->insert(int_marker);
  markerServer->setCallback(int_marker.name, updateCallback);
  markerServer->applyChanges();

  int_markers.insert(std::pair<std::string, visualization_msgs::InteractiveMarker>(int_marker.name, int_marker));
  ++simCount;
}

void VisionSimulator::cb_markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
  ROS_INFO_STREAM("setting pose of " << feedback->marker_name.substr(0, feedback->marker_name.length()-11));
  int_markers.at(feedback->marker_name.substr(0, feedback->marker_name.length()-11)).pose = feedback->pose;
}
