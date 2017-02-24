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

#ifndef _VISION_SIMULATOR_H_
#define _VISION_SIMULATOR_H_

#include <ctime>
#include <fstream>
#include <map>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <orp/WorldObjects.h>
#include <orp/GetObjectPose.h>

#include "orp/core/world_object.h"
#include "orp/core/world_object_manager.h"
#include "orp/core/orp_utils.h"

/**
 * @brief   Simulates ORP vision objects using interactive RViz markers. Position the objects using RViz, then they act
 * like normally-detected ORP objects when looking at ORP recognizer output.
 */
class VisionSimulator {
private:
  /// Standard ROS node handle
  ros::NodeHandle n;

  std::string frame;                                                                /// What frame the objects are being detected in

  ros::ServiceServer objectPoseServer;                                              /// Provides poses for requested objects.

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> markerServer;     /// Serves the interactive markers which control simulated obj positions
  interactive_markers::InteractiveMarkerServer::FeedbackCallback updateCallback;    /// Called when the interactive marker is updated
  std::map<std::string, visualization_msgs::InteractiveMarker> int_markers;         /// One marker per object

  tf::TransformListener* listener;                                                  /// Used to transform sim objects into the correct frame

  
  unsigned short int simCount;                                                      /// Used to generate unique names for simulation markers

  /**
   * ROS service call handler. Searches the known world model for objects that match
   * the given request, and then provides a list of poses for objects that meet the given
   * criteria.
   * @param  req      the request providing the object to search for
   * @param  response the response to fill with the poses found.
   * @return          true
   */
  bool getObjectPose(orp::GetObjectPose::Request &req,
    orp::GetObjectPose::Response &response);

  /**
   * Create a single marker
   */
  void make6DofMarker( unsigned int interaction_mode,
    const tf::Vector3& position, bool show_6dof, std::string name );

public:
  /**
   * Read the given filename and set the output frame to the supplied frame name.
   * Each line in the filename will be converted into a single marker with the given format
   * object_name x_pos y_pos z_pos
   */
  VisionSimulator(ros::NodeHandle nh, std::string filename, std::string outputFrame);

  /// Called when a marker is updated
  void cb_markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ); 
}; //VisionSimulator

#endif //_VISION_SIMULATOR_H_
