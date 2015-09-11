#ifndef _VISION_SIMULATOR_H_
#define _VISION_SIMULATOR_H_

#include "ros/ros.h"

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "orp/WorldObjects.h"
#include <interactive_markers/interactive_marker_server.h>
#include <dynamic_reconfigure/server.h>
#include "orp/GetObjectPose.h"
#include "std_msgs/Empty.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "orp/core/world_object.h"
#include "orp/core/world_object_manager.h"
#include "orp/core/orp_utils.h"

// ADDITIONAL INCLUDES GO HERE
#include <ctime>
#include <fstream>
#include <map>

/**
 * @brief   Simulates vision objects using interactive RViz markers.
 *
 * @version 1.0
 * @ingroup apc
 * @ingroup objectrecognition
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date      4/1/2015
 */
class VisionSimulator {
private:
  /// Standard ROS node handle
  ros::NodeHandle n;

  std::string frame;         /// What frame the objects are being detected in

  ros::ServiceServer objectPoseServer;  /// Provides poses for requested objects.

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> markerServer;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback updateCallback;
  std::map<std::string, visualization_msgs::InteractiveMarker> int_markers;

  tf::TransformListener* listener;

  //used to generate unique names for simulation markers
  unsigned short int simCount;

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
   * Perform recognition.
   *
   * A single function that will
   * do everything required for object recognition.
   */
  void recognize(const ros::TimerEvent& event);

  void make6DofMarker( unsigned int interaction_mode,
    const tf::Vector3& position, bool show_6dof, std::string name );

public:
  /**
   * Read the given filename and set the output frame to the supplied frame name.
   * Each line in the filename will be converted into a single marker with the given format
   * object_name x_pos y_pos z_pos
   */
  VisionSimulator(ros::NodeHandle nh, std::string filename, std::string outputFrame);

  /**
   * Main VisionSimulator destructor
   */
  ~VisionSimulator();

  void cb_setStoredPose( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ); 
}; //VisionSimulator

#endif //_VISION_SIMULATOR_H_
