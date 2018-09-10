// Copyright (c) 2015, Adam Allevato
// Copyright (c) 2017, The University of Texas at Austin
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _RECOGNIZER_H_
#define _RECOGNIZER_H_

#include <ctime>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/VisionInfo.h>

#include "orp/RecognizerConfig.h"
#include "orp/GetObjectPose.h"
#include "orp/WorldObjects.h"
#include "orp/ClassificationResult.h"
#include "orp/GetObjects.h"

#include "orp/core/world_object.h"
#include "orp/core/world_object_manager.h"
#include "orp/core/orp_utils.h"

/**
 * @brief   Base recognizer class.
 *
 * Stores a list of objects that it thinks are currently in the world. This
 * list is dynamically updated by classification messages published by various
 * classifiers. It is designed to work with multiple classifier types.
 */
class Recognizer {
private:
  /// Standard ROS node handle
  ros::NodeHandle n;
  /// Enables usage of dynamic_reconfigure.
  dynamic_reconfigure::Server<orp::RecognizerConfig> reconfigureServer;
  /// Used for dynamic reconfigure internals
  dynamic_reconfigure::Server<orp::RecognizerConfig>::CallbackType
      reconfigureCallbackType;

// ADDITIONAL VARIABLES GO HERE

  /// Used to schedule the recognition calls
  ros::Timer timer;
  /// What frame the objects are being detected in
  std::string recognitionFrame;
  /// A list of currently-accepted objects in the world.
  WorldObjectList model;
  /// Manages the list of world object types.
  WorldObjectManager typeManager;
  /// Where to publish the WorldObjects
  std::string objectTopic;
  /// Where to publish the RViz markers for visualization
  std::string markerTopic;
  /// Where to look for classification messages to be published
  std::string classificationTopic;
  /// If true, udpate our model of the world (and remove old objects). Used
  /// for lazy updates
  bool dirty = false;
  /// if true, begin the recognition loop automatically (auto-subscribe to
  /// classification)
  bool autostart = true;

  /// if true, publish old ORP messages instead of vision_msgs
  bool legacy = false;

// ROS

  /// Publishes vision_msgs meta information
  ros::Publisher visionInfoPub;
  /// Keep track of the vision_msgs meta information to publish
  vision_msgs::VisionInfo visionInfo;

  /// Listens for new recognized objects and adds them to the model.
  ros::Subscriber recognitionSub;
  /// Publishes a MarkerArray with information about detected objects
  ros::Publisher objectPub;
  /// Publishes a WorldObjects message that contains all the recognized
  /// objects from
  ros::Publisher markerPub;
  /// Used to self-stop recognition
  ros::Publisher stopPub;
  /// listen for necessary transformations before publishing
  tf::TransformListener* transformListener;
  /// publish object transformation frames
  tf::TransformBroadcaster* transformBroadcaster;
  /// Provides poses for requested objects.
  ros::ServiceServer objectPoseServer;
  /// Provides poses of all current objects.
  ros::ServiceServer objectsServer;

  /// to start recognition loop
  ros::Subscriber startSub;
  /// to stop recognition loop
  ros::Subscriber stopSub;

  /// used to store best result when looping through model in a search
  WorldObjectPtr best;

// DYNAMICALLY-SET CONFIG VARIABLES

  /// Number of seconds that an object will persist in the world model after
  /// being updated
  ros::Duration staleTime;
  /// Distance, in m, that objects must be separated by in order to be thought
  /// of as separate objects.
  float colocationDist;
  /// The wait between recognition calls
  ros::Duration refreshInterval;

  /// Show "unknown" labels
  bool showUnknownLabels;
  /// Show the decimal recognition probability
  bool showRecognitionProbability;
  /// Show objects' (X,Y,Z) positions
  bool showPosition;
  /// Show objects' poses (rotation about Y-axis)
  bool showPose;
  /// Show standared deviation of pose
  bool showPoseStdDev;
  /// Call debugPrint() on objects in the object model
  bool shouldDebugPrint;

  /// Used to ensure that at least one detection has been processed when
  /// calling service to get objects. This way we can tell the difference
  /// between "no objects in scene" and "no classification results received"
  int classification_count;
  /// Used to set the header.seq values in the messages published to
  /// /detected_objects.
  int object_sequence;

  /**
   * Get the object in the world that has the best shot at being the given
   * object type.
   * @param  wot The object type to look for
   * @return     The WorldObject that best fits the type being searched for.
   *             Returns 0 if there are no objects in the world, or if none of
   *             them have any probability of being the desired type.
   */
  WorldObjectPtr getMostLikelyObjectOfType(WorldObjectType wot);

  /**
   * Get the object in the world that has the best shot at being the given
   * object type. Convenience method. See also
   * getMostLikelyObjectOfType(WorldObjectType* wot).
   * @param  name The name of the object type to look for
   * @return      The WorldObject that best fits the type being searched for.
   *              Returns 0 if there are no objects in the world, or if none
   *              of them have any probability of being the desired type.
   */
  WorldObjectPtr getMostLikelyObjectOfType(std::string name);

  bool isRecognitionStarted();

  /**
   * loads info from the parameter server and stores basic information about
   * each item being detected.
   */
  void loadTypesFromParameterServer();

  /// sets the refresh interval and updates the timer
  void setRefreshInterval(float interval);

  /// Starts the recognition running at pre-specified rate.
  void startRecognition();
  /// Stops the recognition from running.
  void stopRecognition();

  /**
   * Perform recognition.
   *
   * A single function that will
   * do everything required for object recognition.
   */
  void recognize(const ros::TimerEvent& event);

  /**
   * one update tick, used to kill stales.
   */
  void update();

  /**
   * Removes any objects that are older than the "stale time" (i.e., haven't
   * been detected for a while)
   */
  void killStale();

  /**
   * Prepares and sends out ROS-related messages with classification
   * information. This includes object information as well as RViz markers.
   */
  void publishROS();
  /**
   * Old version of ROS publishing code, for backwards compatibility with
   * ORP 1 users.
   */
  void publishROSLegacy();

  /**
   * Creates a new marker for the given object. This should be called when
   * adding an object to the scene if you want it to have a corresponding
   * marker.
   * @arg wo the WorldObjectPtr to add
   */
  void addMarker(WorldObjectPtr wo);

  /**
   * Update a marker with information from the given world object.
   * @arg wo the WorldObjectPtr to update the marker information to
   */
  void updateMarker(WorldObjectPtr wo);

  /**
   * Sets the type to DELETE for the given object marker, assuming such a
   * marker exists. This shouldn't crash if the given id doesn't exist, it
   * just won't do anything.
   * @arg wo the WorldObjectPtr to delete from the world model.
   */
  void deleteMarker(WorldObjectPtr wo);

  /**
   * ROS wrapper for startRecognition()
   * @param msg unused
   */
  void cb_startRecognition(std_msgs::Empty msg);

  /**
   * ROS wrapper for stopRecognition()
   * @param msg unused
   */
  void cb_stopRecognition(std_msgs::Empty msg);

  /**
   * Get all currently known objects immediately.
   *
   * This function makes the following assumptions:
   *   - camera data is being published
   *   - at least one classifier exists
   *
   * If recognition is stopped when this is called, it will start it, and stop
   * it again once the call is complete. It will wait for at least one
   * classification result to be processed and then will return the current
   * internal object state.
   */
  bool cb_getObjects(orp::GetObjects::Request &req,
    orp::GetObjects::Response &response);

  /**
   * Callback for when a classification result is published by any classifier.
   * @param newObject the passed message from the classifier topic
   */
  void cb_processNewClassification(orp::ClassificationResult newObject);

  /**
   * ROS service call handler. Searches the known world model for objects that
   * match the given request, and then provides a list of poses for objects
   * that meet the given criteria.
   * @param  req      the request providing the object to search for
   * @param  response the response to fill with the poses found.
   * @return          true
   */
  bool getObjectPose(orp::GetObjectPose::Request &req,
    orp::GetObjectPose::Response &response);
public:
  /**
   * Main Recognizer constructor.
   * Sets up internal configuration, creates ROS variables, and starts
   * recognition (if autostart is enabled). Autostart enabled by default, can
   * be overriden with ROS parameter at command line or launch file
   */
  Recognizer();

  /// Dynamic Reconfigure callback.
  void paramsChanged(orp::RecognizerConfig &config, uint32_t level);
};

#endif
