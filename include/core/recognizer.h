// Copyright (c) 2015, Adam Allevato
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

#ifndef _RECOGNIZER_H_
#define _RECOGNIZER_H_

#include <ctime>
#include <fstream>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <dynamic_reconfigure/server.h>

#include "orp/RecognizerConfig.h"
#include "orp/GetObjectPose.h"
#include "orp/WorldObjects.h"
#include "orp/ClassificationResult.h"
#include "orp/GetObjects.h"

#include "world_object.h"
#include "world_object_manager.h"
#include "core/orp_utils.h"

/**
 * @brief   Base recognizer class.
 *
 * Stores a list of objects that it thinks are currently in the world.
 *
 * @version 1.0
 * @ingroup apc
 * @ingroup objectrecognition
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 */
class Recognizer {
private:
  /// Standard ROS node handle
  ros::NodeHandle n;
  /// Enables usage of dynamic_reconfigure.
  dynamic_reconfigure::Server<orp::RecognizerConfig> reconfigureServer;
  dynamic_reconfigure::Server<orp::RecognizerConfig>::CallbackType reconfigureCallbackType;

// ADDITIONAL VARIABLES GO HERE

  ros::Timer timer;                             /// Used to schedule the recognition calls
  std::string recognitionFrame;                 /// What frame the objects are being detected in
  WorldObjectList model;                        /// A list of currently-accepted objects in the world.
  WorldObjectManager typeManager;               /// Manages the list of world object types.
  std::string objectTopic;                      /// Where to publish the WorldObjects
  std::string markerTopic;                      /// Where to publish the RViz markers for visualization
  bool dirty;                                   /// If true, udpate our model of the world (and remove old objects). Used for lazy updates
  bool autostart;                               /// if true, begin the recognition loop automatically (auto-subscribe to classification)
        
  //ROS       
  ros::Subscriber recognitionSub;               /// Listens for new recognized objects and adds them to the model.
  ros::Publisher objectPub;                     /// Publishes a MarkerArray with information about detected objects
  ros::Publisher markerPub;                     /// Publishes a WorldObjects message that contains all the recognized objects from 
  ros::Publisher stopPub;                       /// Used to self-stop recognition
  tf::TransformListener* transformListener;     /// listen for necessary transformations before publishing
  tf::Transformer* objectTransformeqr;          /// transform object poses from one frame to another
  ros::ServiceServer objectPoseServer;          /// Provides poses for requested objects.
  ros::ServiceServer objectsServer;             /// Provides poses of all current objects.
        
  ros::Subscriber startSub;                     /// to start recognition loop
  ros::Subscriber stopSub;                      /// to stop recognition loop
        
  WorldObjectPtr best;                          /// used to store best result when looping through model in a search

// DYNAMICALLY-SET CONFIG VARIABLES
  ros::Duration staleTime;          /// Number of seconds that an object will persist in the world model after being updated
  float colocationDist;             /// Distance, in m, that objects must be separated by in order to be thought of as separate objects.
  ros::Duration refreshInterval;    /// The wait between recognition calls

  bool showUnknownLabels;           ///Show "unknown" labels
  bool showRecognitionProbability;  ///Show the decimal recognition probability
  bool showPosition;                ///Show objects' (X,Y,Z) positions
  bool showPose;                    ///Show objects' poses (rotation about Y-axis)
  bool showPoseStdDev;              ///Show standared deviation of pose
  bool shouldDebugPrint;            ///Call debugPrint() on objects in the object model

  int classification_count;         ///Used to ensure that at least one detection has been processed when calling service to get objects. This way we can tell the difference between "no objects in scene" and "no classification results received"
  int object_sequence;              ///Used to set the header.seq values in the messages published to /detected_objects.

  /**
   * Get the object in the world that has the best shot at being the given object type.
   * @param  wot        The object type to look for
   * @return            The WorldObject that best fits the type being searched for.
   *                    Returns 0 if there are no objects in the world, or if none of them
   *                    have any probability of being the desired type.
   */
  WorldObjectPtr getMostLikelyObjectOfType(WorldObjectType wot);

  /**
   * Convenience method. See also getMostLikelyObjectOfType(WorldObjectType* wot)
   * Get the object in the world that has the best shot at being the given object type.
   * @param  name       The name of the object type to look for
   * @return            The WorldObject that best fits the type being searched for.
   *                    Returns 0 if there are no objects in the world, or if none of them
   *                    have any probability of being the desired type.
   */
  WorldObjectPtr getMostLikelyObjectOfType(std::string name);

  bool isRecognitionStarted();
  
  /**
   * loads info from the parameter server and stores basic information about each item
   * being detected.
   */
  void loadTypesFromParameterServer();


  /// sets the refresh interval and updates the timer
  void setRefreshInterval(float interval);

  void startRecognition();  /// Starts the recognition running at pre-specified rate.
  void stopRecognition();   /// Stops the recognition from running.
  
  /**
   * Perform recognition.
   *
   * A single function that will
   * do everything required for object recognition.
   */
  void recognize(const ros::TimerEvent& event);

  void update();
  
  /**
   * Removes any objects that are older than the "stale time" (i.e., haven't
   * been detected for a while)
   */
  void killStale();

  /**
   * Prepares and sends out ROS-related messages with classification information.
   * This includes object information as well as RViz markers.
   */
  void publishROS();

  /**
   * Creates a new marker for the given object. This should be called when adding an object
   * to the scene if you want it to have a corresponding marker.
   * @arg wo the WorldObjectPtr to add
   */
  void addMarker(WorldObjectPtr wo);

  /**
   * Update a marker with information from the given world object.
   * @arg wo the WorldObjectPtr to update the marker information to
   */
  void updateMarker(WorldObjectPtr wo);

  /**
   * Sets the type to DELETE for the given object marker, assuming such a marker exists.
   * This shouldn't crash if the given id doesn't exist, it just won't do anything.
   * @arg wo the WorldObjectPtr to delete from the world model.
   */
  void deleteMarker(WorldObjectPtr wo);

  /// ROS wrappers
  void cb_startRecognition(std_msgs::Empty msg); 
  void cb_stopRecognition(std_msgs::Empty msg);
  
  /**
   * Get all currently known objects immediately.
   * 
   * This function makes the following assumptions:
   *   - camera data is being published
   *   - at least one classifier exists
   * 
   * If recognition is stopped when this is called, it will start it, and stop it again once the call is complete.
   * It will wait for at least one classification result to be processed and then will return the current internal object state.
   */
  bool cb_getObjects(orp::GetObjects::Request &req,
    orp::GetObjects::Response &response);

  /**
   * Callback for when a classification result is published by any classifier.
   * @param newObject the passed message from the classifier topic
   */
  void cb_processNewClassification(orp::ClassificationResult newObject);
  
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
public:
  /**
   * Main Recognizer constructor.
   * Sets up internal configuration, creates ROS variables, and starts recognition (if autostart is enabled).
   * Autostart enabled by default, can be overriden with ROS parameter at command line or launch file
   */
  Recognizer();

  /// Dynamic Reconfigure callback.
  void paramsChanged(orp::RecognizerConfig &config, uint32_t level);
}; //Recognizer

#endif //_RECOGNIZER_H_
