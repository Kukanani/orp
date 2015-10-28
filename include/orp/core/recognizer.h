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

#include <orp/RecognizerConfig.h>
#include <orp/GetObjectPose.h>
#include <orp/DetectionSet.h>
#include <orp/WorldObjects.h>

#include "orp/core/world_object.h"
#include "orp/core/world_object_manager.h"
#include "orp/core/orp_utils.h"

/**
 * roll, pitch and yaw (orientation) struct
 */
struct RPY {
  double roll; //x axis
  double pitch; //y axis
  double yaw; //z axis

  RPY() : roll(0), pitch(0), yaw(0) {};
};


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
 * @copyright BSD 3-paragraph
 * @date      1/29/2015
 */
class Recognizer {
private:
  /// Standard ROS node handle
  ros::NodeHandle n;
  /// Enables usage of dynamic_reconfigure.
  dynamic_reconfigure::Server<orp::RecognizerConfig> reconfigureServer;
  dynamic_reconfigure::Server<orp::RecognizerConfig>::CallbackType reconfigureCallbackType;

// ADDITIONAL VARIABLES GO HERE

  ros::Timer timer;                     /// Used to schedule the recognition calls
  std::string recognitionFrame;         /// What frame the objects are being detected in
  WorldObjectList model;                /// A list of currently-accepted objects in the world.
  WorldObjectManager* typeManager;      /// Manages the list of world object types.
  std::string objectTopic;              /// Where to publish the WorldObjects
  std::string markerTopic;              /// Where to publish the RViz markers for visualization
  visualization_msgs::MarkerArray markerMsg; /// The marker message to send out after each recognition. Used in multiple methods.

  std::vector<std::string> typeList;    /// List of object names that the cph can recognize.
  std::vector<std::string> subTypeList;    /// List of object names that the cph can recognize.
  std::map<std::string, std::pair<visualization_msgs::Marker, RPY> > markerStubs; //used to create object markers

  ros::Subscriber recognitionSub;       /// Listens for new recognized objects and adds them to the model.
  ros::Subscriber detectionSetSub;      /// Listens for a subset of objects to detect.
  ros::Publisher objectPub;             /// Publishes a MarkerArray with information about detected objects
  ros::Publisher markerPub;             /// Publishes a WorldObjects message that contains all the recognized objects from 
  tf::TransformBroadcaster* objectBroadcaster; /// broadcast frames for each found object
  tf::TransformListener* transformListener;    /// listen for necessary transformations before publishing
  tf::Transformer* objectTransformer;          /// transform object poses from one frame to another
  ros::ServiceServer objectPoseServer;  /// Provides poses for requested objects.

  bool autostart;                       /// if true, begin the recognition loop automatically (auto-subscribe to classification)
  ros::Subscriber startSub;             /// to start recognition loop
  ros::Subscriber stopSub;              /// to stop recognition loop

  WorldObjectPtr best;                  /// used to store best result when looping through model in a search

// DYNAMICALLY-SET CONFIG VARIABLES
  ros::Duration staleTime;          /// Number of seconds that an object will persist in the world model after being updated
  float colocationDist;             /// Distance, in m, that objects must be separated by in order to be thought of as separate objects.
  ros::Duration refreshInterval;    /// The wait between recognition calls

  float markerRed;                  ///Red component of RViz marker color (0 to 1)
  float markerGreen;                ///Green component of RViz marker color (0 to 1)
  float markerBlue;                 ///Blue component of RViz marker color (0 to 1)
  float markerAlpha;                ///Alpha component of RViz marker color (1.0 = opaque)
  float markerSize;                 ///Marker size (height of uppercase "A")

  bool showUnknownLabels;           ///Show "unknown" labels
  bool showRecognitionProbability;  ///Show the decimal recognition probability
  bool showPosition;                ///Show objects' (X,Y,Z) positions
  bool showPose;                    ///Show objects' poses (rotation about Y-axis)
  bool showPoseStdDev;              ///Show standared deviation of pose
  bool shouldDebugPrint;                  ///Call debugPrint() on objects in the object model

  /**
   * Callback for when a classification result is published by any classifier.
   * @param newObject the passed message from the classifier topic
   */
  void cb_classificationResult(orp::ClassificationResult newObject);

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

  /**
   * loads info from the parameter server and stores basic information about each item
   * being detected.
   */
  void fillMarkerStubs();

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

  /**
   * Filter objects in the scene. This performs necessary
   * steps such as removing duplicates. Call it after a call to classify().
   */
  void filter();

  /**
   * Removes any objects that are older than the "stale time" (i.e., haven't
   * been detected for a while)
   */
  void killStale();

  /**
   * List out the current model (list of objects) to the console
   */
  void debugPrint();

  /**
   * Prepares and sends out ROS-related messages with classification information.
   * This includes object information as well as RViz markers.
   */
  void publishROS();

  /// Get a pre-loaded stub, or return an unknown stub if one can't be found
  std::pair<visualization_msgs::Marker, RPY> getStubAt(std::string);

  /**
   * Creates a new marker for the given object. This should be called when adding an object
   * to the scene if you want it to have a corresponding marker.
   * @arg wo the WorldObjectPtr to add
   */
  void addMarker(WorldObjectPtr wo);

  /**
   * Generate the human-readable marker label for this object.
   */
  const char* generateMarkerLabel(WorldObject& wo);

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

  void setDetectionSet(std::vector<std::string> set);

  /// ROS wrappers
  void cb_startRecognition(std_msgs::Empty msg); 
  void cb_stopRecognition(std_msgs::Empty msg);
  void cb_detectionSet(orp::DetectionSet msg);
public:
  /**
   * Main Recognizer constructor.
   * @arg nh The NodeHandle to use for all ROS functionality.
   * @arg autostart whether or not to start recognition automatically. Default is false (wait for a message to be published to /orp_start_recognition)
   */
  Recognizer(bool autostart = false);
  /**
   * Main Recognizer destructor
   */
  ~Recognizer();

  /// Dynamic Reconfigure callback.
  void paramsChanged(
    orp::RecognizerConfig &config, uint32_t level);
}; //Recognizer

#endif //_RECOGNIZER_H_