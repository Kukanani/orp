///////////////////////////////////////////////////////////////////////////////
//      Title     : Recognizer
//      Project   : NRG ORP
//      Created   : 1/21/2015
//      Author    : Adam Allevato
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "core/recognizer.h"

#include <algorithm>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include "grasp_generator.h"

// program entry point
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));
  ros::init(argc, argv, "recognizer");

  //get started
  Recognizer s();

  //Run ROS until shutdown
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 1;
}

///////////////////////////////////////////////////////////////////////////////////

Recognizer::Recognizer() :
    colocationDist(0.05),
    dirty(false),
    typeManager(),

    markerTopic("/detected_object_markers"),
    objectTopic("/detected_objects"),

    showUnknownLabels(true),
    showRecognitionProbability(true),
    showPosition(true),
    showPose(false),
    showPoseStdDev(false),
    shouldDebugPrint(false),
    
    classification_count(0),
    object_sequence(0),
    refreshInterval(0.01)
{
  // load any overrides from parameters, otherwise use defaults
  ros::NodeHandle privateNode("~");
  if(!privateNode.getParam("recognition_frame", recognitionFrame)) {
    recognitionFrame = "world";
  }
  if(!privateNode.getParam("autostart", autostart)) {
    autostart = true;
  }

  ROS_INFO_STREAM("[ORP Recognizer] Recognition frame: " << recognitionFrame);

  //dynamic reconfigure
  reconfigureCallbackType = boost::bind(&Recognizer::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);

  //ROS clients and publishers
  markerPub = n.advertise<visualization_msgs::MarkerArray>(markerTopic, 1, true);
  objectPub = n.advertise<orp::WorldObjects>(objectTopic, 1);
  stopPub = n.advertise<std_msgs::Empty>("/orp_stop_recognition", 1, true);

  objectPoseServer = n.advertiseService("/get_object_pose", &Recognizer::getObjectPose, this);
  objectsServer = n.advertiseService("/get_objects", &Recognizer::cb_getObjects, this);
  startSub = n.subscribe("orp_start_recognition", 1, &Recognizer::cb_startRecognition, this);
  stopSub = n.subscribe("orp_stop_recognition", 1, &Recognizer::cb_stopRecognition, this);

  objectBroadcaster = new tf::TransformBroadcaster();
  transformListener = new tf::TransformListener();

  //this will wait until types have been added to the parameter server
  ROS_INFO("[ORP Recognizer] Loading object types");
  typeManager.loadTypesFromParameterServer();

  if(autostart) {
    ROS_INFO("[ORP Recognizer] Autostarting recognition");
    startRecognition();
  }
}

void Recognizer::recognize(const ros::TimerEvent& event)
{
  update();     // update objects 
  publishROS(); // publish markers
  killStale();  // cull old objects
}

void Recognizer::paramsChanged(orp::RecognizerConfig &config, uint32_t level)
{
  setRefreshInterval(config.refresh_interval);
  staleTime                  = ros::Duration(config.stale_time);
  colocationDist             = config.colocation_dist;
  shouldDebugPrint           = config.debug_print;

  showUnknownLabels          = config.show_unknown_labels;
  showRecognitionProbability = config.show_recognition_probability;
  showPosition               = config.show_position;
  showPose                   = config.show_pose;
  showPoseStdDev             = config.show_pose_std_dev;
}

bool Recognizer::getObjectPose(orp::GetObjectPose::Request &req,
  orp::GetObjectPose::Response &response)
{
  response.num_found = 0;
  WorldObjectPtr found = getMostLikelyObjectOfType(req.name);

  if(!found) return true;

  geometry_msgs::PoseStamped pose;
  tf::Pose originalPose;
  tf::poseEigenToTF (found->getPose(), originalPose);
  tf::poseTFToMsg(originalPose, pose.pose);

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = recognitionFrame;
  response.poses.push_back(pose);
  response.num_found++;

  return true;
}

void Recognizer::cb_classificationResult(orp::ClassificationResult objects)
{
  for(int i=0; i < objects.result.size(); ++i) {
    obj_interface::WorldObject newObject = objects.result[i];
    if(newObject.label != "") {
      //transform into recognition frame
      std::string sourceFrame = newObject.pose.header.frame_id;
      if(sourceFrame != recognitionFrame) {
        std::string msg = "";
        if(!transformListener->canTransform(recognitionFrame, sourceFrame, ros::Time(0), &msg)) {
          //can't determine object's pose in real world.
          ROS_WARN_STREAM_THROTTLE(5.0f, "[recognizer] [Throttled at 5s] can't determine objects pose in frame " << sourceFrame << " with respect to recognition frame " << recognitionFrame << ": " << msg);
          return;
        }
        
        //I would like to use tf::Stamped<tf::Pose> here but it seems to create more issues than it solves. More straightforward to manually set the frames
        tf::Stamped<tf::Pose> source, dest;
        newObject.pose.header.stamp = ros::Time(0);
        tf::poseStampedMsgToTF(newObject.pose, source);
        transformListener->transformPose(recognitionFrame, source, dest);
        tf::poseStampedTFToMsg(dest, newObject.pose);
        newObject.pose.header.frame_id = recognitionFrame;
      }
      
      bool merged = false;
      Eigen::Affine3d eigPose;
      tf::poseMsgToEigen(newObject.pose.pose, eigPose);
      
      WorldObjectPtr p = WorldObjectPtr(new WorldObject(colocationDist,&typeManager,newObject.label, recognitionFrame, eigPose, 1.0f));
      for(WorldObjectList::iterator it = model.begin(); it != model.end() && !merged; ++it) {
        if((*it)->isColocatedWith(p)) {
          merged = true;
          // let the objects figure out how to merge themselves
          (*it)->merge(p);
        }
      }

      if(!merged) { //new object
        model.push_back(p);
      }
    }
  }
  if(objects.result.size() > 0) {
    dirty = true; //queue an update
    classification_count += objects.result.size();
  }
}

bool Recognizer::isRecognitionStarted() {
  return (recognitionSub != NULL);
}

void Recognizer::startRecognition() {
  if(!isRecognitionStarted())
  {
    ROS_INFO("Starting Visual Recognition");
    recognitionSub = n.subscribe(
      "/classification",
      10,
      &Recognizer::cb_classificationResult,
      this);

    timer = n.createTimer(ros::Duration(refreshInterval), boost::bind(&Recognizer::recognize, this, _1));
    timer.start();
  } else {
    ROS_ERROR("[ORP Recognizer] Attempted to start recognition, but already started");
  }
}

void Recognizer::stopRecognition() {
  if(isRecognitionStarted())
  {
    ROS_INFO("Stopping Visual Recognition");
    timer.stop();
    recognitionSub.shutdown();
    for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it) {
      (*it)->setStale(true);
    }
    //say goodbye (send delete markers)
    publishROS();
    
    //clear all markers
    model.clear();
  } else {
    ROS_WARN("[ORP Recognizer] Attempted to stop recognition, but it hasn't started.");
  }
}

void Recognizer::update()
{
  //only update the stale objects if we have new data. No camera points->no updates (lazy)
  if(!dirty) return;
  ros::Time now = ros::Time::now();
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    if(now - (**it).getLastUpdated() > staleTime)
    {
      (**it).setStale(true);
    }
    else {
      (**it).setStale(false);
    }
  }
  dirty = false;
}

void Recognizer::publishROS()
{
  //add all world objects to message
  orp::WorldObjects objectMsg;
  visualization_msgs::MarkerArray markerMsg, labelMsg;
  
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    //create the object message
    obj_interface::WorldObject newObject;
    tf::Pose intPose;
    tf::poseEigenToTF((**it).getPose(), intPose);
    tf::poseTFToMsg(intPose, newObject.pose.pose);
    
    newObject.colocationDist = (**it).getColocationDistance();
    newObject.probability = (**it).getProbability();
    newObject.pose.header.frame_id = recognitionFrame;
    newObject.pose.header.stamp = ros::Time::now();
    newObject.pose.header.seq = object_sequence++;
    newObject.label  = (**it).getType().getName();
    
    objectMsg.objects.insert(objectMsg.objects.end(), newObject);

    //create the marker message
    std::vector<visualization_msgs::Marker> newMarkers = (**it).getMarkers();
    markerMsg.markers.insert(markerMsg.markers.end(), newMarkers.begin(), newMarkers.end());
  }
  //publish 'em
  markerPub.publish(markerMsg);
  objectPub.publish(objectMsg);
}

bool Recognizer::cb_getObjects(orp::GetObjects::Request &req,
    orp::GetObjects::Response &response) {
  bool wasStarted = isRecognitionStarted();
  ros::Publisher startPub;
  if(!wasStarted) {
    startPub = n.advertise<std_msgs::Empty>("/orp_start_recognition", 1, true);
    startPub.publish(std_msgs::Empty());
  }
  //block until classification message is published
  int orig_classification_count = classification_count;
  while(classification_count == orig_classification_count) {
    refreshInterval.sleep();
    ros::spinOnce();
  }
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    //create the object message
    obj_interface::WorldObject newObject;
    tf::Pose intPose;
    tf::poseEigenToTF((**it).getPose(), intPose);
    tf::poseTFToMsg(intPose, newObject.pose.pose);
    
    newObject.colocationDist = (**it).getColocationDistance();
    newObject.probability = (**it).getProbability();
    newObject.pose.header.frame_id = recognitionFrame;
    newObject.label  = (**it).getType().getName();
    
    response.objects.objects.push_back(newObject);
  }
  if(!wasStarted) {
    stopPub.publish(std_msgs::Empty());
  }
  return true;
}

void Recognizer::killStale() {
  WorldObjectList::iterator it = model.begin();
  while(it != model.end()) {
    bool isStale = (*it)->isStale();
    if(isStale) {
      it = model.erase(it); //increments it
    }
    else {
      ++it;
    }
  }
}

WorldObjectPtr Recognizer::getMostLikelyObjectOfType(WorldObjectType wot)
{
  float max = 0.0;
  float prob = 0.0;
  best = WorldObjectPtr();
  if(model.size() < 1)
  {
    ROS_ERROR_STREAM("[ORP Recognizer] No vision objects while trying to get most likely object of type " << wot.getName());
  }
  
  int i = 0;
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); it++)
  {
    prob = (**it).getProbability();
    if(prob > max)
    {
      max = prob;
      best = *it;
    }
    ++i;
  }
  return best;
}

WorldObjectPtr Recognizer::getMostLikelyObjectOfType(std::string name)
{
  WorldObjectType type = typeManager.getUnknownType();
  try {
    typeManager.getTypeByName(name);
  } catch(std::logic_error le) {
    ROS_WARN_STREAM_THROTTLE(30, "[ORP Recognizer] Item type " + name + " not found. Continuing with default unknown object type");
  }
  return getMostLikelyObjectOfType(type);
}

void Recognizer::setRefreshInterval(float interval)
{
  refreshInterval = ros::Duration(interval);
  if(timer != NULL)
  {
    timer.setPeriod(refreshInterval);
    timer.start();
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
// ROS shadows for internal functions

void Recognizer::cb_startRecognition(std_msgs::Empty msg) {
  startRecognition();
}

void Recognizer::cb_stopRecognition(std_msgs::Empty msg) {
  stopRecognition();
}