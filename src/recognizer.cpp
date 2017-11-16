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

#include "orp/core/recognizer.h"

#include <algorithm>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <vision_msgs/Detection3DArray.h>

#include "orp/core/grasp_generator.h"

// program entry point
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));
  ros::init(argc, argv, "recognizer");

  //get started
  Recognizer s;

  //Run ROS until shutdown
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 1;
}

///////////////////////////////////////////////////////////////////////////////

Recognizer::Recognizer() :
    colocationDist(0.05),
    dirty(false),
    typeManager(),
    visionInfo(),

    markerTopic(),
    objectTopic(),

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
  if(!privateNode.getParam("detection_topic", markerTopic)) {
    markerTopic = "orp/detected_object_markers";
  }
  if(!privateNode.getParam("detection_markers_topic", objectTopic)) {
    objectTopic = "orp/detected_objects";
  }
  if(!privateNode.getParam("classification_topic", classificationTopic)) {
    objectTopic = "orp/classification";
  }

  ROS_INFO_STREAM_NAMED("ORP Recognizer",
    "Recognition frame: " << recognitionFrame);

  //dynamic reconfigure
  reconfigureCallbackType =
      boost::bind(&Recognizer::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);

  //ROS clients and publishers
  markerPub = privateNode.advertise<visualization_msgs::MarkerArray>(
      markerTopic, 1, true);
  objectPub = n.advertise<vision_msgs::Detection3DArray>(objectTopic, 1);
  stopPub = n.advertise<std_msgs::Empty>("stop_recognition", 1, true);
  visionInfoPub = n.advertise<vision_msgs::VisionInfo>("vision_info", 1, true);

  // Create VisionInfo message with pipeline metadata
  visionInfo.header.frame_id = recognitionFrame;
  visionInfo.database_location = "/orp/items";
  visionInfo.database_version = 0;
  visionInfo.method = "ORP";

  objectPoseServer =
      n.advertiseService("get_object_pose",
        &Recognizer::getObjectPose, this);
  objectsServer =
      n.advertiseService("get_objects",
        &Recognizer::cb_getObjects, this);
  startSub =
      n.subscribe("orp/start_recognition", 1,
        &Recognizer::cb_startRecognition, this);
  stopSub =
      n.subscribe("orp/stop_recognition", 1,
        &Recognizer::cb_stopRecognition, this);

  transformListener = new tf::TransformListener();

  //this will wait until types have been added to the parameter server
  ROS_INFO_NAMED("ORP Recognizer", "Loading object types");
  typeManager.loadTypesFromParameterServer();

  if(autostart) {
    ROS_INFO_NAMED("ORP Recognizer", "Autostarting recognition");
    startRecognition();
  }
}

void Recognizer::recognize(const ros::TimerEvent& event)
{
  // update objects
  update();
  // publish markers
  publishROS();
  // cull old objects
  killStale();
}

void Recognizer::paramsChanged(orp::RecognizerConfig &config, uint32_t level)
{
  setRefreshInterval(config.refresh_interval);
  staleTime = ros::Duration(config.stale_time);
  colocationDist = config.colocation_dist;
  shouldDebugPrint = config.debug_print;

  showUnknownLabels = config.show_unknown_labels;
  showRecognitionProbability = config.show_recognition_probability;
  showPosition = config.show_position;
  showPose = config.show_pose;
  showPoseStdDev = config.show_pose_std_dev;
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

void Recognizer::cb_processNewClassification(orp::ClassificationResult objects)
{
  for(int i=0; i < objects.result.size(); ++i) {
    orp::WorldObject newObject = objects.result[i];
    if(newObject.label != "") {
      //transform into recognition frame
      std::string sourceFrame = newObject.pose.header.frame_id;
      if(sourceFrame != recognitionFrame) {
        std::string msg = "";
        if(!transformListener->canTransform(recognitionFrame, sourceFrame,
                                            ros::Time(0), &msg))
        {
          //can't determine object's pose in real world.
          ROS_WARN_STREAM_THROTTLE_NAMED(5.0f, "ORP Recognizer",
            "[Throttled at 5s] can't determine objects pose in frame " <<
            sourceFrame << " with respect to recognition frame " <<
            recognitionFrame << ": " << msg);
          return;
        }

        tf::Stamped<tf::Pose> source, dest;
        newObject.pose.header.stamp = ros::Time(0);
        tf::poseStampedMsgToTF(newObject.pose, source);
        transformListener->transformPose(recognitionFrame, source, dest);
        tf::poseStampedTFToMsg(dest, newObject.pose);
        newObject.pose.header.frame_id = recognitionFrame;
      }

      Eigen::Affine3d eigPose;
      tf::poseMsgToEigen(newObject.pose.pose, eigPose);

      WorldObjectPtr p = WorldObjectPtr(
        new WorldObject(colocationDist, &typeManager, newObject.label,
                        recognitionFrame, eigPose, 1.0f));
      p->setCloud(newObject.cloud);

      // build a list of objects sorted by their distance to the new one
      std::map<float, WorldObjectPtr> distances;
      for(auto it = model.begin(); it != model.end(); ++it)
      {
        distances[(**it).distanceTo(p)] = *it;
      }
      // std::sort(distances.begin(); distances.end());

      // go through the list, checking for colocation. If any of the objects
      // are colocated with the new one, STOP.
      bool colocated = false;
      for(auto distance : distances)
      {
        if(distance.second->isColocatedWith(p))
        {
          // try to merge
          bool mergedIn = distance.second->merge(p);
          // we found a colocation, so stop looking
          colocated = true;
          break;
        }
      }
      // if we make it all the way to the end with no colocation, then it's
      // time to create a new object
      if(!colocated)
      {
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
      classificationTopic,
      10,
      &Recognizer::cb_processNewClassification,
      this);

    timer = n.createTimer(ros::Duration(refreshInterval),
        boost::bind(&Recognizer::recognize, this, _1));
    timer.start();
  } else {
    ROS_ERROR_NAMED("ORP Recognizer",
                    "Attempted to start recognition, but already started");
  }
}

void Recognizer::stopRecognition() {
  if(isRecognitionStarted())
  {
    ROS_INFO("Stopping Visual Recognition");
    timer.stop();
    recognitionSub.shutdown();
    for(WorldObjectList::iterator it = model.begin();
        it != model.end(); ++it)
    {
      (*it)->setStale(true);
    }
    //say goodbye (send delete markers)
    publishROS();

    //clear all markers
    model.clear();
  } else {
    ROS_WARN_NAMED("ORP Recognizer",
        "Attempted to stop recognition, but it hasn't started.");
  }
}

void Recognizer::update()
{
  // Only update the stale objects if we have new data.
  // No camera points->no updates (lazy)
  if(!dirty) return;
  ros::Time now = ros::Time::now();
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    // decay probabiilty of detection over time
    (**it).setProbability((**it).getProbability() * 0.99);
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
  vision_msgs::Detection3DArray objectArray;
  objectArray.header.frame_id = recognitionFrame;
  objectArray.header.stamp = ros::Time::now();
  objectArray.header.seq = object_sequence;

  visualization_msgs::MarkerArray markerMsg, labelMsg;

  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    //create the object message
    // orp::WorldObject newObject;
    // tf::Pose intPose;
    // tf::poseEigenToTF((**it).getPose(), intPose);
    // tf::poseTFToMsg(intPose, newObject.pose.pose);

    // fill the new object with data
    // newObject.colocationDist = (**it).getColocationDistance();
    // newObject.probability = (**it).getProbability();
    // newObject.pose.header.frame_id = recognitionFrame;
    // newObject.pose.header.stamp = ros::Time::now();
    // newObject.pose.header.seq = object_sequence++;

    // newObject.cloud = (**it).getCloud();

    // objectMsg.objects.insert(objectMsg.objects.end(), newObject);



    // Create a hypothesis
    vision_msgs::ObjectHypothesisWithPose hypothesis;
    hypothesis.score = (**it).getProbability();
    // TODO(kukanani): actually provide a meaningful id
    // // newObject.label  = (**it).getType().getName();
    hypothesis.id = 42;
    tf::poseEigenToMsg((**it).getPose(), hypothesis.pose.pose);


    // create a detection
    vision_msgs::Detection3D newDetection;
    newDetection.header.frame_id = recognitionFrame;
    newDetection.header.stamp = ros::Time::now();
    newDetection.header.seq = object_sequence;
    newDetection.source_cloud = (**it).getCloud();

    // attach the hypothesis to the detection
    newDetection.results.push_back(hypothesis);

    // add the detection to the list
    objectArray.detections.push_back(newDetection);
    object_sequence++;

    //create the marker message
    std::vector<visualization_msgs::Marker> newMarkers = (**it).getMarkers();
    markerMsg.markers.insert(markerMsg.markers.end(),
        newMarkers.begin(), newMarkers.end());
  }
  //publish 'em
  markerPub.publish(markerMsg);
  objectPub.publish(objectArray);
}

bool Recognizer::cb_getObjects(orp::GetObjects::Request &req,
    orp::GetObjects::Response &response) {
  bool wasStarted = isRecognitionStarted();
  ros::Publisher startPub;
  if(!wasStarted) {
    startPub = n.advertise<std_msgs::Empty>("orp_start_recognition", 1, true);
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
    orp::WorldObject newObject;
    tf::Pose intPose;
    tf::poseEigenToTF((**it).getPose(), intPose);
    tf::poseTFToMsg(intPose, newObject.pose.pose);

    newObject.colocationDist = (**it).getColocationDistance();
    newObject.probability = (**it).getProbability();
    newObject.pose.header.frame_id = recognitionFrame;
    newObject.label  = (**it).getType().getName();
    newObject.cloud = (**it).getCloud();

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
    ROS_ERROR_STREAM_NAMED("ORP Recognizer",
      "No vision objects while trying to get most likely object of type "
      << wot.getName());
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
    ROS_WARN_STREAM_THROTTLE(30, "[ORP Recognizer] Item type " + name +
      " not found. Continuing with default unknown object type");
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


///////////////////////////////////////////////////////////////////////////////
// ROS shadows for internal functions

void Recognizer::cb_startRecognition(std_msgs::Empty msg) {
  startRecognition();
}

void Recognizer::cb_stopRecognition(std_msgs::Empty msg) {
  stopRecognition();
}