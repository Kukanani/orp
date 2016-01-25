#include "core/recognizer.h"

#include <algorithm>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include "grasp_generator.h"

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  //read arguments
  if(argc < 2) {
    ROS_FATAL("proper usage is 'recognizer [autostart]");
    return -1;
  }
  bool autostart = (argc >= 3 && std::string(argv[2]) == "true");

  //get started
  ros::init(argc, argv, "recognizer");
  Recognizer s(autostart);

  //Run ROS until shutdown
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 1;
} //main

///////////////////////////////////////////////////////////////////////////////////

Recognizer::Recognizer(bool _autostart) :
    autostart(_autostart),
    colocationDist(0.05),
    dirty(false),
    typeManager("unknown"),
    recognitionFrame("world"),

    markerTopic("/detected_object_markers"),
    objectTopic("/detected_objects"),

    showUnknownLabels(true),
    showRecognitionProbability(true),
    showPosition(true),
    showPose(false),
    showPoseStdDev(false),
    shouldDebugPrint(false),

    refreshInterval(0.01)
{

  //dynamic reconfigure
  reconfigureCallbackType = boost::bind(&Recognizer::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);

  //ROS clients and publishers
  markerPub = n.advertise<visualization_msgs::MarkerArray>(markerTopic, 1);
  objectPub = n.advertise<orp::WorldObjects>(objectTopic, 1);

  objectPoseServer = n.advertiseService("/get_object_pose", &Recognizer::getObjectPose, this);
  startSub = n.subscribe("orp_start_recognition", 1, &Recognizer::cb_startRecognition, this);
  stopSub = n.subscribe("orp_stop_recognition", 1, &Recognizer::cb_stopRecognition, this);

  objectBroadcaster = new tf::TransformBroadcaster();
  objectTransformer = new tf::Transformer();
  transformListener = new tf::TransformListener();

  ROS_INFO("Loading object types");
  typeManager.loadTypesFromParameterServer();

  if(autostart) {
    ROS_INFO("Autostarting recognition");
    startRecognition();
  }
}

Recognizer::~Recognizer()
{
  model.clear();
}

void Recognizer::recognize(const ros::TimerEvent& event)
{
  update();
  publishROS();                          //publish all markers
  killStale();
  
  //ROS_INFO("===========");
}

//use the form
//local_var_name = config.config_var_name;
void Recognizer::paramsChanged(orp::RecognizerConfig &config, uint32_t level)
{
  staleTime           = ros::Duration(config.stale_time);
  colocationDist      = config.colocation_dist;
  setRefreshInterval(config.refresh_interval);

  showUnknownLabels          = config.show_unknown_labels;
  showRecognitionProbability = config.show_recognition_probability;
  showPosition               = config.show_position;
  showPose                   = config.show_pose;
  showPoseStdDev             = config.show_pose_std_dev;

  shouldDebugPrint = config.debug_print;

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
//   ROS_ERROR_STREAM("found->getPose(): " << originalPose.getOrigin().x() << ", " << originalPose.getOrigin().y() << "," << originalPose.getOrigin().z());
  tf::poseTFToMsg(originalPose, pose.pose);
//   ROS_ERROR_STREAM("pose.pose: " << pose.pose.position.x << ", " << pose.pose.position.y << "," << pose.pose.position.z);

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = recognitionFrame;
  response.poses.push_back(pose);
  response.num_found++;
//   ROS_INFO_STREAM("Recognizer returned pose to get_object_pose: " << pose.pose.position.x << ", " << pose.pose.position.y <<
//     ", " << pose.pose.position.z);

  return true;
} //getObjectPose

void Recognizer::cb_classificationResult(orp::ClassificationResult newObject)
{
  Eigen::Affine3d eigPose;
  tf::poseMsgToEigen(newObject.result.pose.pose, eigPose);

  bool merged = false;
  
  WorldObjectPtr p = WorldObjectPtr(new WorldObject(colocationDist,&typeManager,newObject.result.label, recognitionFrame, eigPose, 1.0f));
  for(WorldObjectList::iterator it = model.begin(); it != model.end() && !merged; ++it) {
    if((*it)->isColocatedWith(p)) {
      merged = true;
      (*it)->merge(p);
      //ROS_INFO("Merged object");
    }
  }

  if(!merged) { //new object
    model.push_back(p);
  }
  dirty = true; //queue an update
}

void Recognizer::startRecognition() {
  if(recognitionSub == NULL)
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
    ROS_ERROR("attempted to start recognition, but already started");
  }
} //startRecognition

void Recognizer::stopRecognition() {
  if(recognitionSub != NULL)
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
    ROS_WARN("attempted to stop recognition, but it hasn't started.");
  }
} //stopRecognition

void Recognizer::cb_startRecognition(std_msgs::Empty msg) {
  startRecognition();
} //cb_startRecognition

void Recognizer::cb_stopRecognition(std_msgs::Empty msg) {
  stopRecognition();
} //cb_startRecognition

void Recognizer::update()
{
  //only update the stale objects if we have new data. No camera points->no updates (lazy)
  if(!dirty) return;
  ros::Time now = ros::Time::now();
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    if(now - (**it).getLastUpdated() > staleTime)
    {
      //ROS_INFO("stale-ing world object %i", (**it).getID());
      (**it).setStale(true);
    }
    else {
      (**it).setStale(false);
    }
  }
  dirty = false;
} //update

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
    newObject.label  = (**it).getType().getName();
    
    objectMsg.objects.insert(objectMsg.objects.end(), newObject);
    
    
    //create the marker message
    std::vector<visualization_msgs::Marker> newMarkers = (**it).getMarkers();
    markerMsg.markers.insert(markerMsg.markers.end(), newMarkers.begin(), newMarkers.end());
  }
  //publish 'em
  //ROS_INFO_STREAM("Publishing " << objectMsg.objects.size() << " objects and " << markerMsg.markers.size() << " markers.");
  markerPub.publish(markerMsg);
  objectPub.publish(objectMsg);
}

void Recognizer::killStale() {
  WorldObjectList::iterator it = model.begin();
  while(it != model.end()) {
    bool isStale = (*it)->isStale();
    if(isStale) {
      //ROS_INFO_STREAM("Deleting stale object " << (**it).getID());
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
    ROS_ERROR_STREAM("Recognizer: No vision objects while trying to get most likely object of type " << wot.getName());
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
}; //getMostLikelyObjectOfType

WorldObjectPtr Recognizer::getMostLikelyObjectOfType(std::string name)
{
  WorldObjectType type = typeManager.getUnknownType();
  try {
    typeManager.getTypeByName(name);
  } catch(std::logic_error le) {
    ROS_WARN_STREAM_THROTTLE(30, "Item type " + name + " not found. Continuing with default unknown object type");
  }
  return getMostLikelyObjectOfType(type);
}; //getMostLikelyObjectOfType

void Recognizer::setRefreshInterval(float interval)
{
  refreshInterval = ros::Duration(interval);
  if(timer != NULL)
  {
    timer.setPeriod(refreshInterval);
    timer.start();
  }
}; //setRefreshInterval
