#include "orp/core/recognizer.h"

#include <algorithm>

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
    ROS_FATAL("proper usage is 'recognizer sensor_model_file [autostart]");
    return -1;
  }
  bool autostart = false;
  std::string listFile = argv[1];
  if(argc >= 3) {
    if(std::string(argv[2]) == "true") {
      autostart = true;
    }
  }

  //get started
  ros::init(argc, argv, "recognizer");
  Recognizer s(autostart);

  //Run ROS until shutdown
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  return 1;
} //main

///////////////////////////////////////////////////////////////////////////////////

Recognizer::Recognizer(bool _autostart) :
    autostart(_autostart),
    colocationDist(0.01),
    typeManager(0),
    recognitionFrame("camera_depth_optical_frame"),

    markerTopic("/detected_object_markers"),
    objectTopic("/detected_objects"),
    markerMsg(),
    markerAlpha(1.0),
    markerSize(0.03),

    showUnknownLabels(true),
    showRecognitionProbability(true),
    showPosition(true),
    showPose(false),
    showPoseStdDev(false),
    shouldDebugPrint(false),

    refreshInterval(0.1)
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

  detectionSetSub = n.subscribe(
    "/detection_set",
    1,
    &Recognizer::cb_detectionSet,
    this);

  objectBroadcaster = new tf::TransformBroadcaster();
  objectTransformer = new tf::Transformer();
  transformListener = new tf::TransformListener();

  typeManager = new WorldObjectManager("unknown");

  ROS_INFO("initializing sensor model");

  fillMarkerStubs();

  if(autostart) {
    ROS_INFO("Autostarting recognition");
    startRecognition();
  }
} //Recognizer constructor

Recognizer::~Recognizer()
{
  model.clear();
  delete typeManager;
} //Recognizer destructor

//use the form
//local_var_name = config.config_var_name;
void Recognizer::paramsChanged(orp::RecognizerConfig &config, uint32_t level)
{
  staleTime           = ros::Duration(config.stale_time);
  colocationDist      = config.colocation_dist;
  setRefreshInterval(config.refresh_interval);

  markerAlpha = config.marker_alpha;
  markerSize  = config.marker_size;

  showUnknownLabels          = config.show_unknown_labels;
  showRecognitionProbability = config.show_recognition_probability;
  showPosition               = config.show_position;
  showPose                   = config.show_pose;
  showPoseStdDev             = config.show_pose_std_dev;

  shouldDebugPrint = config.debug_print;

} //paramsChanged

void Recognizer::fillMarkerStubs() {
  for(std::vector<std::string>::iterator names = typeList.begin(); names != typeList.end(); names++)
  {
    visualization_msgs::Marker stub;
    RPY rpy;

    if(*names == "unknown") {
      stub.scale.x = 0.1;
      stub.scale.y = 0.1;
      stub.scale.z = 0.1;

      stub.type = visualization_msgs::Marker::CUBE;

      rpy.roll = 0.0;
      rpy.pitch = 0.0;
      rpy.yaw = 0.0;
    } else {
      try {
        std::string geom;
        ORPUtils::attemptToReloadStringParam(n, "/items/" + *names + "/geometry", geom);
        if(geom == "BOX") {
          stub.type            = visualization_msgs::Marker::CUBE;
        } else if(geom == "CYLINDER") {
          stub.type            = visualization_msgs::Marker::CYLINDER;
        } else if(geom == "FLAT") {
          stub.type            = visualization_msgs::Marker::CUBE;
        } else if(geom == "BLOB") {
          stub.type            = visualization_msgs::Marker::SPHERE;
        } else {
          ROS_ERROR("Did not understand geometry type %s while creating marker stubs", geom.c_str());
          stub.type            = visualization_msgs::Marker::CUBE;
        }
        ORPUtils::attemptToReloadDoubleParam(n, "/items/" + *names + "/depth", stub.scale.x);
        ORPUtils::attemptToReloadDoubleParam(n, "/items/" + *names + "/width", stub.scale.y);
        ORPUtils::attemptToReloadDoubleParam(n, "/items/" + *names + "/height", stub.scale.z);

        ORPUtils::attemptToReloadDoubleParam(n, "/items/" + *names + "/roll", rpy.roll);
        ORPUtils::attemptToReloadDoubleParam(n, "/items/" + *names + "/pitch", rpy.pitch);
        ORPUtils::attemptToReloadDoubleParam(n, "/items/" + *names + "/yaw", rpy.yaw);

        ORPUtils::attemptToReloadFloatParam(n, "/items/" + *names + "/red", stub.color.r);
        ORPUtils::attemptToReloadFloatParam(n, "/items/" + *names + "/green", stub.color.g);
        ORPUtils::attemptToReloadFloatParam(n, "/items/" + *names + "/blue", stub.color.b);

        rpy.roll = ORPUtils::radFromDeg(rpy.roll);
        rpy.pitch = ORPUtils::radFromDeg(rpy.pitch);
        rpy.yaw = ORPUtils::radFromDeg(rpy.yaw);
      } catch(std::exception e) {
        ROS_ERROR("error while creating marker stub for world object of type '%s': %s", e.what(), (*names).c_str());
        stub.scale.x = 0.1;
        stub.scale.y = 0.1;
        stub.scale.z = 0.1;

        stub.color.r = 0.0;
        stub.color.g = 0.0;
        stub.color.b = 0.0;

        stub.type = visualization_msgs::Marker::CUBE;

        rpy.roll = 0.0;
        rpy.pitch = 0.0;
        rpy.yaw = 0.0;
      }
      if(stub.scale.x > 1 && stub.scale.y > 1 && stub.scale.z > 1) { //detect sizes in mm instead of m
        stub.scale.x /= 1000.0f;
        stub.scale.y /= 1000.0f;
        stub.scale.z /= 1000.0f;
      }
    }
    //ROS_INFO("creating marker stub for world object of type '%s'", (*names).c_str());
    stub.header.frame_id = recognitionFrame;
    stub.action          = visualization_msgs::Marker::ADD;
    markerStubs[*names] = std::pair<visualization_msgs::Marker, RPY>(stub, rpy);
  }
  ROS_INFO("The recognizer has created %i visualization marker stubs.", (int) markerStubs.size());
}

bool Recognizer::getObjectPose(orp::GetObjectPose::Request &req,
  orp::GetObjectPose::Response &response)
{
  response.num_found = 0;
  WorldObjectPtr found = getMostLikelyObjectOfType(req.name);

  if(!found) return true;

  geometry_msgs::PoseStamped pose;
  tf::Pose originalPose;
  tf::poseEigenToTF (found->getPose(req.name), originalPose);
  ROS_ERROR_STREAM("found->getPose(" << req.name << "): " << originalPose.getOrigin().x() << ", " << originalPose.getOrigin().y() << "," << originalPose.getOrigin().z());
  tf::poseTFToMsg(originalPose, pose.pose);
  ROS_ERROR_STREAM("pose.pose: " << pose.pose.position.x << ", " << pose.pose.position.y << "," << pose.pose.position.z);

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = recognitionFrame;
  response.poses.push_back(pose);
  response.num_found++;
  ROS_INFO_STREAM("Recognizer returned pose to get_object_pose: " << pose.pose.position.x << ", " << pose.pose.position.y <<
    ", " << pose.pose.position.z);

  return true;
} //getObjectPose

std::pair<visualization_msgs::Marker, RPY> Recognizer::getStubAt(std::string name) {
  std::pair<visualization_msgs::Marker, RPY> stubby;
  try {
    stubby = markerStubs.at(name);
  } catch(std::out_of_range oor) {
    ROS_ERROR("marker stub not found for incoming classification of type %s", name.c_str());
    ROS_ERROR("continuing with 'unknown' marker");
    stubby = markerStubs.at("unknown");
  }
  return stubby;
}

void Recognizer::cb_classificationResult(orp::ClassificationResult newObject)
{
  //ROS_INFO("object incoming, type %s...", newObject.result.label.c_str());
  RPY rpy;

  TypeMap probs;
  tf::Pose stubAdjustmentPose;
  // for(auto it = subTypeList.begin(); it != subTypeList.end(); it++) {
     stubAdjustmentPose = tf::Pose(tf::createQuaternionFromRPY(
       getStubAt(newObject.result.label).second.roll,
       getStubAt(newObject.result.label).second.pitch,
       getStubAt(newObject.result.label).second.yaw
     ));
   
     Eigen::Affine3d eigPose;
     tf::poseTFToEigen(stubAdjustmentPose, eigPose);

  //   float probability = 1.0f;
  //   try {
  //     probability = 0.0
  //   } catch(std::out_of_range oor) {
  //     //swallowed, just default to probability 1
  //   }
    //probs.insert(TypeMap::value_type(*it, PoseGuess(probability, eigPose)));
    probs.insert(TypeMap::value_type(newObject.result.label, PoseGuess(1.0f, eigPose)));
  // }

  //IMPORTANT HACK (TODO): adjust the item's position upwards by half its height, so it will be sitting on the ground.
  //newObject.result.pose.pose.position.z += 0;//getStubAt(newObject.result.label).first.scale.z / 1.0f;


  if(newObject.method == "cph")
  {
    WorldObjectPtr p = WorldObjectPtr(
      new WorldObject(
                                  colocationDist,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    addMarker(p);
  }
  else if(newObject.method == "vfh")
  {
    WorldObjectPtr p = WorldObjectPtr(
      new WorldObject(
                                  colocationDist,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    addMarker(p);
  }
  else if(newObject.method == "sixdof")
  {

    WorldObjectPtr p = WorldObjectPtr(
      new WorldObject(
                                  colocationDist,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    addMarker(p);
  }
  else if(newObject.method == "rgb")
  {
    WorldObjectPtr p = WorldObjectPtr(
      new WorldObject(
                                  colocationDist,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    addMarker(p);
  }
  else
  {
    ROS_ERROR("unknown classification method %s", newObject.method.c_str());
  }
  ROS_DEBUG("Added object");
} //cb_classificationResult

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
      deleteMarker(*it);
    }
    //say goodbye (send delete markers)
    publishROS();
    
    //clear all markers
    model.clear();
  } else {
    ROS_ERROR("attempted to stop recognition, but it hasn't started or the timer doesn't exist.");
  }
} //stopRecognition

void Recognizer::setDetectionSet(std::vector<std::string> set) {
  if(set.size() > typeList.size()) {
    ROS_ERROR("your desired subset is larger than the number of known items. Something's wrong.");
  }

  subTypeList = typeList; // begin with them equal

  std::vector<std::string>::iterator it = std::set_intersection(typeList.begin(), typeList.end(),
    set.begin(), set.end(), subTypeList.begin());

  subTypeList.resize(it-subTypeList.begin());

  if(subTypeList.empty()) { //nothing left!
    ROS_FATAL("No items left in Recognizer detection set after specifying a subset");
    return;
  }
} //setDetectionSet

void Recognizer::cb_startRecognition(std_msgs::Empty msg) {
  startRecognition();
} //cb_startRecognition

void Recognizer::cb_stopRecognition(std_msgs::Empty msg) {
  stopRecognition();
} //cb_startRecognition

void Recognizer::cb_detectionSet(orp::DetectionSet msg)
{
  setDetectionSet(msg.objects);
} //cb_detectionSet

void Recognizer::recognize(const ros::TimerEvent& event)
{
  if(shouldDebugPrint) debugPrint();     //list markers 
  killStale();
  filter();                              //set some markers to DELETE
  publishROS();                          //publish all markers
  if(shouldDebugPrint) debugPrint();     //list markers 
  markerMsg.markers.clear();

  //ROS_INFO("===========");
} //recognize

void Recognizer::filter()
{
  if(model.size() < 2) {
    return;
  }

  WorldObjectList deleteList;

  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    //TODO: replace boost::next with std::next when C++11 support is added
    for(WorldObjectList::iterator it2 = boost::next(it,1); it2 != model.end(); ++it2)
    {
      // ROS_INFO("dist: %f", tf::tfDistance((*it)->getPoseTf().getOrigin(), (*it2)->getPoseTf().getOrigin()));
      if(std::find(deleteList.begin(), deleteList.end(), *it2) == deleteList.end() &&
          (**it).shouldMergeWith(&(**it2))) {
        // ROS_INFO_STREAM("merging objects " << (**it).getID() << " and " << (**it2).getID());
        (**it).merge(&(**it2));
        deleteList.push_back(*it2);
      }
    }
  }

  //ROS_INFO_STREAM("deleting " << (int)deleteList.size() << "items");
  for(WorldObjectList::iterator it = deleteList.begin(); it != deleteList.end(); ++it)
  {
    deleteMarker(*it);
    model.remove(*it);
  }
  //ROS_INFO_STREAM("done deleting stuff: " << (int)deleteList.size() << ", " << (int)model.size());
} //filter

void Recognizer::killStale()
{
  ros::Time now = ros::Time::now();
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); )
  {
    if((**it).isStale() || now - (**it).getLastUpdated() > staleTime)
    {
      //ROS_INFO("killing stale world object %i", (**it).getID());
      deleteMarker(*it);
      model.erase(it++);
    }
    else
    {
      ++it;
    }
  }
} //killStale

void Recognizer::debugPrint()
{
  ROS_INFO("%d Objects in scene: ", (int)model.size());
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); it++)
  {
    (**it).debugPrint();
  }

  ROS_INFO("MARKERS:");
  for(std::vector<visualization_msgs::Marker>::iterator it2 = markerMsg.markers.begin();
    it2 != markerMsg.markers.end(); ++it2)
  {
    if((*it2).action == visualization_msgs::Marker::DELETE)
    {
      ROS_INFO("\tDELETE: %i", (*it2).id);
    }
    else if((*it2).action == visualization_msgs::Marker::ADD)
    {
      ROS_INFO("\tADD: %i", (*it2).id);
    }
    else
    {
      ROS_INFO("\tUNKNOWN: %i", (*it2).id);
    }
  }
} //debugPrint

void Recognizer::publishROS()
{
  //add all world objects to message
  orp::WorldObjects objectMsg;
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    orp::WorldObject newObject;
    newObject.label              = (**it).getBestType().name;

    tf::Pose intPose;
    tf::poseEigenToTF((**it).getBestPose(), intPose);
    tf::poseTFToMsg(intPose, newObject.pose.pose);

    newObject.pose.header.frame_id = "world";
    objectMsg.objects.insert(objectMsg.objects.end(), newObject);
  }

  //publish 'em
  objectPub.publish(objectMsg);
  markerPub.publish(markerMsg);
  //ROS_INFO("published markers");
}

void Recognizer::addMarker(WorldObjectPtr wo)
{
  if((*wo).getBestType().name == "unknown" && showUnknownLabels)
  {
    return;
  }

  //ROS_INFO_STREAM("Setting marker number " << (*wo).getID() << "to ADD");

  tf::Pose originalPose;
  tf::poseEigenToTF (wo->getBestPose(), originalPose);
  //objectBroadcaster->sendTransform(tf::StampedTransform(originalPose, ros::Time::now(), recognitionFrame, wo->getBestType().name));
  //ROS_INFO("Recognizer marker pos: %f %f %f", originalPose.getOrigin().x(), originalPose.getOrigin().y(),  originalPose.getOrigin().z());
  //ROS_INFO("objPose quaternion: %f %f %f %f", rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
  
  ros::Time now = ros::Time::now();

  visualization_msgs::Marker objMarker = visualization_msgs::Marker(getStubAt((*wo).getBestType().name).first);
  objMarker.header.stamp       = now;
  objMarker.id                 = (*wo).getID()+100000;
  tf::poseTFToMsg(originalPose, objMarker.pose);
  objMarker.color.a = markerAlpha*0.5;
  objMarker.header.frame_id = "world"; //FIXME

  visualization_msgs::Marker labelMarker;
  labelMarker.header.frame_id    = "world"; //FIXME
  labelMarker.header.stamp       = now;
  labelMarker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
  labelMarker.action             = visualization_msgs::Marker::ADD;
  labelMarker.id                 = (*wo).getID();
  labelMarker.text = generateMarkerLabel(*wo);
  tf::poseTFToMsg(originalPose, labelMarker.pose);
  labelMarker.pose.position.z += 0.1f;
  labelMarker.scale.z = markerSize;
  labelMarker.color.r = 1.0f;
  labelMarker.color.g = 1.0f;
  labelMarker.color.b = 1.0f;
  labelMarker.color.a = 1.0f;

  markerMsg.markers.push_back(labelMarker);
  markerMsg.markers.push_back(objMarker);

} //addMarker

void Recognizer::updateMarker(WorldObjectPtr wo)
{
  ros::Time now = ros::Time(0);
  std::string woName;
  for(std::vector<visualization_msgs::Marker>::iterator it = markerMsg.markers.begin();
    it != markerMsg.markers.end(); ++it)
  {
    if(it->id == wo->getID())
    {
      woName = (*wo).getBestType().name;
      (*it).header.stamp       = now;
      (*it).id                 = (*wo).getID();

      tf::Pose intPose;
      tf::poseEigenToTF((*wo).getBestPose(), intPose);
      tf::poseTFToMsg(intPose,(*it).pose);

      if(!(showUnknownLabels && woName == "unknown"))
      {
        (*it).text = generateMarkerLabel(*wo);
      }
      else
      {
        (*it).text = "";
      }
    }
  }
} //updateMarker

const char* Recognizer::generateMarkerLabel(WorldObject& wo) {
  std::stringstream markerstream;
  std::string woName = wo.getBestType().name;
  markerstream << woName;
  if(showRecognitionProbability)
  {
    markerstream << " (" << wo.getBestProbability() << ")";
  }

  if(showPose || showPosition || showPoseStdDev)
  {
    markerstream << "\n";
  }
  if(showPosition)
  {
    markerstream << "[" << wo.getX(woName) << ", "
                        << wo.getY(woName) << ", "
                        << wo.getZ(woName) << "]";
  }
  if(showPose)
  {
    markerstream << "[P: " << wo.getPoseTf(woName).getRotation().getY() << "]";
  }
  if(showPoseStdDev)
  {
    //markerstream << "[S: " << wo.getPoseStdDev() << "]";
  }
  return markerstream.str().c_str();
} //generateMarkerLabel

void Recognizer::deleteMarker(WorldObjectPtr wo)
{
  // ROS_INFO_STREAM("Setting marker number " << wo->getID() << " to DELETE");

  bool deleted = false;
  for(std::vector<visualization_msgs::Marker>::iterator it2 = markerMsg.markers.begin();
    it2 != markerMsg.markers.end(); ++it2)
  {
    if(it2->id == wo->getID() || it2->id == wo->getID() + 100000)
    {
      it2->action = visualization_msgs::Marker::DELETE;
      deleted = true;
    }
  }
  if(deleted) return;

  ros::Time now = ros::Time::now();

  visualization_msgs::Marker newDeleteMarker;
  newDeleteMarker.header.frame_id    = "world"; //FIXME
  newDeleteMarker.header.stamp       = now;

  visualization_msgs::Marker newDeleteMarker2;
  newDeleteMarker.header.frame_id    = "world"; //FIXME
  newDeleteMarker.header.stamp       = now;

  newDeleteMarker.id = wo->getID();
  newDeleteMarker2.id = wo->getID()+100000;

  newDeleteMarker.action = visualization_msgs::Marker::DELETE;
  newDeleteMarker2.action = visualization_msgs::Marker::DELETE;

  markerMsg.markers.push_back(newDeleteMarker);
  markerMsg.markers.push_back(newDeleteMarker2);

} //deleteMarker

WorldObjectPtr Recognizer::getMostLikelyObjectOfType(WorldObjectType wot)
{
  float max = 0.0;
  float prob = 0.0;
  best = WorldObjectPtr();
  if(model.size() < 1)
  {
    ROS_ERROR("Recognizer: No vision objects while trying to get most likely object of type");
    //throw std::runtime_error("No vision objects while trying to get most likely object of type");
  }
  int i = 0;
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); it++)
  {
    prob = (**it).getProbabilityOf(wot);
    std::string newName = (**it).getBestType().name;
    ROS_INFO_STREAM("chance that item " << i << " is a(n) " << wot.name.c_str() << ": " << prob << " at position "
      << (**it).getX(wot.name) << ", " << (**it).getY(wot.name) << ", " << (**it).getZ(wot.name)
      << ". Its most likely a " << newName.c_str() << " and the position is "
      << (**it).getX(newName) << ", " << (**it).getY(newName) << ", " << (**it).getZ(newName));
    if(prob > max && (**it).getX(wot.name) != 0 && (**it).getY(wot.name) != 0 && (**it).getZ(wot.name) != 0 )
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
  WorldObjectType type = typeManager->getUnknownType();
  try {
    typeManager->getTypeByName(name);
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
