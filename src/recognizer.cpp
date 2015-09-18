#include "orp/core/recognizer.h"

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "recognizer");
  ros::NodeHandle n;

  if(argc < 2) {
    ROS_FATAL("proper usage is 'recognizer sensor_model_file");
    return -1;
  }
  std::string listFile = argv[1];
  ROS_INFO("Starting Recognizer");
  Recognizer s(n, listFile);

  ros::spin();
  return 1;
} //main

///////////////////////////////////////////////////////////////////////////////////

Recognizer::Recognizer(ros::NodeHandle nh, std::string sensorModelFile) :
    n(nh),
    colocationDist(0.01),
    typeManager(0),
    recognitionFrame("camera_depth_optical_frame"),

    markerTopic("/detected_object_markers"),
    objectTopic("/detected_objects"),
    markerMsg(),
    markerRed(1.0),
    markerGreen(1.0),
    markerBlue(0.0),
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
    50,
    &Recognizer::cb_detectionSet,
    this);

  objectBroadcaster = new tf::TransformBroadcaster();
  objectTransformer = new tf::Transformer();
  transformListener = new tf::TransformListener();

  typeManager = new WorldObjectManager("unknown");

  ROS_INFO("initializing sensor model");
  initializeBayesSensorModel(sensorModelFile);

  //startRecognition();

  fillMarkerStubs();
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

  markerRed   = config.marker_red;
  markerGreen = config.marker_green;
  markerBlue  = config.marker_blue;
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
        rpy.roll = ORPUtils::radFromDeg(rpy.roll);
        rpy.pitch = ORPUtils::radFromDeg(rpy.pitch);
        rpy.yaw = ORPUtils::radFromDeg(rpy.yaw);
      } catch(std::exception e) {
        ROS_ERROR("error while creating marker stub for world object of type '%s': %s", e.what(), (*names).c_str());
        stub.scale.x = 0.1;
        stub.scale.y = 0.1;
        stub.scale.z = 0.1;

        stub.type = visualization_msgs::Marker::CUBE;

        rpy.roll = 0.0;
        rpy.pitch = 0.0;
        rpy.yaw = 0.0;
      }
    }
    //ROS_INFO("creating marker stub for world object of type '%s'", (*names).c_str());
    stub.header.frame_id = recognitionFrame;
    stub.action          = visualization_msgs::Marker::ADD;
    markerStubs[*names] = std::make_pair<visualization_msgs::Marker, RPY>(stub, rpy);
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


void Recognizer::initializeBayesSensorModel(std::string path)
{
  //parse
  std::ifstream objectListFile;
  std::string tempString, eachLine, label;

  int rowCount = 0, columnCount = 0;

  try {
    objectListFile.open(path.c_str(), std::ios::in);
    std::getline(objectListFile, eachLine);
    std::istringstream nameSS(eachLine);
    while(nameSS >> eachLine) {
      typeList.push_back(eachLine);
      WorldObjectType thisType = WorldObjectType(eachLine);
      typeManager->addType(thisType);
      columnCount++;
    }

    ROS_INFO("Done reading list of objects (%i). Now reading each row", columnCount);
    //continue onwarda
    while(std::getline(objectListFile, eachLine))
    {
      if(eachLine.at(0) == '#') continue;

      //ROS_INFO("row: %s", eachLine.c_str());
      FloatLookupRow sensorModelRow;

      std::istringstream objectSS(eachLine);
      //1. Record object name and add to map.
      objectSS >> label;
      //ROS_INFO("label: %s", label.c_str());

      //FIXME: rotation standard deviation
      objectSS >> tempString;  //dummy
      //ROS_INFO("pose std dev: %s", tempString.c_str());
      //FIXME FIXME 
      //typeManager->getTypeByName(label).rotStdDev = atof(tempString.c_str());

      //2. Get the row of the confusion matrix
      int i = 0;
      while(objectSS >> tempString)
      {
        if(i >= columnCount)
        {
          ROS_ERROR("There are too many columns on the line %i, starting with %s.", i, label.c_str());
        }
        else
        {
          //ROS_INFO("inserting %f as probability for %s/%s (i=%d)", atof(tempString.c_str()), label.c_str(), typeList[i].c_str(), i);
          sensorModelRow.insert(std::pair<std::string,float>(typeList[i], atof(tempString.c_str())));
          rowCount++;
        }
        i++;
      }
      //ROS_INFO("%i rows", rowCount);
      fullSensorModel.insert(std::make_pair<std::string, FloatLookupRow>(label, sensorModelRow));
    }
    //ROS_INFO("%i columns", columnCount);
    subSensorModel = fullSensorModel;
    ROS_INFO("done reading sensor file");

    objectListFile.close();
  }
  catch(const std::ifstream::failure& e)
  {
    ROS_ERROR_STREAM("Exception opening/reading sensor file " << path.c_str() << ": " << e.what());
    ROS_ERROR("Bayesian sensor model not loaded.");
  }
  catch(const std::logic_error& e)
  {
    ROS_ERROR("Exception creating type list and sensor model: %s", e.what());
    ROS_ERROR("Bayesian sensor model not loaded.");
  }

} //initializeSensorModel

void Recognizer::cb_classificationResult(orp::ClassificationResult newObject)
{
  //ROS_INFO("object incoming, type %s...", newObject.result.label.c_str());
  tf::Stamped<tf::Pose> objPose;
  tf::poseStampedMsgToTF(newObject.result.pose, objPose);

  RPY rpy;

  //ROS_INFO("objPose: %f %f %f", objPose.getOrigin().x(),objPose.getOrigin().y(),objPose.getOrigin().z());
  //ROS_INFO_STREAM("objPose: " << objPose.getRotation().);

  TypeMap probs;
  tf::Pose stubAdjustmentPose, adjustedObjPose;
  Eigen::Affine3d eigPose;
  //eigPose = Eigen::Affine3d();
  for(FloatLookupTable::iterator it = subSensorModel.begin(); it != subSensorModel.end(); it++) {
    stubAdjustmentPose = tf::Pose(tf::createQuaternionFromRPY(
      getStubAt(it->first).second.roll,
      getStubAt(it->first).second.pitch,
      getStubAt(it->first).second.yaw
    ));
     //ROS_INFO("stub adjustment for %s is %f %f %f", it->first.c_str(), getStubAt(it->first).second.roll,
     //  getStubAt(it->first).second.pitch,
     //  getStubAt(it->first).second.yaw);
    tf::poseTFToEigen(objPose, eigPose);
    Eigen::Matrix3d rotMat = eigPose.linear();
    Eigen::Quaterniond rotQ; rotQ = rotMat;
    //ROS_INFO("objPose quaternion: %f %f %f %f", rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());

    adjustedObjPose = stubAdjustmentPose;
   
    tf::poseTFToEigen(adjustedObjPose, eigPose);

    rotMat = eigPose.linear();
    rotQ = rotMat;
    //ROS_INFO("output pose quaternion: %f %f %f %f", rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
    //ROS_INFO("chance that it's %s is %f", it->first.c_str(), it->second.at(newObject.result.label));
    probs.insert(TypeMap::value_type(it->first, PoseGuess(it->second.at(newObject.result.label), eigPose)));
  }
  //ROS_INFO_STREAM("Recognizer result pose: " << newObject.result.pose.pose.position.x << ", " << newObject.result.pose.pose.position.y <<
  //  ", " << newObject.result.pose.pose.position.z);
  
  // for(TypeMap::iterator it = probs.begin(); it != probs.end(); ++it) {
  //   ROS_INFO_STREAM("actual probs value: " << it->first.name.c_str() << ": " << it->second.prob << "; " << it->second.pose.translation()(0) << ", " << it->second.pose.translation()(3) << ", " << it->second.pose.translation()(3));
  // }
  //ROS_INFO("%lf %lf %lf", getStubAt(((*wo).getBestType().name)).second.roll, getStubAt(((*wo).getBestType().name)).second.pitch, getStubAt(((*wo).getBestType().name)).second.yaw);
  // for(TypeMap::iterator it = probs.begin(); it != probs.end(); ++it) {
  //   Eigen::Matrix3d rotMat = it->second.pose.linear();
  //   Eigen::Quaterniond rotQ; rotQ = rotMat;
  //   ROS_INFO("PreProbs: pose for %s has quat %f, %f, %f, %f", it->first.name.c_str(), rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
  // }

  if(newObject.method == "cph")
  {
    WorldObjectPtr p = WorldObjectPtr(
      new WorldObjectBayesKalman(subSensorModel, 
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
      new WorldObjectBayesKalman(subSensorModel, 
                                  colocationDist,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    addMarker(p);
  }
  else if(newObject.method == "cvfh")
  {
    WorldObjectPtr p = WorldObjectPtr(
      new WorldObjectBayesKalman(subSensorModel, 
                                  colocationDist,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    addMarker(p);
  }
  else if(newObject.method == "icp")
  {
    WorldObjectPtr p = WorldObjectPtr(
      new WorldObjectBayesKalman(subSensorModel, 
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
      new WorldObjectBayes(subSensorModel, 
                                  colocationDist*2,
                                  typeManager,
                                  newObject.result,
                                  probs));
    model.push_back(p);
    //ROS_INFO("the new object has type %s", p->getBestType().name.c_str());
    Eigen::Affine3d eigPose = p->getBestPose();
    //ROS_INFO("recognizer: new object has position %f, %f, %f", eigPose(0,3), eigPose(1,3), eigPose(3,3));
    //Eigen::Matrix3d rotMat = eigPose.linear();
    //Eigen::Quaterniond rotQ; rotQ = rotMat;
    addMarker(p);
  }
  else
  {
    ROS_ERROR("unknown classification method %s", newObject.method.c_str());
  }
  //clean up (DELETEs markers for stale objects)
  //ROS_INFO("killing stale...");
  killStale();
} //cb_classificationResult

void Recognizer::startRecognition() {
  if(timer == NULL || recognitionSub == NULL)
  {
    ROS_INFO("Starting Visual Recognition");
    recognitionSub = n.subscribe(
      "/classification",
      50,
      &Recognizer::cb_classificationResult,
      this);

    timer = n.createTimer(ros::Duration(refreshInterval), boost::bind(&Recognizer::recognize, this, _1));
    timer.start();
  } else {
    ROS_ERROR("attempted to start recognition, but already started");
  }
} //startRecognition

void Recognizer::stopRecognition() {
  if(timer != NULL && recognitionSub != NULL)
  {
    ROS_INFO("Stopping Visual Recognition");
    timer.stop();
    recognitionSub.shutdown();
    for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it) {
      deleteMarker(*it);
    }
    //say goodbye (send delete markers)
    publishROS();
    //clear delete markers
    refreshMarkers();
    //clear all markers
    model.clear();
  } else {
    ROS_ERROR("attempted to stop recognition, but it hasn't started or the timer doesn't exist.");
  }
} //stopRecognition

void Recognizer::setDetectionSet(std::vector<std::string> set) {
  if(set.size() > fullSensorModel.size()) {
    ROS_ERROR("your desired subset is larger than the number of known items. Something's wrong.");
  }
  subSensorModel = fullSensorModel; // begin with them equal
  for(FloatLookupTable::iterator source = fullSensorModel.begin(); source != fullSensorModel.end(); ++source) {
    if(std::find(set.begin(), set.end(), source->first) == set.end() && source->first != "unknown") {
      //this row (object type) isn't found in the desired subset.
      //first, remove its row from the current subset sensor model;
      subSensorModel.erase(source->first);
      //now, loop through the subset model and remove this object's column (probability) from each row.
      for(FloatLookupTable::iterator row = subSensorModel.begin(); row != subSensorModel.end(); ++row) {
        row->second.erase(source->first);
      }
    }
  }
  if(subSensorModel.empty()) { //nothing left!
    ROS_FATAL("No items left in Recognizer detection subset!");
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
  filter();         //set some markers to DELETE
  publishROS();     //publish all markers
  refreshMarkers(); //get rid of DELETE markers 

  if(shouldDebugPrint) debugPrint();     //list markers 
} //recognize


void Recognizer::filter()
{
  //remove objects that are spatial duplicates
  //ROS_INFO("filtering model of length %i", (int) model.size());
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); ++it)
  {
    for(WorldObjectList::iterator it2 = model.begin(); it2 != model.end(); )
    {
      //ROS_INFO("dist: %f", tf::tfDistance((*it)->getPoseTf().getOrigin(), (*it2)->getPoseTf().getOrigin()));
      if(it != it2)
      {
        // ROS_INFO("merging objects...");
        (**it).mergeIn(&(**it2));
        ++it2;
      }
      else
      {
        ++it2;
      }
    }
  }
} //filter

void Recognizer::killStale()
{
  ros::Time now = ros::Time::now();
  for(WorldObjectList::iterator it = model.begin(); it != model.end(); )
  {
    if((**it).isStale() || now - (**it).getLastUpdated() > staleTime)
    {
      //ROS_INFO("killing stale world object %i", (**it).getID());
      deleteMarker((*it));
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
  if(model.size() == 0)
  {
    return;
  }

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

    newObject.pose.header.frame_id = recognitionFrame;
    objectMsg.objects.insert(objectMsg.objects.end(), newObject);
  }

  //publish 'em
  objectPub.publish(objectMsg);
  markerPub.publish(markerMsg);
}

void Recognizer::refreshMarkers()
{
  for(std::vector<visualization_msgs::Marker>::iterator it = markerMsg.markers.begin();
    it != markerMsg.markers.end(); )
  { //delete old DELETE markers
    if((*it).action == visualization_msgs::Marker::DELETE)
    {
      it = markerMsg.markers.erase(it);
    }
    else
    {
      ++it;
    }
  }
} //refreshMarkers

void Recognizer::addMarker(WorldObjectPtr wo)
{
  if((*wo).getBestType().name == "unknown" && showUnknownLabels)
  {
    return;
  }

  tf::Pose originalPose;
  tf::poseEigenToTF (wo->getBestPose(), originalPose);
  //ROS_INFO("Recognizer marker pos: %f %f %f", originalPose.getOrigin().x(), originalPose.getOrigin().y(),  originalPose.getOrigin().z());
  //ROS_INFO("objPose quaternion: %f %f %f %f", rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
  
  ros::Time now = ros::Time::now();

  visualization_msgs::Marker labelMarker;
  labelMarker.header.frame_id    = recognitionFrame;
  labelMarker.header.stamp       = now;
  labelMarker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
  labelMarker.action             = visualization_msgs::Marker::ADD;
  //labelMarker.lifetime           = refreshInterval;
  labelMarker.id                 = (*wo).getID();
  labelMarker.text = generateMarkerLabel(*wo);
  tf::poseTFToMsg(originalPose, labelMarker.pose);
  labelMarker.scale.z = markerSize;
  labelMarker.color.r = markerRed;
  labelMarker.color.g = markerGreen;
  labelMarker.color.b = markerBlue;
  labelMarker.color.a = markerAlpha;

  visualization_msgs::Marker objMarker = visualization_msgs::Marker(getStubAt((*wo).getBestType().name).first);
  objMarker.header.stamp       = now;
  //objMarker.lifetime           = ros::Duration(0.2);
  objMarker.id                 = (*wo).getID()+10000;

  transformListener->waitForTransform("shelf", "base_link", ros::Time(0), ros::Duration(0.1));

  tf::poseTFToMsg(originalPose, objMarker.pose);
  objMarker.color.r = markerRed;
  objMarker.color.g = markerGreen;
  objMarker.color.b = markerBlue;
  objMarker.color.a = markerAlpha*0.5;

  tf::Transform transform = originalPose;
  //objectBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), recognitionFrame, wo->getBestType().name));

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
  for(std::vector<visualization_msgs::Marker>::iterator it2 = markerMsg.markers.begin();
    it2 != markerMsg.markers.end(); ++it2)
  {
    if(it2->id == wo->getID() || it2->id == wo->getID() + 10000)
    {
      it2->action = visualization_msgs::Marker::DELETE;
    }
  }
} //deleteMarker

WorldObjectPtr Recognizer::getMostLikelyObjectOfType(WorldObjectType wot)
{
  float max = 0.0;
  float prob = 0.0;
  int maxStr = 0;
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
     ROS_INFO("chance that item %i a %s: %f at position %f %f %f with strength %d. Its most likely a %s and the position is %f %f %f",
      i, wot.name.c_str(), prob, 
      (**it).getX(wot.name), (**it).getY(wot.name), (**it).getZ(wot.name),
      (**it).getStrength(),
      newName.c_str(), (**it).getX(newName), (**it).getY(newName), (**it).getZ(newName) );
    if(prob > max && (**it).getX(wot.name) != 0 && (**it).getY(wot.name) != 0 && (**it).getZ(wot.name) != 0 && (**it).getStrength() > maxStr)
    {
      max = prob;
      best = *it;
      maxStr = (**it).getStrength();
    }
    ++i;
  }
  return best;
}; //getMostLikelyObjectOfType

WorldObjectPtr Recognizer::getMostLikelyObjectOfType(std::string name)
{
  return getMostLikelyObjectOfType(typeManager->getTypeByName(name));
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
