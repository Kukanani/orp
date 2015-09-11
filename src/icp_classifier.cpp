#include "orp/classifier/icp_classifier.h"

int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "icp_classifier");
  ros::NodeHandle n;

  if(argc < 3) {
    ROS_FATAL("proper usage is 'icp_classifier data_directory object_list_file");
    return -1;
  }
  std::string directory = argv[1];
  std::string listFile = argv[2];

  ROS_INFO("Starting ICP Classifier");
  ICPClassifier v(n, directory, listFile);
  v.init();

  ros::spin();
  return 1;
} //main

ICPClassifier::ICPClassifier(ros::NodeHandle nh, std::string folder, std::string filepath) :
  n(nh),
  dataFolder(folder),
  name("icp"),
  fileExtension(".pcd"),
  path(filepath)
{

} //ICPClassifier

ICPClassifier::ICPClassifier(const ICPClassifier& other) {
  n = other.n;
  name = other.name;
  dataFolder = other.dataFolder;
  //may have to add other copies here 
}

void ICPClassifier::init() {
  ROS_INFO("%s: Reading list file %s", name.c_str(), path.c_str());

  FloatLookupTable table;
  //parse
  std::ifstream objectListFile;
  std::string tempString, eachLine, label;

  objectListFile.open(path.c_str(), std::ios::in);
  if(objectListFile.fail()) {
    ROS_ERROR("could not open list file");
  }
  std::getline(objectListFile, eachLine);
  std::istringstream nameSS(eachLine);
  ROS_INFO("items:");
  while(nameSS >> eachLine)
  {
    ROS_INFO("\t%s", eachLine.c_str());
    typeList.push_back(eachLine);
  }

  objectListFile.close();

  loadModelsRecursive(dataFolder, fileExtension);
  ROS_INFO("%s: Loaded %d models.\n", name.c_str(), (int)knownObjects.size());

  if(knownObjects.size() < 1)
  {
    ROS_FATAL ("%s: No models loaded.", name.c_str());
    return;
  }

  subscribe();
} //ICPClassifier

ICPClassifier::~ICPClassifier() {
} //~ICPClassifier

void ICPClassifier::subscribe()
{
  if(depthInfoSubscriber == NULL)
  {
    depthInfoSubscriber = n.subscribe(
      "/camera/depth_registered/points", 1, &ICPClassifier::cb_classify, this);
    classificationPub = n.advertise<orp::ClassificationResult>("classification", 1);
  }
  if(segmentationClient == NULL)
  {
    segmentationClient = n.serviceClient<orp::Segmentation>("/segmentation");
  }
} //subscribe

void ICPClassifier::cb_classify(sensor_msgs::PointCloud2 cloudMsg) {

  ROS_INFO("camera callback");
  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloudMsg;
  //ROS_INFO("CVFH classifier calling segmentation");
  segmentationClient.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;
  //ROS_INFO("CVFH classifier finished calling segmentation");

  orp::ClassificationResult classRes;

  for(std::vector<sensor_msgs::PointCloud2>::iterator eachCloud = clouds.begin(); eachCloud != clouds.end(); eachCloud++) {

    sensor_msgs::PointCloud2 transformedMessage;
    std::string transformToFrame = "/shelf_vision";
    pcl::PointCloud<ORPPoint>::Ptr thisCluster (new pcl::PointCloud<ORPPoint>);

    if(transformListener.waitForTransform(eachCloud->header.frame_id, transformToFrame,
      ros::Time::now(), ros::Duration(0.5)))
    { 
      pcl_ros::transformPointCloud (transformToFrame, *eachCloud, transformedMessage, transformListener);
      pcl::fromROSMsg(transformedMessage, *thisCluster);
    }
    else {
      ROS_ERROR("listen for transformation to %s timed out. Proceeding...", transformToFrame.c_str());
      pcl::fromROSMsg(*eachCloud, *thisCluster);
    }

    pcl::IterativeClosestPoint<ORPPoint, ORPPoint> icp;
    icp.setInputTarget(thisCluster);

    int iterations = 100;
    float maxDistance = 1e300;
    float bestDistance = maxDistance;
    std::string bestName = "unknown";

    for(std::vector<ICPModel>::iterator knownObject = knownObjects.begin(); knownObject != knownObjects.end(); ++knownObject) {
      icp.setInputSource(knownObject->second);
      ROS_INFO("performing ICP...");
      pcl::PointCloud<ORPPoint>::Ptr cloud_final (new pcl::PointCloud<ORPPoint>);
      icp.setMaximumIterations(iterations);
      icp.align(*cloud_final);
      if(icp.hasConverged()) {
        double fitness = icp.getFitnessScore();
        ROS_INFO("converged with distance %f", fitness);
        if(fitness < bestDistance) {
          bestDistance = fitness;
          bestName = knownObject->first;
        }
      }
    }

    classRes.result.pose.header.frame_id = "camera_depth_optical_frame";
    classRes.result.pose.pose.position.x = (icp.getFinalTransformation())(0,3);
    classRes.result.pose.pose.position.y = (icp.getFinalTransformation())(1,3);
    classRes.result.pose.pose.position.z = (icp.getFinalTransformation())(2,3);
    classRes.method = "icp";
    
    if(bestDistance < maxDistance) {
      //IF FOUND ONE BETTER THAN MAX DISTANCE

      classRes.result.pose.header.frame_id = "camera_depth_optical_frame";
      classRes.result.pose.pose.position.x = (icp.getFinalTransformation())(0,3);
      classRes.result.pose.pose.position.y = (icp.getFinalTransformation())(1,3);
      classRes.result.pose.pose.position.z = (icp.getFinalTransformation())(2,3);
      classRes.method = "icp";

      classRes.result.label = bestName;

      //Eigen::AngleAxisd yawAngle = Eigen::AngleAxisd( ORPUtils::radFromDeg(loadedModels.at(kIndices[0][0]).first.angle), Eigen::Vector3d::UnitY());
      //Eigen::Quaternion<double> q(yawAngle);
      //
      Eigen::Matrix4f transformation = icp.getFinalTransformation();
      Eigen::Transform<float, 3, Eigen::Affine> transform;
      transform = Eigen::Transform<float, 3, Eigen::Affine>(transformation);
      Eigen::Quaternionf q1;
      q1 = Eigen::Quaternionf(transform.rotation());
      Eigen::Quaterniond q  = q1.cast<double>();

      tf::quaternionEigenToMsg(q, classRes.result.pose.pose.orientation);
    }  else { 
      ROS_INFO("couldn't match using ICP");
      classRes.result.label = "unknown";
    }
    ROS_INFO("publishing classification result.");
    classificationPub.publish(classRes);
  }
} //classify

void ICPClassifier::unsubscribe()
{
  depthInfoSubscriber.shutdown();
  classificationPub.shutdown();
  segmentationClient.shutdown();
} //unsubscribe

void ICPClassifier::loadModelsRecursive(
  const boost::filesystem::path &base_dir,
  const std::string &extension)
{
  if(!boost::filesystem::is_directory (base_dir)) {
    ROS_FATAL("%s: target path %s is not a directory!", name.c_str(), base_dir.string().c_str());
    return;
  }
  if(!boost::filesystem::exists (base_dir)) {
    ROS_FATAL("%s: target folder %s does not exist", name.c_str(), base_dir.string().c_str());
    return;
  }

  for(boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it) {
    if(boost::filesystem::is_directory(it->status ())) {
      std::stringstream ss;
      ss << it->path();
      //ROS_INFO("%s: NOT traversing into directory %s.", ss.str().c_str(), (unsigned long)loadedModels.size());
      loadModelsRecursive (it->path(), extension);
    }
    if(boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == extension) {
      pcl::PointCloud<ORPPoint>::Ptr cloud_out;
      boost::regex pattern("");
      boost::cmatch what;
      ROS_INFO("checking file %s", it->path().filename().string().c_str());
      for(std::vector<std::string>::iterator types = typeList.begin(); types != typeList.end(); ++types) {
        pattern = boost::regex("(" + *types + ")(.*)");
        if(boost::regex_match(it->path().filename().string().c_str(), what, pattern)) {
          cloud_out = ORPUtils::loadCloudFrom(it->path().string().c_str());

          std::string name = it->path().filename().string();
          name.erase(name.size() - 4);
          ROS_INFO("ICP loaded %s", name.c_str());
          knownObjects.push_back(std::make_pair<std::string, pcl::PointCloud<ORPPoint>::Ptr>(name, cloud_out));
          ++types; //only find one object of each name
        }
      }
    }
  }
} //loadModelsRecursive