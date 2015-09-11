#include "orp/classifier/vfh_classifier.h"

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "vfh_classifier");
  ros::NodeHandle n;

  if(argc < 3) {
    ROS_FATAL("proper usage is 'vfh_classifier data_directory object_list_file");
    return -1;
  }
  std::string directory = argv[1];
  std::string listFile = argv[2];

  ROS_INFO("Starting VFH Classifier");
  VFHClassifier v = VFHClassifier(n, directory, listFile);
  v.init();

  ros::spin();
  return 1;
} //main

VFHClassifier::VFHClassifier(ros::NodeHandle nh, std::string dataFolder, std::string path):
  Classifier(nh, 10000, "vfh", path, dataFolder, ".vfh")
{

} //VFHClassifier

bool VFHClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &vfh) {
  //ROS_INFO("Loading histogram %s", path.string().c_str());
  //path is the location of the file being read.
  int vfh_idx;
  // Load the file as a PCD
  try {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type;   unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return false;
    if ((int)cloud.width * cloud.height != 1)
      return false;
  }
  catch (pcl::InvalidConversionException e)
  {
    return false;
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  for (size_t i = 0; i < vfh.second.size(); ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }

  KnownPose kp;

  std::string cloud_name = path.filename().string();
  cloud_name.erase(cloud_name.end()-3, cloud_name.end());
  kp.name.assign(cloud_name.begin()+cloud_name.rfind("/")+1, cloud_name.begin()+cloud_name.rfind("_"));

  std::string angleStr;
  angleStr.assign(cloud_name.begin()+cloud_name.rfind("_")+1, cloud_name.end());
  kp.angle = atoi(angleStr.c_str());

  vfh.first = kp;
  return true;
} //loadHist

void VFHClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {

  orp::ClassificationResult classRes;
  pcl::PointCloud<ORPPoint>::Ptr thisCluster (new pcl::PointCloud<ORPPoint>);
  pcl::fromROSMsg(cloud, *thisCluster);

  //Get the centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*thisCluster, centroid);

  //Compute vfh:
  pcl::VFHEstimation<ORPPoint, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (thisCluster);
  //Estimate normals:
  pcl::NormalEstimation<ORPPoint, pcl::Normal> ne;
  ne.setInputCloud (thisCluster);
  pcl::search::KdTree<ORPPoint>::Ptr treeNorm (new pcl::search::KdTree<ORPPoint> ());
  ne.setSearchMethod (treeNorm);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);

  //VFH estimation
  vfh.setInputNormals (cloud_normals);
  pcl::search::KdTree<ORPPoint>::Ptr tree (new pcl::search::KdTree<ORPPoint> ());
  vfh.setSearchMethod (tree);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  vfh.compute (*vfhs);

  //Nearest neighbor algorigthm
  FeatureVector histogram;
  histogram.second.resize(308);

  for (size_t i = 0; i < 308; ++i) {
    histogram.second[i] = vfhs->points[0].histogram[i];
  }
  //KNN classification find nearest neighbors based on histogram
  nearestKSearch (*kIndex, histogram, 5, kIndices, kDistances);


  //ROS_INFO("VFH: %i rows, %i columns returned from nearest K search.", kIndices.rows, kIndices.cols);
  for(int j=0; j<kIndices.cols; j++) {
    //ROS_INFO("VFH: Dist(%s@%.2f): %f", loadedModels.at(kIndices[0][j]).first.name.c_str(),
      //loadedModels.at(kIndices[0][j]).first.angle, kDistances[0][j]);
  }

  classRes.result.pose.pose.position.x = centroid(0);
  classRes.result.pose.pose.position.y = centroid(1);
  classRes.result.pose.pose.position.z = centroid(2);
  classRes.method = "vfh";

  //make sure we're within our threshold
  if(kDistances[0][0] < threshold){
    classRes.result.label = loadedModels.at(kIndices[0][0]).first.name;

    Eigen::AngleAxisd yawAngle = Eigen::AngleAxisd( ORPUtils::radFromDeg(loadedModels.at(kIndices[0][0]).first.angle), Eigen::Vector3d::UnitY());
    Eigen::Quaternion<double> q(yawAngle);

    tf::quaternionEigenToMsg(q, classRes.result.pose.pose.orientation);
  }
  else { //if nothing matches well enough, return "unknown"
    ROS_ERROR("not within threshold, distance was %f", kDistances[0][0]);
    classRes.result.label = "unknown";
  }
  classificationPub.publish(classRes);
} //classify