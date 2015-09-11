#include "orp/classifier/cvfh_classifier.h"

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "cvfh_classifier");
  ros::NodeHandle n;

  if(argc < 3) {
    ROS_FATAL("proper usage is 'cvfh_classifier data_directory object_list_file");
    return -1;
  }
  std::string directory = argv[1];
  std::string listFile = argv[2];

  ROS_INFO("Starting CVFH Classifier");
  CVFHClassifier v(n, directory, listFile);
  v.init();

  ros::spin();
  return 1;
} //main

CVFHClassifier::CVFHClassifier(ros::NodeHandle nh, std::string dataFolder, std::string path):
  Classifier(nh, 20000, "cvfh", path, dataFolder, ".cvfh")
{

} //CVFHClassifier

bool CVFHClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &cvfh) {
  //ROS_INFO("Loading histogram %s", path.string().c_str());
  //path is the location of the file being read.
  int cvfh_idx;
  // Load the file as a PCD
  try {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type;   unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    //cvfh_idx = pcl::getFieldIndex (cloud, "cvfh");
    //if (cvfh_idx == -1)
      //return false;
    //if ((int)cloud.width * cloud.height != 1)
      //return false;
  }
  catch (pcl::InvalidConversionException e)
  {
    return false;
  }

  // Treat the CVFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile(path.string(), point);
  cvfh.second.resize(308);

  for (size_t i = 0; i < cvfh.second.size(); ++i)
  {
    cvfh.second[i] = point.points[0].histogram[i];
  }

  KnownPose kp;

  std::string cloud_name = path.filename().string();
  cloud_name.erase(cloud_name.end()-3, cloud_name.end());
  kp.name.assign(cloud_name.begin()+cloud_name.rfind("/")+1, cloud_name.begin()+cloud_name.rfind("_"));

  std::string angleStr;
  angleStr.assign(cloud_name.begin()+cloud_name.rfind("_")+1, cloud_name.end());
  kp.angle = atoi(angleStr.c_str());

  cvfh.first = kp;
  return true;
} //loadHist

void CVFHClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {
  //ROS_INFO("camera callback");
  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  //ROS_INFO("CVFH classifier calling segmentation");
  segmentationClient.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;
  //ROS_INFO("CVFH classifier finished calling segmentation");

  orp::ClassificationResult classRes;

  //ROS_INFO("data size: %d x %d", kData->rows, kData->cols);

  for(std::vector<sensor_msgs::PointCloud2>::iterator eachCloud = clouds.begin(); eachCloud != clouds.end(); eachCloud++) {

    sensor_msgs::PointCloud2 transformedMessage;
    std::string transformToFrame = "/shelf_vision";
    pcl::PointCloud<ORPPoint>::Ptr thisCluster (new pcl::PointCloud<ORPPoint>);
    //ROS_INFO("CVFH: going from %s to %s", eachCloud->header.frame_id.c_str(), transformToFrame.c_str());


    if(listener.waitForTransform(eachCloud->header.frame_id, transformToFrame,
      ros::Time::now(), ros::Duration(0.5)))
    { 
      pcl_ros::transformPointCloud (transformToFrame, *eachCloud, transformedMessage, listener);
      pcl::fromROSMsg(transformedMessage, *thisCluster);
    }
    else {
      ROS_ERROR("listen for transformation to %s timed out. Proceeding...", transformToFrame.c_str());
      pcl::fromROSMsg(*eachCloud, *thisCluster);
    }

    //Get the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*thisCluster, centroid);

    //Compute cvfh:
    pcl::CVFHEstimation<ORPPoint, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud (thisCluster);
    //Estimate normals:
    pcl::NormalEstimation<ORPPoint, pcl::Normal> ne;
    ne.setInputCloud (thisCluster);
    pcl::search::KdTree<ORPPoint>::Ptr treeNorm (new pcl::search::KdTree<ORPPoint> ());
    ne.setSearchMethod (treeNorm);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    //CVFH estimation
    cvfh.setInputNormals(cloud_normals);
    cvfh.setSearchMethod(treeNorm);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhs(new pcl::PointCloud<pcl::VFHSignature308> ());
    
    // Set the maximum allowable deviation of the normals,
    // for the region segmentation step.
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    // Set the curvature threshold (maximum disparity between curvatures),
    // for the region segmentation step.
    cvfh.setCurvatureThreshold(1.0);
    // Set to true to normalize the bins of the resulting histogram,
    // using the total number of points. Note: enabling it will make CVFH
    // invariant to scale just like VFH, but the authors encourage the opposite.
    cvfh.setNormalizeBins(false);

    cvfh.compute(*cvfhs);

    //Nearest neighbor algorigthm
    FeatureVector histogram;
    histogram.second.resize(308);

    for (size_t i = 0; i < 308; ++i) {
      histogram.second[i] = cvfhs->points[0].histogram[i];
    }

    int numNeighbors = 5;
    //KNN classification find nearest neighbors based on histogram
    int numFound = nearestKSearch (*kIndex, histogram, numNeighbors, kIndices, kDistances);


    int limit = std::min<int>(numNeighbors, numFound);
    //ROS_INFO("CVFH: %i rows, %i columns returned from nearest K search.", kIndices.rows, kIndices.cols);
    
    for(int j=0; j<limit; j++) {
      //ROS_INFO("CVFH: Dist(%s@%.2f): %f", subModels.at(kIndices[0][j]).first.name.c_str(),
      //  subModels.at(kIndices[0][j]).first.angle, kDistances[0][j]);
      ROS_DEBUG("CVFH: Dist(%s@%.2f): %f", subModels.at(kIndices[0][j]).first.name.c_str(),
        subModels.at(kIndices[0][j]).first.angle, kDistances[0][j]);
    }

    classRes.result.pose.pose.position.x = centroid(0);
    classRes.result.pose.pose.position.y = centroid(1);
    classRes.result.pose.pose.position.z = centroid(2);

    //ROS_INFO("CVFH: centroid is at %f %f %f", centroid(0), centroid(1), centroid(2));

    classRes.method = "cvfh";

    double angle;
    //make sure we're within our threshold
    if(kDistances[0][0] < threshold){
      classRes.result.label = subModels.at(kIndices[0][0]).first.name;

      angle = subModels.at(kIndices[0][0]).first.angle;
    }
    else { //if nothing matches well enough, return "unknown"
      ROS_ERROR("not within threshold, distance was %f", kDistances[0][0]);
      classRes.result.label = "unknown";
      angle = 0;
    }

    Eigen::AngleAxisd yawAngle = Eigen::AngleAxisd( ORPUtils::radFromDeg(angle), Eigen::Vector3d::UnitY());
    Eigen::Quaternion<double> q(yawAngle);
    tf::quaternionEigenToMsg(q, classRes.result.pose.pose.orientation);
    
    classificationPub.publish(classRes);

    delete[] kIndices.ptr();
    delete[] kDistances.ptr();
  }
} //classify*/