///////////////////////////////////////////////////////////////////////////////
//      Title     : CPH Classifier
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

#include "classifier/cph_classifier.h"

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "cph_classifier");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if(argc < 3) {
    ROS_FATAL("proper usage is 'cph_classifier data_directory object_list_file");
    return -1;
  }
  std::string directory = argv[1];
  std::string listFile = argv[2];
  bool autostart = false;
  if(argc >= 4) {
    if(std::string(argv[3])  == "true") autostart = true;
  }

  ROS_INFO("Starting CPH Classifier");
  CPHClassifier c = CPHClassifier(directory, autostart);
  c.init();

  ros::spin();
  return 1;
} //main

CPHClassifier::CPHClassifier(std::string dataFolder, bool _autostart):
  Classifier(10000, "cph", dataFolder, ".cph", _autostart),
  cph(5,72)
{
  yBins = 5;
  rBins = 72;
  cphSize = yBins*rBins+3;
} //CPHClassifier

bool CPHClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &cph) {
  //ROS_INFO("Loading histogram %s", path.string().c_str());
  //path is the location of the file being read.
  std::ifstream featureFile;
  featureFile.open(path.string().c_str(), std::ifstream::in);
  if(!featureFile.is_open()) {
    ROS_ERROR("Error opening histogram file %s", path.string().c_str());
    return false;
  }

  cph.second.clear();
  float value;
  for(unsigned int i=0; i<cphSize; i++){
    featureFile >> value;
    cph.second.push_back(value); 
  }

  KnownPose kp;

  std::string cloud_name = path.filename().string();
  cloud_name.erase(cloud_name.end()-3, cloud_name.end());
  kp.name.assign(cloud_name.begin()+cloud_name.rfind("/")+1, cloud_name.begin()+cloud_name.rfind("_"));
  
  std::string angleStr;
  angleStr.assign(cloud_name.begin()+cloud_name.rfind("_")+1, cloud_name.end());
  kp.angle = atoi(angleStr.c_str());

  cph.first = kp;
  return true;
} //loadHist

void CPHClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {
  //ROS_INFO_STREAM("Camera classification callback with " << cloud.width*cloud.height << " points.");
  orp::ClassificationResult classRes;

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  //ROS_INFO("SixDOF classifier calling segmentation");
  segmentationClient.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;
  //ROS_INFO("SixDOF classifier finished calling segmentation");
  //ROS_INFO("data size: %d x %d", kData->rows, kData->cols);

  for(std::vector<sensor_msgs::PointCloud2>::iterator eachCloud = clouds.begin(); eachCloud != clouds.end(); eachCloud++) {

    pcl::PointCloud<ORPPoint>::Ptr thisCluster (new pcl::PointCloud<ORPPoint>);
    pcl::fromROSMsg(*eachCloud, *thisCluster);

    //Get the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*thisCluster, centroid);

    //ROS_INFO("creating cph...");
    //Compute cph:
    cph.setInputCloud (thisCluster);
    std::vector<float> feature;
    cph.compute(feature);

    //Nearest neighbor algorigthm
    FeatureVector histogram;
    histogram.second.resize(cphSize);

    for (size_t i = 0; i < cphSize; ++i) {
      histogram.second[i] = feature[i];
    }
    //ROS_INFO("knn searching...");
    //KNN classification find nearest neighbors based on histogram
    nearestKSearch (*kIndex, histogram, 5, kIndices, kDistances);

    //ROS_INFO("CPH: %i rows, %i columns returned from nearest K search.", kIndices.rows, kIndices.cols);
      //ROS_INFO("CPH: Dist(%s@%.2f): %f", loadedModels.at(kIndices[0][0]).first.name.c_str(),
    //  loadedModels.at(kIndices[0][0]).first.angle, kDistances[0][0]);
    // for(int j=0; j<kIndices.cols; j++) {
    //   ROS_INFO("CPH: Dist(%s@%.2f): %f", loadedModels.at(kIndices[0][j]).first.name.c_str(),
    //     loadedModels.at(kIndices[0][j]).first.angle, kDistances[0][j]);
    // }

    classRes.result.pose.pose.position.x = centroid(0);
    classRes.result.pose.pose.position.y = centroid(1);
    classRes.result.pose.pose.position.z = centroid(2);
    classRes.method = "cph";

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
    classRes.result.pose.header.frame_id = eachCloud->header.frame_id;
    classificationPub.publish(classRes);
  }
} //classify
