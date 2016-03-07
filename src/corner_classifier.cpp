///////////////////////////////////////////////////////////////////////////////
//      Title     : corner_classifier
//      Project   : NRG ORP
//      Created   : 2/29/2016
//      Author    : Meredith Pitsch
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

#include "classifier/corner_classifier.h"

#include <sstream>

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "corner_classifier");

  if(argc < 2) {
    ROS_FATAL("proper usage is 'corner_classifier [autostart]");
    return -1;
  }
  bool autostart = false;
  if(argc >= 2) {
    if(std::string(argv[1])  == "true") autostart = true;
  }

  ROS_INFO("Starting Corner Classifier");
  CornerClassifier v(autostart);
  v.init();

  //cv::namedWindow( "CylinderCluster", cv::WINDOW_NORMAL );
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  //ros::Duration(10.0).sleep();
  //ros::shutdown();
  ros::waitForShutdown();
  //cv::destroyAllWindows();
  return 1;
} //main

CornerClassifier::CornerClassifier(bool autostart):
  Classifier(10000, "corner", ".", ".corner", autostart),
  maxIterations(10000),
  distanceThreshold(0.05)
{

  planes_pub_ = n.advertise<sensor_msgs::PointCloud2>("/dominant_plane",1);
  //dynamic reconfigure
  reconfigureCallbackType = boost::bind(&CornerClassifier::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);
} //CornerClassifier

void CornerClassifier::paramsChanged(orp::CornerClassifierConfig &config, uint32_t level)
{
  maxIterations = config.max_iterations;
  distanceThreshold = config.distance_threshold;

} //paramsChanged

bool CornerClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &Corner) {
  //no histogram loading
  return true;
} //loadHist

void CornerClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {
  //ROS_INFO_STREAM("Camera classification callback with " << cloud.width*cloud.height << " points.");
  orp::ClassificationResult classRes;

  //ROS_INFO("cloud acceptable size");
  PCP thisCluster = PCP(new PC());
  pcl::fromROSMsg(cloud, *thisCluster);
   // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<ORPPoint> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (distanceThreshold);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        //how many points to get leave
  int targetSize = 3;
  //ROS_INFO("target size: %d", targetSize);
  std::vector<Eigen::Vector4f> plane_vector;
  Eigen::Vector4f current_plane;
  pcl::PointCloud<ORPPoint>::Ptr processCloud (new pcl::PointCloud<ORPPoint>);

  PCP planes(new PC());
  pcl::PointCloud<ORPPoint>::Ptr planeCloud (new pcl::PointCloud<ORPPoint>);


  while(plane_vector.size() < targetSize) {
    seg.setInputCloud(thisCluster);
    seg.segment (*planeIndices, *coefficients);
    current_plane.x() = coefficients->values[0];
    current_plane.y() = coefficients->values[1];
    current_plane.z() = coefficients->values[2];
    current_plane.w() = coefficients->values[3];
    plane_vector.push_back(current_plane);
    if(planeIndices->indices.size () == 0) {
      ROS_ERROR("Could not estimate a planar model for the given dataset.");
      break;
    }
    pcl::ExtractIndices<ORPPoint> extract;
    extract.setInputCloud(thisCluster);
    extract.setIndices(planeIndices);

    extract.setNegative(false);
    extract.filter (*planeCloud);
    //store it for the planes message
    planes->insert(planes->end(), planeCloud->begin(), planeCloud->end());


    // Publish dominant planes
    sensor_msgs::PointCloud2 planes_pc2;
    pcl::toROSMsg(*planes, planes_pc2);
    planes_pc2.header.frame_id = "/camera_depth_optical_frame";
    planes_pub_.publish(planes_pc2);
    
    extract.setNegative(true);
    extract.filter(*processCloud);
    thisCluster = processCloud;  


  }
  
  Eigen::Vector3f intersection_point;
  pcl::threePlanesIntersection(plane_vector.at(0), plane_vector.at(1), plane_vector.at(2), intersection_point, 1e-6);

  ROS_INFO("Intersection at (%3.4f, %3.4f, %3.4f)", intersection_point[0], intersection_point[1], intersection_point[2]);

  geometry_msgs::Pose intersection_pose;
  intersection_pose.position.x = intersection_point[0];
  intersection_pose.position.y = intersection_point[1];
  intersection_pose.position.z = intersection_point[2];
  

  classRes.result.pose.pose = intersection_pose;
  classRes.result.pose.header.frame_id = cloud.header.frame_id;
  classRes.method = "corner";
  classRes.result.label = "corner";
  classificationPub.publish(classRes);

  delete[] kIndices.ptr();
  delete[] kDistances.ptr();
}//classify