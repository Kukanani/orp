// Copyright (c) 2016, Adam Allevato
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "classifier/cylinder_classifier.h"

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

  ros::init(argc, argv, "cylinder_classifier");

  if(argc < 2) {
    ROS_FATAL("proper usage is 'cylinder_classifier [autostart]");
    return -1;
  }
  bool autostart = false;
  if(argc >= 2) {
    if(std::string(argv[1])  == "true") autostart = true;
  }

  ROS_INFO("Starting Cylinder Classifier");
  CylinderClassifier v(autostart);
  v.init();

  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  ros::waitForShutdown();
  return 1;
} //main

CylinderClassifier::CylinderClassifier(bool autostart):
  Classifier(10000, "cylinder", ".", ".cylinder", autostart),
  normalDistanceWeight(0.1),
  maxIterations(10000),
  distanceThreshold(0.05),
  minRadius(0.0005),
  maxRadius(0.1)
{

  //dynamic reconfigure
  reconfigureCallbackType = boost::bind(&CylinderClassifier::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);
} //CylinderClassifier

void CylinderClassifier::paramsChanged(orp::CylinderClassifierConfig &config, uint32_t level)
{
  normalDistanceWeight = config.normal_distance_weight;
  maxIterations = config.max_iterations;
  distanceThreshold = config.distance_threshold;
  minRadius = config.min_radius;
  maxRadius = config.max_radius;

} //paramsChanged

bool CylinderClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &Cylinder) {
  //no histogram loading
  return true;
} //loadHist

void CylinderClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {
  //ROS_INFO_STREAM("Camera classification callback with " << cloud.width*cloud.height << " points.");
  orp::ClassificationResult classRes;
  classRes.method = "cylinder";

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  //ROS_INFO("Cylinder classifier calling segmentation");
  segmentationClient.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;
  //ROS_INFO("Cylinder classifier finished calling segmentation");
  //ROS_INFO("data size: %d x %d", kData->rows, kData->cols);

  if(!clouds.empty()) {
    for(std::vector<sensor_msgs::PointCloud2>::iterator eachCloud = clouds.begin(); eachCloud != clouds.end(); eachCloud++) {
      orp::WorldObject thisObject;
      if(eachCloud->width < 3) {
        //ROS_INFO("Cloud too small!");
        continue;
      }
      //ROS_INFO("cloud acceptable size");
      pcl::PointCloud<ORPPoint>::Ptr thisCluster (new pcl::PointCloud<ORPPoint>);
      pcl::fromROSMsg(*eachCloud, *thisCluster);

      Eigen::Vector4f clusterCentroid;
      pcl::compute3DCentroid(*thisCluster, clusterCentroid);
      
      pcl::PointCloud<pcl::Normal>::Ptr thisClusterNormals (new pcl::PointCloud<pcl::Normal>);
      pcl::NormalEstimation<ORPPoint, pcl::Normal> ne;
      
      pcl::search::KdTree<ORPPoint>::Ptr tree (new pcl::search::KdTree<ORPPoint> ());
      ne.setSearchMethod (tree);
      ne.setInputCloud (thisCluster);
      ne.setKSearch (50);
      ne.compute (*thisClusterNormals);
      
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_CYLINDER);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight (normalDistanceWeight);
      seg.setMaxIterations (maxIterations);
      seg.setDistanceThreshold (distanceThreshold);
      seg.setRadiusLimits (minRadius, maxRadius);
      seg.setInputCloud (thisCluster);
      seg.setInputNormals (thisClusterNormals);

      pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
      seg.segment (*inliers_cylinder, *coefficients_cylinder);
    // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
      
      Eigen::Affine3d finalPose;
      
      finalPose(0,3) = coefficients_cylinder->values[0];
      finalPose(1,3) = coefficients_cylinder->values[1];
      
      //the z value is so special because the cylinder model has infinite height. So instead we take the midway point
      // between the highest-z and lowest-z points as the center. Using a centroid-based approach wouldn't work because
      //  sometimes we can see the top/bottom of the object, which would throw us off.
      //TODO fix this to work for arbitrary axis orientations by finding the principal components, generating bounding box, etc.
      ORPPoint min_pt, max_pt;
      pcl::getMinMax3D(*thisCluster, min_pt, max_pt);
      
      finalPose(2,3) = (max_pt.z + min_pt.z)/2.0f;
      
      // http://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/
      
      Eigen::Vector3d start_vector(0.0, 0.0, 1.0); //cylinder default axis orientation: up
      Eigen::Vector3d axis_vector(coefficients_cylinder->values[3], coefficients_cylinder->values[3], coefficients_cylinder->values[5]);
      Eigen::Quaterniond rotation;
      rotation.setFromTwoVectors(start_vector, axis_vector).normalize();
      Eigen::Matrix3d rotMat; rotMat = rotation;
      finalPose.linear() = rotMat;
      
      tf::poseEigenToMsg(finalPose, thisObject.pose.pose);
      thisObject.label = "obj_red";
      classRes.result.push_back(thisObject);
      delete[] kIndices.ptr();
      delete[] kDistances.ptr();
    }
  }
  classificationPub.publish(classRes);
} //classify