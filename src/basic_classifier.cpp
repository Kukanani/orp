// Copyright (c) 2018, Adam Allevato
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

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>

#include "orp/core/orp_utils.h"
#include "orp/classifier/basic_classifier.h"

#include <sstream>

int main(int argc, char **argv)
{
  // Start up the name and handle command-line arguments.
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "basic_classifier");

  ROS_INFO("Starting basic Classifier");
  BasicClassifier v;
  v.init();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  return 1;
}

BasicClassifier::BasicClassifier():
  Classifier3D()
{
}

void BasicClassifier::cb_classify(const sensor_msgs::PointCloud2& cloud)
{
  ROS_DEBUG_NAMED("Basic Classfiier",
      "Received point cloud to find objects");

  orp::ClassificationResult classRes;
  classRes.method = "basic";

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  bool segmentation_succeeded = segmentation_client_.call(seg_srv);
  if(!segmentation_succeeded)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(5, "Basic Classifier", "Could not call segmentation service at " << segmentation_service_);
  }
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;

  if(!clouds.empty()) {
    for(auto eachCloud = clouds.begin();
        eachCloud != clouds.end(); eachCloud++)
    {
      if(eachCloud->width < 3 || eachCloud->width > 500) {
        continue;
      }
      pcl::PointCloud<ORPPoint>::Ptr thisCluster(
          new pcl::PointCloud<ORPPoint>);
      pcl::fromROSMsg(*eachCloud, *thisCluster);

      orp::WorldObject thisObject;
      thisObject.label = "object";
      thisObject.pose.header.frame_id = eachCloud->header.frame_id;

      Eigen::Vector4f clusterCentroid;
      pcl::compute3DCentroid(*thisCluster, clusterCentroid);

      Eigen::Affine3d finalPose;
      finalPose(0,3) = clusterCentroid(0);
      finalPose(1,3) = clusterCentroid(1);
      finalPose(2,3) = clusterCentroid(2);
      tf::poseEigenToMsg(finalPose, thisObject.pose.pose);

      thisObject.pose.pose.orientation.x = 0;
      thisObject.pose.pose.orientation.y = 0;
      thisObject.pose.pose.orientation.z = 0;
      thisObject.pose.pose.orientation.w = 1;

      thisObject.probability = 0.75;
      classRes.result.push_back(thisObject);
    }
  }
  classification_pub_.publish(classRes);
}
