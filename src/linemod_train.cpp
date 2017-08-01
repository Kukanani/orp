// Copyright (c) 2016, Adam Allevato
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

#include "linemod_train.h"

#include <pcl/segmentation/sac_segmentation.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "linemod_train");

  LinemodTrain t = LinemodTrain();

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 1;
}

LinemodTrain::LinemodTrain() : node_()
{
  // TODO(Kukanani): remove these magic constants and replace with
  // configurable parameters (ROS params?)
  DISTANCE_THRESHOLD = 0.01;
  FAR_CLIP = 2.0;
  NEAR_CLIP = 0.5;
  MAX_RANSAC_ITERS = 1000;
  X_EXTENT = 0.3;

  depth_cb_ = node_.subscribe(
    "/camera/depth_registered/points", 1, &LinemodTrain::cb_depth, this);
}

void LinemodTrain::cb_depth(sensor_msgs::PointCloud2 cloud_msg)
{
  cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::fromROSMsg(cloud_msg, *cloud);

  std::vector<bool> mask = calculate_mask(cloud);
  for (size_t i = 0; i < mask.size (); ++i)
  {
    if (!mask[i])
    {
      pcl::PointXYZRGBA & p = cloud->points[i];
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
    }
  }
  pcl::MaskMap mask_map(cloud->width, cloud->height);

  size_t min_x(cloud->width), min_y(cloud->height), max_x(0), max_y(0);

  //find the maximum and minimum unmasked pixels
  for(size_t j=0; j<cloud->height; ++j) {
    for(size_t i=0; i<cloud->width; ++i) {
      mask_map(i,j) = mask[j*cloud->width+i];
      if(mask[j*cloud->width+i]) {
        min_x = std::min(min_x, i);
        max_x = std::max(max_x, i);
        min_y = std::min(min_y, j);
        max_y = std::max(max_y, j);
      }
    }
  }

  ROS_INFO_STREAM("mask params: " << min_x << ", " << max_x << ", " << min_y << ", " << max_y);

  pcl::RegionXY region;
  region.x = static_cast<int>(min_x);
  region.y = static_cast<int>(min_y);
  region.width = std::max(1,static_cast<int>(max_x-min_x+1));
  region.height = std::max(1,static_cast<int>(max_y-min_y+1));
  ROS_INFO_STREAM("saving one! There are " << cloud->size() << " points");

  line_rgbd_.createAndAddTemplate(*cloud, 10, mask_map, mask_map, region);

  line_rgbd_.linemod_.saveTemplates("test.test");
  ROS_INFO("saved one!");
  ros::shutdown();
}

std::vector<bool> LinemodTrain::calculate_mask(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  pcl::IndicesPtr clipped(new std::vector<int>());

  std::vector<bool> mask = std::vector<bool>(cloud->size(), false);

  //clip far and near pixels
  for(size_t i = 0; i < cloud->size(); ++i) {
    float z = cloud->points[i].z;
    float x = cloud->points[i].x;
    if(z > NEAR_CLIP && z < FAR_CLIP && x < X_EXTENT && x > -X_EXTENT)
    {
      //use
      mask[i] = true;
      clipped->push_back(static_cast<int>(i));
    }
  }

  //do RANSAC to remove floor
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (MAX_RANSAC_ITERS);
  seg.setDistanceThreshold (DISTANCE_THRESHOLD);

  seg.setInputCloud(cloud);
  seg.setIndices(clipped);

  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //unused (but required for call)
  seg.segment (*plane_inliers, *coefficients);

  for(size_t i = 0; i < plane_inliers->indices.size(); ++i) {
    mask[plane_inliers->indices[i]] = false;
  }

  return mask;
}