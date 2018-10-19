// Copyright (c) 2015, Adam Allevato
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

#include "orp/core/segmentation.h"

int main(int argc, char **argv)
{
  // Start the segmentation node and all ROS publishers
  ros::init(argc, argv, "segmentation");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  ROS_INFO("Starting Segmentation");
  Segmentation s;
  s.run();
  return 1;
} //main

Segmentation::Segmentation() :
  node("segmentation"),
  transformToFrame(),
  listener(),
  spinner(4),
  maxClusters(100)
{
  ros::NodeHandle privateNode("~");
  if(!privateNode.getParam("clippingFrame", transformToFrame)) {
    transformToFrame = "odom";
  }

  boundedScenePublisher =
    privateNode.advertise<sensor_msgs::PointCloud2>("bounded_scene", 5);
  voxelPublisher =
    privateNode.advertise<sensor_msgs::PointCloud2>("voxel_scene", 5);
  allPlanesPublisher =
    privateNode.advertise<sensor_msgs::PointCloud2>("all_planes", 5);
  largestObjectPublisher =
    privateNode.advertise<sensor_msgs::PointCloud2>("largest_object", 5);
  allObjectsPublisher =
    privateNode.advertise<sensor_msgs::PointCloud2>("all_objects", 5);

  segmentationServer =
    node.advertiseService("segmentation", &Segmentation::cb_segment, this);

  // dynamic reconfigure
  reconfigureCallbackType =
    boost::bind(&Segmentation::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);
}

void Segmentation::run() {
  ROS_INFO("Segmentation running...");
  spinner.start();
  ros::waitForShutdown();
}

void Segmentation::paramsChanged(
  orp::SegmentationConfig &config, uint32_t level)
{
  //spatial bounds
  minX = config.spatial_min_x;
  maxX = config.spatial_max_x;
  minY = config.spatial_min_y;
  maxY = config.spatial_max_y;
  minZ = config.spatial_min_z;
  maxZ = config.spatial_max_z;

  maxClusters = config.max_clusters;

  //Segmentation
  maxPlaneSegmentationIterations = config.max_plane_segmentation_iterations;
  segmentationDistanceThreshold = config.segmentation_distance_threshold;
  percentageToAnalyze = config.percentage_to_analyze;

  //filtering
  voxelLeafSize = config.voxel_leaf_size;

  //clustering
  clusterTolerance = config.cluster_tolerance;
  minClusterSize = config.min_cluster_size;
  maxClusterSize = config.max_cluster_size;

  _publishAllObjects = config.publishAllObjects;
  _publishAllPlanes = config.publishAllPlanes;
  _publishBoundedScene = config.publishBoundedScene;
  _publishLargestObject = config.publishLargestObject;
  _publishVoxelScene = config.publishVoxelScene;
}

bool compareClusterSize(
  const sensor_msgs::PointCloud2& a,
  const sensor_msgs::PointCloud2& b)
{
  return a.width > b.width;
}

bool Segmentation::cb_segment(orp::Segmentation::Request &req,
    orp::Segmentation::Response &response) {
  ROS_DEBUG("received segmentation request");
  if(req.scene.height * req.scene.width < 3) {
    ROS_DEBUG("Not segmenting cloud, it's too small.");
    return false;
  }

  PC tmpCloud;
  // processCloud = PCPtr(new PC());
  // inputCloud->points.clear();
  originalCloudFrame = req.scene.header.frame_id;

  sensor_msgs::PointCloud2 transformedMessage;
  sensor_msgs::PointCloud2 rawMessage;

  if(transformToFrame == "") {
    pcl::fromROSMsg(req.scene, tmpCloud);
    rawMessage.header.frame_id = req.scene.header.frame_id;
  }
  else if(listener.waitForTransform(
    req.scene.header.frame_id, transformToFrame, req.scene.header.stamp,
    ros::Duration(0.5)))
  {
    pcl_ros::transformPointCloud(transformToFrame, req.scene,
      transformedMessage, listener);
    pcl::fromROSMsg(transformedMessage, tmpCloud);
  }
  else {
    ROS_WARN_STREAM_THROTTLE(60,
      "listen for transformation from " <<
      req.scene.header.frame_id.c_str() <<
      " to " << transformToFrame.c_str() <<
      " timed out. Proceeding...");
    pcl::fromROSMsg(req.scene, tmpCloud);
    rawMessage.header.frame_id = req.scene.header.frame_id;
  }
  PCPtr inputCloud = PCPtr(new PC(tmpCloud));

  if(inputCloud->points.size() <= minClusterSize) {
    ROS_INFO_STREAM(
      "point cloud is too small to segment: Min: " <<
      minClusterSize << ", actual: " << inputCloud->points.size());
    return false;
  }

  //clip
  int preVoxel = inputCloud->points.size();
  *inputCloud =
    *(clipByDistance(inputCloud, minX, maxX, minY, maxY, minZ, maxZ));
  if(_publishBoundedScene) {
    pcl::toROSMsg(*inputCloud, transformedMessage);
    transformedMessage.header.frame_id = transformToFrame;
    boundedScenePublisher.publish(transformedMessage);
  }

  *inputCloud = *(voxelGridify(inputCloud, voxelLeafSize));

  if(!inputCloud->points.empty() && inputCloud->points.size() < preVoxel) {
    // Publish voxelized
    if(_publishVoxelScene) {
      sensor_msgs::PointCloud2 voxelized_cloud;
      pcl::toROSMsg(*inputCloud, voxelized_cloud);
      voxelized_cloud.header.frame_id = transformToFrame;
      voxelPublisher.publish(voxelized_cloud);
    }

    //remove planes
    inputCloud =
      removePrimaryPlanes(inputCloud,maxPlaneSegmentationIterations,
        segmentationDistanceThreshold, percentageToAnalyze, transformToFrame);

    if(_publishAllObjects) {
      pcl::toROSMsg(*inputCloud, transformedMessage);
      transformedMessage.header.frame_id = transformToFrame;
      allObjectsPublisher.publish(transformedMessage);
    }

    if(_publishLargestObject) {
      response.clusters = cluster(inputCloud, clusterTolerance, minClusterSize,
        maxClusterSize);
      if(!response.clusters.empty()) {
        largestObjectPublisher.publish(response.clusters[0]);
      }
    }
  } else {
    if(inputCloud->points.empty()) {
      ROS_WARN_STREAM("After filtering, the cloud "
        << "contained no points. No segmentation will occur.");
    }
    else {
      ROS_ERROR_STREAM(
        "After filtering, the cloud contained "
        << inputCloud->points.size() << " points. This is more than BEFORE "
        << "the voxel filter was applied, so something is wrong. No "
        << "segmentation will occur.");
    }
  }
  return true;
}


PCPtr Segmentation::clipByDistance(PCPtr &unclipped,
    float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {

  PCPtr processCloud = PCPtr(new PC());
  // processCloud->resize(0);

  // We must build a condition.
  // And "And" condition requires all tests to check true.
  // "Or" conditions also available.
  // Checks available: GT, GE, LT, LE, EQ.
  pcl::ConditionAnd<ORPPoint>::Ptr clip_condition(
    new pcl::ConditionAnd<ORPPoint>);
  clip_condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("x", pcl::ComparisonOps::GT, minX)));
  clip_condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("x", pcl::ComparisonOps::LT, maxX)));
  clip_condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("y", pcl::ComparisonOps::GT, minY)));
  clip_condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("y", pcl::ComparisonOps::LT, maxY)));
  clip_condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("z", pcl::ComparisonOps::GT, minZ)));
  clip_condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("z", pcl::ComparisonOps::LT, maxZ)));

  // Filter object.
  pcl::ConditionalRemoval<ORPPoint> filter;
  filter.setCondition(clip_condition);
  filter.setInputCloud(unclipped);
  // If true, points that do not pass the filter will be set to a certain value
  // (default NaN). If false, they will be just removed, but that could break
  // the structure of the cloud (organized clouds are clouds taken from
  // camera-like sensors that return a matrix-like image).
  filter.setKeepOrganized(true);
  // If keep organized was set true, points that failed the test will have
  // their Z value set to this.
  filter.setUserFilterValue(0.0);

  filter.filter(*processCloud);
  return processCloud;
}

PCPtr Segmentation::voxelGridify(PCPtr &loose, float gridSize) {
  //ROS_INFO("Voxel grid filtering...");

  PCPtr processCloud = PCPtr(new PC());
  // processCloud->resize(0);
  // Create the filtering object: downsample the dataset
  pcl::VoxelGrid<ORPPoint> vg;
  vg.setInputCloud(loose);
  vg.setLeafSize(gridSize, gridSize, gridSize);
  vg.filter(*processCloud);

  return processCloud;
}

PCPtr Segmentation::removePrimaryPlanes(PCPtr &input, int maxIterations,
  float thresholdDistance, float percentageGood, std::string parentFrame)
{
  PCPtr planes(new PC());
  PCPtr planeCloud(new PC());

  PCPtr processCloud = PCPtr(new PC());
  // processCloud->resize(0);
  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentation<ORPPoint> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (thresholdDistance);

  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PCDWriter writer;

  //how many points to get leave
  int targetSize = percentageGood * input->points.size();
  //ROS_INFO("target size: %d", targetSize);

  while(input->points.size() > targetSize) {
    seg.setInputCloud(input);
    seg.segment (*planeIndices, *coefficients);

    if(planeIndices->indices.size () == 0) {
      ROS_ERROR_THROTTLE(10,
        "Could not find any good planes in the point cloud (printed every 10s)...");
      break;
    }
    // Segment the largest planar component from the remaining cloud
    pcl::ExtractIndices<ORPPoint> extract;
    extract.setInputCloud(input);
    extract.setIndices(planeIndices);

    //extract.setNegative(false);
    extract.filter (*planeCloud);
    //store it for the planes message
    planes->insert(planes->end(), planeCloud->begin(), planeCloud->end());

    //now actually take it out
    extract.setNegative(true);
    extract.filter(*processCloud);
    input = processCloud;
    //ROS_INFO("removed a plane.");
  }

  // Publish dominant planes
  if(_publishAllPlanes) {
    sensor_msgs::PointCloud2 planes_pc2;
    pcl::toROSMsg(*planes, planes_pc2);
    planes_pc2.header.frame_id = parentFrame;
    allPlanesPublisher.publish(planes_pc2);
  }

  return input;
}

std::vector<sensor_msgs::PointCloud2> Segmentation::cluster(
  PCPtr &input, float clusterTolerance,
  int minClusterSize, int maxClusterSize)
{
  // clusters.clear();
  std::vector<sensor_msgs::PointCloud2> clusters;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<ORPPoint>::Ptr tree (new pcl::search::KdTree<ORPPoint>);
  tree->setInputCloud (input);

  IndexVector cluster_indices;
  pcl::EuclideanClusterExtraction<ORPPoint> ec;
  ec.setInputCloud(input);

  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minClusterSize);
  ec.setMaxClusterSize(maxClusterSize);
  ec.setSearchMethod(tree);

  ec.extract (cluster_indices);

  if(cluster_indices.empty()) return clusters;

  // go through the set of indices. Each set of indices is one cloud
  for(IndexVector::const_iterator it = cluster_indices.begin();
      it != cluster_indices.end(); ++it)
  {
    //extract all the points based on the set of indices
    PCPtr processCloud = PCPtr(new PC());
    for(std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end (); ++pit)
    {
      processCloud->points.push_back (input->points[*pit]);
    }
    processCloud->width = processCloud->points.size();
    processCloud->height = 1;
    processCloud->is_dense = true;

    //publish the cluster
    sensor_msgs::PointCloud2 tempROSMsg;

    pcl::toROSMsg(*processCloud, tempROSMsg);
    tempROSMsg.header.frame_id = transformToFrame;
    clusters.push_back(tempROSMsg);
  }

  if(clusters.size() > 0) {
    std::sort(clusters.begin(), clusters.end(), compareClusterSize);
  }

  return clusters;
}
