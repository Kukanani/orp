///////////////////////////////////////////////////////////////////////////////
//      Title     : Segmentation
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

#include "core/segmentation.h"

/**
 * Start the segmentation node and all ROS publishers
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
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
    transformToFrame = "world";
  }

  boundedScenePublisher = node.advertise<sensor_msgs::PointCloud2>("/bounded_scene",1);
  voxelPublisher = node.advertise<sensor_msgs::PointCloud2>("/voxel_scene",1);
  allPlanesPublisher = node.advertise<sensor_msgs::PointCloud2>("/all_planes",1);
  largestObjectPublisher = node.advertise<sensor_msgs::PointCloud2>("/largest_object",1);
  allObjectsPublisher = node.advertise<sensor_msgs::PointCloud2>("/all_objects",1);
  
  segmentationServer = node.advertiseService("/segmentation", &Segmentation::processSegmentation, this);
  
  reconfigureCallbackType = boost::bind(&Segmentation::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);
}

void Segmentation::run() {
  ROS_INFO("Segmentation running...");
  spinner.start();
  ros::waitForShutdown();
}

void Segmentation::paramsChanged(orp::SegmentationConfig &config, uint32_t level)
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

bool compareClusterSize(const sensor_msgs::PointCloud2& a, const sensor_msgs::PointCloud2& b) { return a.width > b.width; }

bool Segmentation::processSegmentation(orp::Segmentation::Request &req,
    orp::Segmentation::Response &response) {
  if(req.scene.height * req.scene.width < 3) {
    ROS_DEBUG("Not segmenting cloud, it's too small.");
    return false;
  }

  inputCloud = PCP(new PC());
  processCloud = PCP(new PC());
  inputCloud->points.clear();
  originalCloudFrame = req.scene.header.frame_id;

  sensor_msgs::PointCloud2 transformedMessage;
  sensor_msgs::PointCloud2 rawMessage;

  if(transformToFrame == "") {
    pcl::fromROSMsg(req.scene, *inputCloud);
    rawMessage.header.frame_id = req.scene.header.frame_id;
  }
  else if(listener.waitForTransform(req.scene.header.frame_id, transformToFrame, req.scene.header.stamp, ros::Duration(0.5)))
  {
    pcl_ros::transformPointCloud (transformToFrame, req.scene, transformedMessage, listener);
    pcl::fromROSMsg(transformedMessage, *inputCloud);
  }
  else {
    ROS_WARN_THROTTLE(60, "[ORP Segmentation] listen for transformation from %s to %s timed out. Proceeding...", req.scene.header.frame_id.c_str(), transformToFrame.c_str());
    pcl::fromROSMsg(req.scene, *inputCloud);
    rawMessage.header.frame_id = req.scene.header.frame_id;
  }

  if(inputCloud->points.size() <= minClusterSize) {
    ROS_INFO_STREAM("[ORP Segmentation] point cloud is too small to segment: Min: " << minClusterSize << ", actual: " << inputCloud->points.size());
    return false;
  }

  //clip
  int preVoxel = inputCloud->points.size();
  *inputCloud = *(clipByDistance(inputCloud, minX, maxX, minY, maxY, minZ, maxZ));
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
    inputCloud = removePrimaryPlanes(inputCloud, maxPlaneSegmentationIterations, segmentationDistanceThreshold, percentageToAnalyze);

    if(_publishAllObjects) {
      pcl::toROSMsg(*inputCloud, transformedMessage);
      transformedMessage.header.frame_id = transformToFrame;
      allObjectsPublisher.publish(transformedMessage);
    }

    if(_publishLargestObject) {
      response.clusters = cluster(inputCloud, clusterTolerance, minClusterSize, maxClusterSize);
      if(!response.clusters.empty()) {
        largestObjectPublisher.publish(response.clusters[0]);
      }
    }
  } else {
    if(inputCloud->points.empty()) {
      ROS_WARN_STREAM("[ORP Segmentation] After filtering, the cloud contained no points. No segmentation will occur.");
    }
    else {
      ROS_ERROR_STREAM("[ORP Segmentation] After filtering, the cloud contained " << inputCloud->points.size() << " points. This is more than BEFORE the voxel filter was applied, so something is wrong. No segmentation will occur.");
    }
  }
  return true;
}


PCP& Segmentation::clipByDistance(PCP &unclipped,
    float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {

  processCloud->resize(0);

  // We must build a condition.
  // And "And" condition requires all tests to check true. "Or" conditions also available.
  // Checks available: GT, GE, LT, LE, EQ.
  pcl::ConditionAnd<ORPPoint>::Ptr condition(new pcl::ConditionAnd<ORPPoint>);
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("x", pcl::ComparisonOps::GT, minX)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("x", pcl::ComparisonOps::LT, maxX)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("y", pcl::ComparisonOps::GT, minY)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("y", pcl::ComparisonOps::LT, maxY)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("z", pcl::ComparisonOps::GT, minZ)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("z", pcl::ComparisonOps::LT, maxZ)));
 
  // Filter object.
  pcl::ConditionalRemoval<ORPPoint> filter(condition);
  filter.setInputCloud(unclipped);
  // If true, points that do not pass the filter will be set to a certain value (default NaN).
  // If false, they will be just removed, but that could break the structure of the cloud
  // (organized clouds are clouds taken from camera-like sensors that return a matrix-like image).
  filter.setKeepOrganized(true);
  // If keep organized was set true, points that failed the test will have their Z value set to this.
  filter.setUserFilterValue(0.0);
 
  filter.filter(*processCloud);
  return processCloud;
}

PCP& Segmentation::voxelGridify(PCP &loose, float gridSize) {
  //ROS_INFO("Voxel grid filtering...");

  processCloud->resize(0);
  // Create the filtering object: downsample the dataset
  pcl::VoxelGrid<ORPPoint> vg;
  vg.setInputCloud(loose);
  vg.setLeafSize(gridSize, gridSize, gridSize);
  vg.filter(*processCloud);
  
  return processCloud;
}

PCP& Segmentation::removePrimaryPlanes(PCP &input, int maxIterations, float thresholdDistance,
    float percentageGood) {
  //ROS_INFO("Filtering planes...");
  PCP planes(new PC());
  PCP planeCloud(new pcl::PointCloud<ORPPoint> ());

  processCloud->resize(0);
  // Create the segmentation object for the planar model and set all the parameters
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
      ROS_ERROR("[ORP Segmentation] Could not find any good planes in the point cloud.");
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
    planes_pc2.header.frame_id = originalCloudFrame;
    allPlanesPublisher.publish(planes_pc2);
  }

  return input;
}

std::vector<sensor_msgs::PointCloud2> Segmentation::cluster(PCP &input, float clusterTolerance,
    int minClusterSize, int maxClusterSize) {
  clusters.clear();

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
  for (IndexVector::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    //extract all the points based on the set of indices
    processCloud = PCP(new PC());
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); ++pit) {
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
