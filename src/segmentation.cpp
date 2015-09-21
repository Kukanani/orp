#include "orp/core/segmentation.h"

Segmentation::Segmentation() :
  node("segmentation"),
  transformToFrame(""),
  listener(),
  spinner(4),
  maxClusters(100)
{
  planesPublisher = node.advertise<sensor_msgs::PointCloud2>("/dominant_plane",1);
  boundedScenePublisher = node.advertise<sensor_msgs::PointCloud2>("/bounded_scene",1);
  clustersPublisher = node.advertise<sensor_msgs::PointCloud2>("/pre_clustering",1);
  segmentationServer = node.advertiseService("/segmentation", &Segmentation::processSegmentation, this);
  reloadParamsServer = node.advertiseService("/reload_params", &Segmentation::cb_reloadParams, this);

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
} //paramsChanged

bool Segmentation::reloadParams()
{
  ROS_INFO("reloading spatial segmentation parameters from parameter server");
  bool result = true;
  result &= ORPUtils::attemptToReloadFloatParam(node, "spatial_min_x", minX);
  result &= ORPUtils::attemptToReloadFloatParam(node, "spatial_max_x", maxX);
  result &= ORPUtils::attemptToReloadFloatParam(node, "spatial_min_y", minY);
  result &= ORPUtils::attemptToReloadFloatParam(node, "spatial_max_y", maxY);
  result &= ORPUtils::attemptToReloadFloatParam(node, "spatial_min_z", minZ);
  result &= ORPUtils::attemptToReloadFloatParam(node, "spatial_max_z", maxZ);
  return result;
} //loadParams

bool Segmentation::cb_reloadParams(
    orp::ReloadParams::Request &request,
    orp::ReloadParams::Response &response)
{
  return reloadParams();
}

bool compareClusterSize(const sensor_msgs::PointCloud2& a, const sensor_msgs::PointCloud2& b)
{
  return a.width > b.width;
}

bool Segmentation::processSegmentation(orp::Segmentation::Request &req,
    orp::Segmentation::Response &response) {
  ROS_DEBUG("Segmenting...");
  inputCloud = PCP(new PC());
  processCloud = PCP(new PC());
  inputCloud->points.clear();

  sensor_msgs::PointCloud2 transformedMessage;
  sensor_msgs::PointCloud2 pc2_message;

  std::string transformFromFrame = req.scene.header.frame_id;

  //ROS_INFO("segmentation: going from %s to %s", req.scene.header.frame_id.c_str(), transformToFrame.c_str());

  // if(transformToFrame != "" && 
  //   listener.waitForTransform(req.scene.header.frame_id, transformToFrame, req.scene.header.stamp, ros::Duration(0.5)))
  // {
  //   pcl_ros::transformPointCloud (transformToFrame, req.scene, transformedMessage, listener);
  //   pcl::fromROSMsg(transformedMessage, *inputCloud);
  //   //pc2_message.header.frame_id = transformToFrame;
  // }
  //else {
    //ROS_WARN_THROTTLE(60, "Segmentation: listen for transformation to %s timed out. Proceeding...", transformToFrame.c_str());
    pcl::fromROSMsg(req.scene, *inputCloud);
    pc2_message.header.frame_id = req.scene.header.frame_id;
  //}

  if(inputCloud->points.size() <= minClusterSize) {
    ROS_INFO("Segmentation point cloud is too small.");
    return false;
  }

  //ROS_INFO("%d points remain in dataset before spatial/voxel filtering (%f).",
  //  (int) inputCloud->points.size(), voxelLeafSize);
  //ROS_INFO_STREAM("The first point in the cloud is " << inputCloud->points.front().x << ", " << inputCloud->points.front().y << ", " << inputCloud->points.front().z);

  //clip
  int preVoxel = inputCloud->points.size();
  *inputCloud = *(clipByDistance(inputCloud, minX, maxX, minY, maxY, minZ, maxZ));
  *inputCloud = *(voxelGridify(inputCloud, voxelLeafSize));

  //ROS_INFO("%d points remain in dataset after spatial/voxel filtering (%f).",
  //  (int) inputCloud->points.size(), voxelLeafSize);

  if(inputCloud->points.size() > 0 && inputCloud->points.size() < preVoxel) {

    pcl::toROSMsg(*inputCloud, pc2_message);
    boundedScenePublisher.publish(pc2_message);

    //remove planes
    inputCloud = removePrimaryPlanes(inputCloud, maxPlaneSegmentationIterations,
      segmentationDistanceThreshold, percentageToAnalyze);
    //ROS_INFO("%d points remain in dataset after removing planes.", (int) inputCloud->points.size());

    pcl::toROSMsg(*inputCloud, pc2_message);
    //pc2_message.header.frame_id = transformToFrame;
    clustersPublisher.publish(pc2_message);

    //ROS_INFO("clustering");
    response.clusters = cluster(inputCloud, clusterTolerance,
      minClusterSize, maxClusterSize);
    for(std::vector<sensor_msgs::PointCloud2>::iterator it = response.clusters.begin(); it != response.clusters.end(); ++it) {
      //pcl_ros::transformPointCloud (transformFromFrame, *it, *it, listener);
    }
  } else {
    ROS_WARN_STREAM("not clustering filtered point cloud containing " << inputCloud->points.size() << " points.");
  }
  // ROS_INFO("segmentation call finished");
  return true;
} //processSegmentation


PCP& Segmentation::clipByDistance(PCP &unclipped,
    float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {

  processCloud->resize(0);
  //ROS_INFO("Spatial filtering. Bounds are %f - %f, %f - %f, %f - %f", 
 //   minX, maxX, minY, maxY, minZ, maxZ);

  // We must build a condition.
  // And "And" condition requires all tests to check true. "Or" conditions also available.
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
  // Checks available: GT, GE, LT, LE, EQ.
 
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
} //clipByDistance

PCP& Segmentation::voxelGridify(PCP &loose, float gridSize) {
  //ROS_INFO("Voxel grid filtering...");

  processCloud->resize(0);
  // Create the filtering object: downsample the dataset
  pcl::VoxelGrid<ORPPoint> vg;
  vg.setInputCloud(loose);
  vg.setLeafSize(gridSize, gridSize, gridSize);
  vg.filter(*processCloud);
  return processCloud;
} //voxelGridify

PCP& Segmentation::removePrimaryPlanes(PCP &input, int maxIterations, float thresholdDistance,
    float percentageGood) {
  //ROS_INFO("Filtering planes...");
  //PCP planes(new PC());

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
  //PCP planeCloud(new pcl::PointCloud<ORPPoint> ());
  pcl::PCDWriter writer;

  //how many points to get leave
  int targetSize = percentageGood * input->points.size();
  //ROS_INFO("target size: %d", targetSize);

  while(input->points.size() > targetSize) {
    //ROS_INFO_STREAM("num points in cloud: " << input->points.size());
    seg.setInputCloud(input);
    seg.segment (*planeIndices, *coefficients);

    if(planeIndices->indices.size () == 0) {
      ROS_ERROR("Could not estimate a planar model for the given dataset.");
      break;
    }
    // Segment the largest planar component from the remaining cloud
    pcl::ExtractIndices<ORPPoint> extract;
    extract.setInputCloud(input);
    extract.setIndices(planeIndices);

    //extract.setNegative(false);
    //extract.filter (*planeCloud);
    //store it for the planes message
    //planes->insert(planes->end(), planeCloud->begin(), planeCloud->end());

    //now actually take it out
    extract.setNegative(true);
    extract.filter(*processCloud);
    input = processCloud;
    //ROS_INFO("removed a plane.");
  }

  // Publish dominant planes
  //sensor_msgs::PointCloud2 planes_pc2;
  //pcl::toROSMsg(*planes, planes_pc2);
  //planes_pc2.header.frame_id = "/camera_depth_optical_frame";
  //planesPublisher.publish(planes_pc2);

  return input;
} //removePrimaryPlanes

std::vector<sensor_msgs::PointCloud2> Segmentation::cluster(PCP &input, float clusterTolerance,
    int minClusterSize, int maxClusterSize) {
  clusters.clear();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<ORPPoint>::Ptr tree (new pcl::search::KdTree<ORPPoint>);
  tree->setInputCloud (input);

  IndexVector cluster_indices;
  pcl::EuclideanClusterExtraction<ORPPoint> ec;
  ec.setClusterTolerance(clusterTolerance); 
  ec.setMinClusterSize(minClusterSize); 
  ec.setMaxClusterSize(maxClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  //ROS_INFO("extracting clusters...");
  ec.extract (cluster_indices);

  //ROS_INFO("found %d clusters", (int) cluster_indices.size());
  if(cluster_indices.size() <= 0) return clusters;

  int j = 0;
  for (IndexVector::const_iterator it = cluster_indices.begin();
      it != cluster_indices.end(); ++it) {
    processCloud->resize(0);
    //ROS_INFO("Looping through indices for this cluster");
    for (std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end (); pit++) {
      processCloud->points.push_back (input->points[*pit]);
    }
    processCloud->width = processCloud->points.size();
    processCloud->height = 1;
    processCloud->is_dense = true;
    //ROS_INFO("writing cluster to segmentation service response. It has %d points.",
    //  (int) processCloud->points.size());
    sensor_msgs::PointCloud2 tempROSMsg;
    pcl::toROSMsg(*processCloud, tempROSMsg);
    clusters.push_back(tempROSMsg);
    j++;
  }

  if(clusters.size() > 0) {
    std::sort(clusters.begin(), clusters.end(), compareClusterSize);
  }

  return clusters;
} //cluster

/**
 * Start the segmentation node and all ROS publishers
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation");
  ROS_INFO("Starting Segmentation");
  Segmentation s;
  s.run();
  return 1;
} //main