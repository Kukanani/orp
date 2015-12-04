#ifndef _SEGMENTATION_H_
#define _SEGMENTATION_H_

#define _USE_MATH_DEFINES
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <orp/Segmentation.h>
#include <orp/SegmentationConfig.h>
#include <orp/ReloadParams.h>

#include "core/orp_utils.h"

//usefule PCL typedefs
typedef pcl::PointCloud<ORPPoint> PC;
typedef pcl::PointCloud<ORPPoint>::Ptr PCP;
typedef std::vector<pcl::PointIndices> IndexVector;

/**
 * @brief   Performs point cloud segmentation to clarify noisy data for object recognition.
 *
 * Filtering used includes Euclidean clustering, transformation, plane segmentation.
 *
 * @version 2.0
 * @ingroup objectrecognition
 * 
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class Segmentation {
private:
  /*================================================*/
  /* DYNAMIC RECONFIGURE */
  /*================================================*/
  /**
   * Enables usage of dynamic_reconfigure for reognition algorithm parameters.
   */
  dynamic_reconfigure::Server<orp::SegmentationConfig> 
    reconfigureServer;
  /**
   * Required for using dynamic_reconfigure.
   */
  dynamic_reconfigure::Server<orp::SegmentationConfig>::CallbackType
    reconfigureCallbackType;
  /**
   * Called when configuration parameters are changed.
   * @param config The configuration variables
   * @param level  bitmask
   */
  void paramsChanged(
    orp::SegmentationConfig &config, uint32_t level);

  /*================================================*/
  /* CLASS VARS */
  /*================================================*/
  /**
   * Standard ROS node handle
   */
  ros::NodeHandle node;
  ///Accepts the segmentation requests
  //ros::Subscriber segmentationSubscriber;
  ros::ServiceServer segmentationServer;

  ros::ServiceServer reloadParamsServer;

  /// Publishes planes cut from the scene
  ros::Publisher planesPublisher;
  /// Publishes clipped cloud for visualization
  ros::Publisher boundedScenePublisher;
  /**
   * Publishes the various clusters identified by the Euclidean algorithm.
   * All of these will lie within the bounded_scene point cloud.
   * NOTE: concatenated clouds from pre_clustering = bounded_scene - planes
   */
  ros::Publisher clustersPublisher;

  /// Publishes the first (largest) cluster in the scene.
  ros::Publisher clusterPublisher;

  tf::TransformListener listener;
  std::string transformToFrame;
  /*================================================*/
  /* PARAMS */
  /*================================================*/
  //The minimum camera-space X for the working area bounding box
  float minX; // left in world space
  float maxX; // right in world space
  float minY; // up in world space
  float maxY; // down in world space
  float minZ; // near clipping in world space
  float maxZ; // far clipping in world space

  ros::AsyncSpinner spinner;

  int maxClusters;
  /**
   * The maximum number of iterations to perform when looking for planar features.
   */
  int maxPlaneSegmentationIterations;
  /**
   * The maximum distance that a point can be from a planar feature to be considered part of that
   * planar feature.
   */
  float segmentationDistanceThreshold;
  /**
   * The percentage of the scene to analyze (pass onward to clustering).
   * The clustering algorithm will continue to remove planar features until this condition
   * is satisfied.
   *
   * 1.0 = the entire scene is one object
   * 0.0 = nothing will be analyzed
   */
  float percentageToAnalyze;
  /**
   * The distance between points in the voxel grid (used to clean up the point cloud and make
   * it well-behaved for further analysis). If this value is too small, the grid will be too fine.
   * there won't be enough integers to provide indices for each point and you will get errors.
   */
  float voxelLeafSize;
  /**
   * The maximum distance between points in a cluster (used in the Euclidean
   * clustering algorithm).
   */
  float clusterTolerance;
  /**
   * Clusters with less than this number of points won't be analyzed. Usually this is used to
   * filter out small point clouds like anomalies, noise, bits of whatnot in the scene, etc.
   */
  int minClusterSize;
  /**
   * Clusters that have more than this number of points won't be analyzed. The assumption
   * is that this is either a) too computationally intensive to analyze, or b) this is just a 
   * background object like a wall which should be ignored anyway.
   */
  int maxClusterSize;

  ///input is stored here
  PCP inputCloud;
  ///used as intermediate step for cloud processing
  PCP processCloud;

  std::vector<sensor_msgs::PointCloud2> clusters;


  /*================================================*/
  /* FILTERING STEPS (FUNCTIONS) */
  /*================================================*/

  /**
   * Spatially filter a point cloud
   * @param  unclipped the point cloud to be filtered
   * @param  minX      min bound to process
   * @param  maxX      max bound to process
   * @param  minY      min bound to process
   * @param  maxY      max bound to process
   * @param  minZ      min bound to process
   * @param  maxZ      max bound to process
   * @return           the filtered point cloud
   */
  PCP& clipByDistance(PCP &unclipped,
      float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

  /**
   * Create a voxel grid based on point cloud data. See
   * http://www.pointclouds.org/documentation/tutorials/voxel_grid.php and
   * http://docs.pointclouds.org/1.7.1/classpcl_1_1_voxel_grid.html.
   * @param  loose    unstructured (not on a grid) point cloud
   * @param  gridSize the distance between voxels in the grid
   * @return          the points of the voxel grid created from the input
   */
  PCP& voxelGridify(PCP &loose, float gridSize);

  /**
   * Segment out planar clouds. See
   * http://pointclouds.org/documentation/tutorials/planar_segmentation.php
   * @param  input             the point cloud from which to remove planes
   * @param  maxIterations     maximum iterations for clustering algorithm.
   * @param  thresholdDistance how close a point must be to hte model in order to be considered
   *  an inlier.
   * @param  percentageGood    keep removing planes until the amount of data left is less than this
   *  percentage of the initial data.
   * @return                   the point cloud with primary planes removed as specified.
   */
  PCP& removePrimaryPlanes(PCP &input, int maxIterations, float thresholdDistance, float percentageGood);
  
  /**
   * Euclidean clustering algorithm. See
   * http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
   * @param input            the cloud to cluster
   * @param clusterTolerance the maximum distance between points in a given cluster
   * @param minClusterSize   clusters of size less than this will be discarded
   * @param maxClusterSize   clusters of size greater than this will be discarded
   * @return                 a vector of point clouds, each representing a cluster from the clustering
   *  algorithm.
   */
  std::vector<sensor_msgs::PointCloud2> cluster(PCP &input, float clusterTolerance, int minClusterSize, int maxClusterSize);
public:
  /**
   * The only constructor.
   * @param nh The already-initalized ROS node handle
   */
  Segmentation();

  /**
   * Reload parameters from the parameter server (instead of being set by dynamic reconfigure).
   * @return true if all parameters could be set.
   */
  bool reloadParams();
  
  void run();

  //Callbacks
  bool cb_reloadParams(
      orp::ReloadParams::Request &request,
      orp::ReloadParams::Response &response);

  /**
   * Linked to incoming camera data Orchestrates the segmentation process.
   * @param  cloud the cloud to segment
   */
  //void cb_segment(sensor_msgs::PointCloud2 incoming);
  bool processSegmentation(orp::Segmentation::Request &req,
      orp::Segmentation::Response &response);
}; //Segmentation

#endif //_SEGMENTATION_H_
