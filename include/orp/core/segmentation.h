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

#ifndef _SEGMENTATION_H_
#define _SEGMENTATION_H_

#define _USE_MATH_DEFINES
#include <cmath>

// TODO(Kukanani): make sure all of these includes are still needed, and
// see which can be moved to the .cpp file instead of slowing down the compile
// here in the .h
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include <orp/Segmentation.h>
#include <orp/SegmentationConfig.h>

#include "orp/core/orp_utils.h"

/**
 * @brief Performs point cloud segmentation to clarify noisy data for object
 *        recognition.
 *
 * Filtering used includes Euclidean clustering, transformation, plane
 * segmentation. Many of these are modified versions of the PCL tutorials with
 * their parameters exposed.
 *
 * TODO(Kukanani): this could be combined with other ORP nodes into one process
 * by using nodelets. That has ramifications for debugging and readability, but
 * will result in improved performance, because it's very expensive to pass
 * point clouds from node to node.
 *
 * @version 2.0
 * @ingroup objectrecognition
 *
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 */
class Segmentation {
private:
///////////////////////////////////////////////////////////////////////////////
// DYNAMIC RECONFIGURE
///////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////
// CLASS VARS
///////////////////////////////////////////////////////////////////////////////
  /// Standard ROS node handle
  ros::NodeHandle node;
  /// Because of the time taken to process segmentations, use a multithreaded
  /// spinner.
  ros::AsyncSpinner spinner;

  /// Accepts the segmentation requests
  ros::ServiceServer segmentationServer;

  /// Publishes planes cut from the scene
  ros::Publisher allPlanesPublisher;
  /// Publishes clipped cloud for visualization
  ros::Publisher boundedScenePublisher;
  /// Publishes clipped cloud after voxel gridification
  ros::Publisher voxelPublisher;
  /**
   * Publishes the various clusters identified by the Euclidean algorithm.
   * All of these will lie within the bounded_scene point cloud.
   * NOTE: concatenated clouds from pre_clustering = bounded_scene - planes
   */
  ros::Publisher allObjectsPublisher;

  /// Publishes the first (largest) cluster in the scene.
  ros::Publisher largestObjectPublisher;

  /// Used to transform into the correct processing/recognition frames
  tf::TransformListener listener;

  /// The frame to transform the recognition results into. This is useful if
  ///   you need to use recognition results for
  ///   motion planning in a specific frame, or pose x/y/z values, etc.
  std::string transformToFrame;

  /// updated after each message received
  std::string originalCloudFrame;

///////////////////////////////////////////////////////////////////////////////
// SEGMENTATION PARAMS
///////////////////////////////////////////////////////////////////////////////
  /// Minimum X for processing area bounding box (left in world space)
  float minX;
  /// Maximum X for processing area bounding box (right in world space)
  float maxX;
  /// Minimum Y for processing area bounding box (up in world space)
  float minY;
  /// Maximum Y for processing area bounding box (down in world space)
  float maxY;
  /// Minimum Z for processing area bounding box (near clipping in world space)
  float minZ;
  /// Maximum Z for processing area bounding box (far clipping in world space)
  float maxZ;

  /// Publish a point cloud of all objects?
  bool _publishAllObjects;
  /// Publish a point cloud of all removed planes?
  bool _publishAllPlanes;
  /// Publish a point cloud of the scene after clipping to the processing area?
  bool _publishBoundedScene;
  /// Publish a point cloud of only the largest object?
  bool _publishLargestObject;
  /// Publish a point cloud of the bounded scene after voxelization?
  bool _publishVoxelScene;

  /// Maximum number of object clusters to process
  int maxClusters;
  /**
   * The maximum number of iterations to perform when looking for planar
   * features.
   */
  int maxPlaneSegmentationIterations;
  /**
   * The maximum distance that a point can be from a planar feature to be
   * considered part of that planar feature.
   */
  float segmentationDistanceThreshold;
  /**
   * The percentage of the scene to analyze (pass onward to clustering).
   * The clustering algorithm will continue to remove planar features until
   * this condition is satisfied.
   *
   * 1.0 = the entire scene is one object
   * 0.0 = nothing will be analyzed
   */
  float percentageToAnalyze;
  /**
   * The distance between points in the voxel grid (used to clean up the point
   * cloud and make it well-behaved for further analysis). If this value is too
   * small, there won't be enough integers to provide indices for each point
   * and you will get errors.
   */
  float voxelLeafSize;
  /**
   * The maximum distance between points in a cluster (used in the Euclidean
   * clustering algorithm).
   */
  float clusterTolerance;
  /**
   * Clusters with less than this number of points won't be analyzed. Usually
   * this is used to filter out small point clouds like anomalies, noise, bits
   * of whatnot in the scene, etc.
   */
  int minClusterSize;
  /**
   * Clusters that have more than this number of points won't be analyzed. The
   * assumption is that this is either a) too computationally intensive to
   * analyze, or b) this is just a large background object like a wall which
   * should be ignored anyway.
   */
  int maxClusterSize;

  /// Input is stored here
  // PCPtr inputCloud;
  /// Used as intermediate step for cloud processing, without having to
  /// realloc more clouds.
  /// TODO(kukanani): does this actually save any time? Use of this variable
  /// should be revisited.
  // PCPtr processCloud;

  std::vector<sensor_msgs::PointCloud2> clusters;

///////////////////////////////////////////////////////////////////////////////
// FILTERING STEPS (FUNCTIONS)
///////////////////////////////////////////////////////////////////////////////

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
  PCPtr clipByDistance(PCPtr &unclipped,
      float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

  /**
   * Create a voxel grid based on point cloud data. See
   * http://www.pointclouds.org/documentation/tutorials/voxel_grid.php and
   * http://docs.pointclouds.org/1.7.1/classpcl_1_1_voxel_grid.html.
   * @param  loose    unstructured (not on a grid) point cloud
   * @param  gridSize the distance between voxels in the grid
   * @return          the points of the voxel grid created from the input
   */
  PCPtr voxelGridify(PCPtr &loose, float gridSize);

  /**
   * Segment out planar clouds. See
   * http://pointclouds.org/documentation/tutorials/planar_segmentation.php
   * @param  input             the point cloud from which to remove planes
   * @param  maxIterations     maximum iterations for clustering algorithm.
   * @param  thresholdDistance how close a point must be to hte model in order
   *                           to be considered an inlier.
   * @param  percentageGood    keep removing planes until the amount of data
   *                           left is less than this percentage of the initial
   *                           data.
   * @return                   the point cloud with primary planes removed as
   *                           specified.
   */
  PCPtr removePrimaryPlanes(PCPtr &input, int maxIterations,
      float thresholdDistance, float percentageGood);

  /**
   * Euclidean clustering algorithm. See
   * http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
   * @param input            the cloud to cluster
   * @param clusterTolerance the maximum distance between points in a given
   *                         cluster
   * @param minClusterSize   clusters of size less than this will be discarded
   * @param maxClusterSize   clusters of size greater than this will be
   *                         discarded
   * @return                 a vector of point clouds, each representing a
   *                         cluster from the clustering algorithm.
   */
  std::vector<sensor_msgs::PointCloud2> cluster(PCPtr &input,
      float clusterTolerance, int minClusterSize, int maxClusterSize);
public:
  /**
   * Default constructor
   */
  Segmentation();

  /// Start!
  void run();

  /// Do the segmentation steps enabled by parameter flags and return the
  /// result.
  bool cb_segment(orp::Segmentation::Request &req,
      orp::Segmentation::Response &response);
};

#endif
