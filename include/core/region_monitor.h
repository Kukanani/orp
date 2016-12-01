

#ifndef _REGION_MONITOR_H_
#define _REGION_MONITOR_H_

#define _USE_MATH_DEFINES
#include <cmath>

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
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <orp/Monitor.h>
#include <orp/Region.h>

#include "core/orp_utils.h"

//usefule PCL typedefs
typedef pcl::PointCloud<ORPPoint> PC;
typedef pcl::PointCloud<ORPPoint>::Ptr PCP;
typedef std::vector<pcl::PointIndices> IndexVector;

class RegionMonitor {
private:
  /*================================================*/
  /* CLASS VARS */
  /*================================================*/
  /// Standard ROS node handle
  ros::NodeHandle node;
  ros::AsyncSpinner spinner;

  /// Accepts the segmentation requests
  ros::ServiceServer monitorServer;

  /// Set the region of interest
  ros::Subscriber regionSub;

  /// Publishes clipped cloud for visualization
  ros::Publisher boundedScenePublisher;

  /// Listens for point clouds
  ros::Subscriber pointCloudSub;

  /// Used to transform into the correct processing/recognition frames
  tf::TransformListener listener;
  std::string transformToFrame;



  /*================================================*/
  /* SEGMENTATION PARAMS */
  /*================================================*/
  //The minimum camera-space X for the working area bounding box
  float minX; // left in world space
  float maxX; // right in world space
  float minY; // up in world space
  float maxY; // down in world space
  float minZ; // near clipping in world space
  float maxZ; // far clipping in world space
  
  ///input is stored here
  PCP inputCloud;
  ///used as intermediate step for cloud processing
  PCP processCloud;


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
  PCP& clipByDistance(PCP &unclipped);

public:
  RegionMonitor();

  /// Start!
  void run();

  /// Do the segmentation steps enabled by parameter flags and return the result.
  void cb_params(const orp::Region::ConstPtr& region);
  void cb_pointCloud(const sensor_msgs::PointCloud::ConstPtr& cloud);

  bool cb_monitor(orp::MonitorRequest& req, orp::MonitorResponse& res);
}; //RegionMonitor

#endif //_SEGMENTATION_H_
