#ifndef _HISTOGRAM_SAVER_H_
#define _HISTOGRAM_SAVER_H_

#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>

#include <orp/SaveCloud.h>
#include <orp/Segmentation.h>
#include <orp/HistogramSaverConfig.h>

#include "orp/classifier/cph.h"
#include "orp/core/orp_utils.h"

/**
 * @brief Extracts features from point clouds and saves to file.
 *
 * Extracts features from point clouds and saves three types of files per cloud:
 * 1. the circular projection histogram (CPH)
 * 2. the viewpoint feature histogram (VFH)
 * 3. the raw point cloud (.CSV)
 *
 * @version 2.0
 * @ingroup objectrecognition
 * 
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class HistogramSaver {
private:
  /**
   * Standard ROS node handle.
   */
  ros::NodeHandle n;

  ros::ServiceClient segClient;
  ros::ServiceServer saveCloudSrv;

  ros::Subscriber tableCenterPointSub;

  /// Required for using dynamic reconfigure.
  dynamic_reconfigure::Server<orp::HistogramSaverConfig> reconfigureServer;
  /// Required for using dynamic reconfigure.
  dynamic_reconfigure::Server<orp::HistogramSaverConfig>::CallbackType reconfigureCallbackType;

  //where to save histogram-based files
  std::string outDir;

  //method flags
  bool savePCD, saveCPH, saveVFH, saveCVFH, save6DOF;

  //CPH options
  int cphRadialBins;
  int cphVerticalBins;

  //VFH options
  float vfhRadiusSearch;

  //CVFH options
  float cvfhRadiusSearch;
  Eigen::Vector4f tableCenterPoint; ///Used to calculate distance between point cloud center and object center.

  std::vector<float> feature;  ///Gets filled with classifier data

  /**
   * Called when dynamic_reconfigure is used to change parameters for the recognition algorithm.
   * @param config the configuration listing the parameter changes
   * @param level  mask for changed parameters.
   */
  void paramsChanged(orp::HistogramSaverConfig &config, uint32_t level);

  void writeRawCloud(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);
  void writeVFH(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);
  void writeCVFH(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);
  void writeCPH(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);
  void write6DOF(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int num);

  void setTableCenterPoint(float x, float y, float z);

public:
  /**
   * Initialize the histogram saver and set up subscribers.
   * @param nh the ROS node handle to use for subscribers, etc.
   * @param location the folder (absolute path) in which to save the output data.
   */
  HistogramSaver(ros::NodeHandle nh, std::string location);

  /**
   * Save a point cloud to file. This will automatically save multiple files to the
   * output folder, depending on which save flags have been enabled.
   *
   * @param cluster the PCL point cloud to save to file
   * @param name the name to save the files under (i.e., the cluster/object name)
   * @param angle the number for the pose. In a 4-DOF pose estimation (position + rotation
                  about vertical axis), this would be the angle from which the cluster
                  was collected. But it can also simply be a sequential number for
                  keeping track of various views of an object.
   */
  bool saveCloud(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);
  
  /**
   * ROS shadow for saveCloud. Creates a segmentation request to split the large-
   * scale point cloud into smaller clouds. Then performs saveCloud() on each
   * smaller point cloud.
   *
   * Eventually, this splitting functionality should probably be moved into
   * the segmentation node, to help keep the code modular.
   */
  bool cb_saveCloud(orp::SaveCloud::Request &req, orp::SaveCloud::Response &res);

  ///ROS shadow for setCenterPoint().
  void cb_setTableCenterPoint(geometry_msgs::Vector3 _centerPoint);

}; // HistogramSaver

#endif