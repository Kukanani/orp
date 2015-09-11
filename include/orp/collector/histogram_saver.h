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

  ros::ServiceClient seg_client;
  ros::ServiceServer SaveCloud_serv;


  /**
   * Enables usage of dynamic_reconfigure for reognition algorithm parameters.
   */
  dynamic_reconfigure::Server<orp::HistogramSaverConfig> 
    reconfigureServer;
  /**
   * Required for using dynamic_reconfigure.
   */
  dynamic_reconfigure::Server<orp::HistogramSaverConfig>::CallbackType
    reconfigureCallbackType;

  std::string outDir;

  bool savePCD, saveCPH, saveVFH, saveCVFH, save6DOF;

  int cphRadialBins;
  int cphVerticalBins;
  float vfhRadiusSearch;
  float cvfhRadiusSearch;

  //used to fill with CPH data
  std::vector<float> feature;

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

public:
  HistogramSaver(ros::NodeHandle nh, std::string location);

  bool cloud_cb(orp::SaveCloud::Request &req,
    orp::SaveCloud::Response &res);

}; // HistogramSaver

int main(int argc, char **argv)
{
  ros::init(argc, argv, "histogram_saver");
  ros::NodeHandle n;

  std::string location = argc > 1 ? argv[1] : ".";

  ROS_INFO("Starting Histogram Saver");
  HistogramSaver* hs = new HistogramSaver(n, location);
  ros::spin();
  return 0;
}; //main

#endif // _HISTOGRAM_SAVER_H_