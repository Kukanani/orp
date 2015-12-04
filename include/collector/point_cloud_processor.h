#ifndef _POINT_CLOUD_PROCESSOR_H_
#define _POINT_CLOUD_PROCESSOR_H_

#include <sstream>

#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <orp/DataCollect.h>
#include <orp/SaveCloud.h>

#include "core/orp_utils.h"

/**
 * @brief A pipeline for doing arbitrary transformations (batch processing) on pcd files.
 *
 *
 * @version 1.0
 * @ingroup objectrecognition
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class PointCloudProcessor {
public:
  /**
   * Standard ROS node handle.
   */
  ros::NodeHandle n;

  bool bigBird;                         //true if using bigbird dataset, true by default.

  ros::ServiceClient histogram_client;  ///Used to send point clouds off for file processing.
  orp::SaveCloud srv;                   ///the message to send to the histogram client

  /**
   * Constructor
   * @arg nh the ros NodeHandle to use for everything.
   * @arg path the file path to recursively search for .PCD files.
   */
  PointCloudProcessor(ros::NodeHandle nh, std::string path, std::string out, std::string name, bool bb);

  /**
   * Takes an individual .PCD file, loads it, and sends the points to the classifier saver
   * @arg thePath the PCD file to publish
   */
  void processFile(boost::filesystem::path thePath);
}; //PointCloudProcessor


#endif