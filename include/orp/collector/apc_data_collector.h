#ifndef _APC_DATA_COLLECTOR_H_
#define _APC_DATA_COLLECTOR_H_


#include <sstream>
#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "orp/DataCollect.h"
#include "orp/SaveCloud.h"

#include "orp/core/orp_utils.h"

/**
 * @brief Sends pan commands and pipes sensor point cloud data off to the SaveCloud service.
 *
 *
 * @version 1.1
 * @ingroup objectrecognition
 * 
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class APCDataCollector {
public:
  /**
   * Standard ROS node handle.
   */
  ros::NodeHandle n;

  bool bigBird;                         //true if using bigbird dataset, true by default.

  std::string name;                     //(optional)name attached for all objects

  ros::ServiceClient histogram_client;  ///Used to send point clouds off for file processing.
  orp::SaveCloud srv;              ///the message to send to the histogram client

  /**
   * Constructor
   * @arg nh the ros NodeHandle to use for everything.
   * @arg path the file path to recursively search for .PCD files.
   */
  APCDataCollector(ros::NodeHandle nh, std::string path, std::string name, bool bb);

  /**
   * Load the PCD models in the given folder, calling itself recursively on subfolders.
   * @param arg path the folder to search
   */
  void loadModelsRecursive(std::string path);

  /**
   * Takes an individual .PCD file, loads it, and sends the points to the classifier saver
   * @arg thePath the PCD file to publish
   */
  void publishFile(boost::filesystem::path thePath);
}; //APCDataCollector


#endif // _APC_DATA_COLLECTOR_H_