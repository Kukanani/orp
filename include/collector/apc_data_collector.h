// Copyright (c) 2016, Adam Allevato
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef _APC_DATA_COLLECTOR_H_
#define _APC_DATA_COLLECTOR_H_

#include <sstream>

#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <orp/DataCollect.h>
#include <orp/SaveCloud.h>

#include "core/orp_utils.h"

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