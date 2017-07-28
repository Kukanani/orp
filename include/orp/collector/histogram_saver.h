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
 */
class HistogramSaver {
private:
  /**
   * Standard ROS node handle.
   */
  ros::NodeHandle n;

  ros::ServiceClient segClient;         /// Calls segmentation
  ros::ServiceServer saveCloudSrv;      /// Handles requests to save files

  ros::Subscriber tableCenterPointSub;  /// Listens for the centerpoint of the training table, used to set object origins

  //where to save histogram-based files
  std::string outDir;

  //method flags
  bool savePCD, saveCVFH, save6DOF;

  //CVFH options
  float cvfhRadiusSearch;
  Eigen::Vector4f tableCenterPoint;   ///Used to calculate distance between point cloud center and object center.

  std::vector<float> feature;         ///Gets filled with classifier data

  /// Dynamic reconfigure
  dynamic_reconfigure::Server<orp::HistogramSaverConfig> reconfigureServer;
  dynamic_reconfigure::Server<orp::HistogramSaverConfig>::CallbackType reconfigureCallbackType;
  void paramsChanged(orp::HistogramSaverConfig &config, uint32_t level);


  /// Write the cloud as a PCD file
  void writeRawCloud(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);

  /// Improved VFH descriptor
  void writeCVFH(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle);

  /// CVFH descriptor, plus CRH (roll histogram) and object centroid to allow inference of 6DOF pose
  void write6DOF(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int num);

  /// Used to calc object origins
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

};

#endif