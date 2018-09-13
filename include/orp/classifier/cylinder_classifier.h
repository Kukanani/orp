// Copyright (c) 2016, Adam Allevato
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

#ifndef CYLINDER_CLASSIFIER_H
#define CYLINDER_CLASSIFIER_H

#include <eigen_conversions/eigen_msg.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

//NRG internal files
#include <orp/CylinderClassifierConfig.h>
#include "orp/core/classifier3d.h"
#include "orp/core/orp_utils.h"

/**
 * @brief   Cylinder-only classification - fits a cylinder to the dataset and
 *          returns the cylinder's position and orientation
 *
 * TODO(Kukanani): THIS IMPLEMENTATION IS INCOMPLETE.
 * The cylinder is infinitely long, so although the axis of the object will
 * probably be correct, you would need to adjust the position along the
 * cylinder's Z-axis based on the maximum and minimum spatial extents of the
 * classified point cloud, after projecting on to the object's Z-axis
 */
class CylinderClassifier : public Classifier3D {
protected:
  /// used to find the cylindrical shape
  pcl::SACSegmentationFromNormals<ORPPoint, pcl::Normal> seg;

  /// tuning parameter for the the cylinder-finding algorithm.
  double normalDistanceWeight;

  /// terminate after this many refinements
  double maxIterations;

  /// Not exactly sure what this does, perhaps points must be within this
  /// distance (m) to be considered as part of the cylinder?
  double distanceThreshold;

  /// minimum radius of cylinder model
  double minRadius;

  /// maximum radius of cylinder model
  double maxRadius;

  /// Enables usage of dynamic_reconfigure.
  dynamic_reconfigure::Server<orp::CylinderClassifierConfig> reconfigureServer;
  /// Used for dynamic reconfigure internals
  dynamic_reconfigure::Server<orp::CylinderClassifierConfig>::CallbackType
      reconfigureCallbackType;
public:
  /**
   * Basic constructor.
   */
  CylinderClassifier();

  /**
   * Dynamic reconfigure callback
   */
  void paramsChanged(orp::CylinderClassifierConfig &config, uint32_t level);

  /**
   * Takes the incoming point cloud and runs classification on it, passing
   * the result into the output topic.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  void cb_classify(const sensor_msgs::PointCloud2& cloud);
};

#endif