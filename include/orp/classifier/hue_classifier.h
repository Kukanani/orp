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

#ifndef _HUE_CLASSIFIER_H_
#define _HUE_CLASSIFIER_H_

#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl_ros/transforms.h>

//NRG internal files
#include "orp/core/classifier3d.h"

#include <mutex>


/**
 * A range of valid HSV values and the object name associated with that range.
 */
struct ObjHsv {
  std::string name;
  double min_h, max_h, min_s, max_s, min_v, max_v;
  ObjHsv(std::string name, double min_h, double max_h, double min_s, double max_s, double min_v, double max_v) :
    name(name),
    min_h(min_h),
    max_h(max_h),
    min_s(min_s),
    max_s(max_s),
    min_v(min_v),
    max_v(max_v)
  {}

};

/**
 * Hue-based classification - posterizing colors into red, green,
 * blue, yellow, orange, and purple
 */
class HueClassifier : public Classifier3D {
protected:

  /**
   * Load the list of object types from the parameter server.
   * This function looks for "hue_min", "hue_max", "sat_min", etc.
   * for each object on the parameter server and stores it internally.
   */
  void loadTypeList();

  std::mutex segmentation_mutex_;

  std::vector<ObjHsv> obj_hsvs;
public:
  HueClassifier();

  /**
   * Takes the incoming point cloud and runs classification on it, passing
   * the result into the output topic.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  void cb_classify(const sensor_msgs::PointCloud2& cloud);

  /**
   * Get a string name that represents the most common color in this image.
   */
  std::string getClassByColor(const cv::Mat& cloud);
};

#endif
