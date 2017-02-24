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

#ifndef _RGB_CLASSIFIER_H_
#define _RGB_CLASSIFIER_H_

#include <pcl/features/cvfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <pcl_ros/transforms.h>

//NRG internal files
#include "orp/core/classifier3d.h"

/**
 * @brief   RGB-only classification - posterizing colors into main color groups
 */
class RGBClassifier : public Classifier3D {
protected:
  cv::Mat M; ///For RGB cluster visualization
public:
  RGBClassifier();

  /**
   * Takes the incoming point cloud and runs classification on it, passing
   * the result into the output topic.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  void cb_classify(sensor_msgs::PointCloud2 cloud);

  /**
   * Posterize the colors in a cv Mat.
   * http://stackoverflow.com/questions/5906693/how-to-reduce-the-number-of-colors-in-an-image-with-opencv-in-python
   */
  void processColors(cv::Mat& img);

  /**
   * Get a string name that represents the most common color in this image.
   */
  std::string getColor(float r, float g, float b);
};

#endif