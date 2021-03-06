// Copyright (c) 2017, Adam Allevato
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

#ifndef _CLASSIFIER_2D_H_
#define _CLASSIFIER_2D_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "orp/core/classifier.h"

/**
 * @brief A 2D classifier
 *
 * Extension of the basic classifier, but includes an image subscriber.
 */
class Classifier2D : public Classifier {
protected:
  /// Where to listen for images
  std::string image_topic_;
  /// Listens for incoming images
  image_transport::Subscriber image_sub_;
  /// Generates image subscriptions
  image_transport::ImageTransport image_transport_;
public:
  /**
   * Constructor. Don't forget to call init() afterwards.
   */
  Classifier2D();

  /**
   * callback for whenever an image is received. Implementations of this
   * method should emit a Classification message to some topic.
   *
   * @param input_image the image to generate a classification from.
   */
  virtual void cb_classify(const sensor_msgs::ImageConstPtr& input_image) = 0;

  /**
   * Start listening to images
   */
  virtual void start();
  /**
   * Stop listening for images
   */
  virtual void stop();
};

#endif
