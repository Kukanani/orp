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

#ifndef _CLASSIFIER_H_
#define _CLASSIFIER_H_

#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <orp/ClassificationResult.h>

class Classifier {
protected:
  /// Interface to ROS
  ros::NodeHandle node_;
  ros::NodeHandle node_private_;

  /// Publishes completed classifications
  std::string classification_topic_;
  ros::Publisher classification_pub_;

  /// Listens for requests to start recognition
  std::string start_topic_;
  ros::Subscriber start_sub_;

  /// Listens for requests to stop recognition
  std::string stop_topic_;
  ros::Subscriber stop_sub_;


  /// subscribe to image messages and start publishing classifications
  virtual void start();
  /// unsubscribe from image messages and stop publishing classifications
  virtual void stop();

  /// ROS shadow for start()
  void cb_start(std_msgs::Empty msg);

  /// ROS shadow for stop()
  void cb_stop(std_msgs::Empty msg);

public:
  Classifier();

  /**
   * If the autostart argument is true, then start. Call at the end of the constructor.
   * This has to be its own method because otherwise
   * Classifier would call virtual methods in the constructor. See this page for more info:
   * http://www.parashift.com/c++-faq-lite/calling-virtuals-from-ctors.html
   */
  virtual void init();
};

#endif