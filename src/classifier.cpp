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

#include "orp/core/classifier.h"

Classifier::Classifier():
  node_private_("~"),
  node_("")
{
  //load configuration parameters and defaults
  node_private_.param<std::string>("classification_topic",
      classification_topic_, "classification");
  node_private_.param<std::string>("start_topic",
      start_topic_, "start_recognition");
  node_private_.param<std::string>("stop_topic",
      stop_topic_, "stop_recognition");

  start_sub_ = node_.subscribe(start_topic_, 1, &Classifier::cb_start, this);
  stop_sub_ = node_.subscribe(stop_topic_, 1, &Classifier::cb_stop, this);
}

void Classifier::init()
{
  //autostart
  bool autostart = true;
  node_.param<bool>("autostart", autostart, autostart);
  if(autostart) {
    ROS_INFO("Classifier is autostarting");
    start();
  }
  else {
  }
}

void Classifier::start()
{
  ROS_INFO("Classifier is starting");
  classification_pub_ =
      node_.advertise<orp::ClassificationResult>(classification_topic_, 10);
}

void Classifier::stop()
{
  if(classification_pub_ != NULL)
  {
    ROS_INFO("Classifier is shutting down");
    classification_pub_.shutdown();
  }
}

void Classifier::cb_start(std_msgs::Empty msg)
{
  start();
}

void Classifier::cb_stop(std_msgs::Empty msg)
{
  stop();
}