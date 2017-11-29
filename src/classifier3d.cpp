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

#include "orp/core/classifier3d.h"
#include "orp/core/world_object.h"
#include "orp/core/orp_utils.h"

Classifier3D::Classifier3D():
  Classifier()
{
  // allow remapping to different segmentation service
  node_private_.param<std::string>("segmentation_service",
      segmentation_service_, "segmentation/segmentation");
  segmentation_client_ =
      node_.serviceClient<orp::Segmentation>(segmentation_service_);

  // allow remapping to different depth cloud topic, but by default
  // use the default camera's point cloud
  node_private_.param<std::string>("depth_topic", depth_topic_,
    "/camera/depth_registered/points");
}

void Classifier3D::start()
{
  Classifier::start();
  depth_sub_ = node_.subscribe(depth_topic_, 10,
      &Classifier3D::cb_classify, this);
}

void Classifier3D::stop()
{
  Classifier::start();
  if(depth_sub_ != NULL)
  {
    depth_sub_.shutdown();
  }
}