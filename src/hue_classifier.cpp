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

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>

#include "orp/core/orp_utils.h"
#include "orp/classifier/hue_classifier.h"
#include "orp/core/world_object.h"

#include <sstream>
#include <cmath>

int main(int argc, char **argv)
{
  // Start up the name and handle command-line arguments.
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "rgb_classifier");

  ROS_INFO("Starting Hue Classifier");
  HueClassifier v;
  v.init();

  // for cluster visualization
  //cv::namedWindow( "RGBCluster", cv::WINDOW_NORMAL );
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::waitForShutdown();
  //cv::destroyAllWindows();
  return 1;
}


HueClassifier::HueClassifier():
  Classifier3D()
{
  loadTypeList();
}

void HueClassifier::loadTypeList()
{
  XmlRpc::XmlRpcValue paramMap;
  while(!node_.getParam("items", paramMap)) {
    ROS_INFO_DELAYED_THROTTLE(5.0,
                      "Waiting for object type list on parameter server...");
    ros::Duration(1.0).sleep();
  }
  for(XmlRpc::XmlRpcValue::iterator it = paramMap.begin();
      it != paramMap.end(); ++it) {
    ROS_INFO_STREAM(it->first);
    ROS_INFO_STREAM(it->second);
    bool has_all = false, has_some = false;
    if(it->second.hasMember("hue_min") &&
       it->second.hasMember("hue_max") &&
       it->second.hasMember("sat_min") &&
       it->second.hasMember("sat_max") &&
       it->second.hasMember("val_min") &&
       it->second.hasMember("val_max"))
    {
      has_all = true;
    }
    if(it->second.hasMember("hue_min") ||
       it->second.hasMember("hue_max") ||
       it->second.hasMember("sat_min") ||
       it->second.hasMember("sat_max") ||
       it->second.hasMember("val_min") ||
       it->second.hasMember("val_max"))
    {
      has_some = true;
    }

    if(has_some && !has_all)
    {
      ROS_WARN_STREAM("Object " << it->first << " has some of the values " <<
                      "needed for HSV object classification, but not all " <<
                      "of them. You must provide hue_min, hue_max, " <<
                      "sat_min, sat_max, val_min, and val_max.");
    }
    if(has_all)
    {
      obj_hsvs.push_back(ObjHsv(
        it->first,
        static_cast<double>(it->second["hue_min"]),
        static_cast<double>(it->second["hue_max"]),
        static_cast<double>(it->second["sat_min"]),
        static_cast<double>(it->second["sat_max"]),
        static_cast<double>(it->second["val_min"]),
        static_cast<double>(it->second["val_max"])
        ));
    }

  }
}

void HueClassifier::cb_classify(const sensor_msgs::PointCloud2& cloud)
{
  ROS_DEBUG_NAMED("Hue Classfiier",
      "Received point cloud to classify objects by hue");

  orp::ClassificationResult classRes;
  classRes.method = "hsv";

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;

  {
    std::lock_guard<std::mutex> lock(segmentation_mutex_);
    bool segmentation_succeeded = segmentation_client_.call(seg_srv);
    if(!segmentation_succeeded)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(5, "Hue Classifier", "Could not call segmentation service at " << segmentation_service_);
    }
  }
  std::vector<sensor_msgs::PointCloud2> cloudMessages = seg_srv.response.clusters;

  int numClouds = cloudMessages.size();
  if(!cloudMessages.empty()) {
    int cloudCounter = 0;
    for(const auto& cloudMessage : cloudMessages)
    {
      if(cloudMessage.width < 3) {
        continue;
      }
      pcl::PointCloud<ORPPoint>::Ptr cloudPtr(
          new pcl::PointCloud<ORPPoint>);
      pcl::fromROSMsg(cloudMessage, *cloudPtr);

      std::string color = "unknown";
      cv::Mat M = cv::Mat(cloudPtr->points.size(), 1, CV_8UC3);

      int i=0;
      for(const auto& point : cloudPtr->points)
      {
        M.at<cv::Vec3b>(0, i) =
          cv::Vec3b(static_cast<uint8_t>(point.b), static_cast<uint8_t>(point.g), static_cast<uint8_t>(point.r));
        i++;
      }

      color = getClassByColor(M);
      if(color == "") {
        // no detection!
        continue;
      }

      orp::WorldObject thisObject;
      thisObject.label = color;

      // Now set the object pose: the center of the top of the pointcloud AABB
      float maxX = -1e300, minX = 1e300;
      float maxY = -1e300, minY = 1e300;
      float maxZ = -1e300, minZ = 1e300;
      for(const auto& pt : cloudPtr->points)
      {
        maxX = std::max(maxX, pt.x);
        minX = std::min(minX, pt.x);
        maxY = std::max(maxY, pt.y);
        minY = std::min(minY, pt.y);
        maxZ = std::max(maxZ, pt.z);
        minZ = std::min(minZ, pt.z);
      }

      thisObject.pose.pose.position.x = minX + (maxX - minX) / 2;
      // thisObject.pose.pose.position.y = minY + (maxY - minY) / 2;
      thisObject.pose.pose.position.y = maxY;
      thisObject.pose.pose.position.z = maxZ;

      thisObject.pose.pose.orientation.x = 0;
      thisObject.pose.pose.orientation.y = 0;
      thisObject.pose.pose.orientation.z = 0;
      thisObject.pose.pose.orientation.w = 1;
      thisObject.pose.header.frame_id = cloudMessage.header.frame_id;

      thisObject.probability = 0.75;
      classRes.result.push_back(thisObject);
      cloudCounter++;
      ROS_DEBUG_STREAM("processed cloud " << cloudCounter<< " of " << numClouds);
    }
  }
  ROS_DEBUG_STREAM("Finished processing " << numClouds << " clouds");
  if(classification_pub_ != NULL)
  {
    ROS_DEBUG_STREAM("publishing");
    classification_pub_.publish(classRes);
  }
}


std::string HueClassifier::getClassByColor(const cv::Mat& input)
{
  cv::Mat hsv = cv::Mat::zeros(1, 1, CV_8UC3);
  cv::Scalar avg_bgr = cv::mean(input);
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(avg_bgr[0], avg_bgr[1], avg_bgr[2]);
  cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);
  ROS_DEBUG_STREAM("raw average HSV: " << hsv.at<cv::Vec3b>(0, 0));
  cv::Vec3b hsv_raw = hsv.at<cv::Vec3b>(0, 0);

  // Hue goes into range 0-360
  int h = static_cast<int>(hsv_raw[0] * 2);

  // Saturation goes into range 0-100
  int s = static_cast<int>(hsv_raw[1] * 0.3921568);

  // Value goes into range 0-100
  int v = static_cast<int>(hsv_raw[2] * 0.3921568);

  ROS_DEBUG_STREAM("" << h << ", " << s << ", " << v);

  // now go through each HSV range known by the classifier and check it
  for(const auto& obj_hsv : obj_hsvs)
  {
    ROS_DEBUG_STREAM("" << obj_hsv.min_h << ", " << obj_hsv.max_h << ", " <<
      obj_hsv.min_s << ", " << obj_hsv.max_s << ", " << obj_hsv.min_v <<
      ", " << obj_hsv.max_v);

    // Hue can wrap around over 360, so we have to check two conditions
    bool h_in_range = false;
    if(obj_hsv.max_h <= 360)
    {
      // a basic range from 0-360
      h_in_range = h > obj_hsv.min_h && h <= obj_hsv.max_h;
    } else
    {
      // a range that wraps around
      h_in_range = h < obj_hsv.max_h-360 || h > obj_hsv.min_h;
    }
    if(h_in_range)
    {
      ROS_DEBUG_STREAM("Hue in range for " << obj_hsv.name);
    }
    // Check that we are in all ranges
    if(h_in_range &&
       s > obj_hsv.min_s && s <= obj_hsv.max_s &&
       v > obj_hsv.min_v && v <= obj_hsv.max_v)
    {
      // done!
      ROS_DEBUG_STREAM("Object class detected as: " << obj_hsv.name);
      return obj_hsv.name;
    }
  }

  // if no desired color is found, don't detect the object
  return "";
}

// Old version
// std::string HueClassifier::getClassByColor(int r, int g, int b)
// {

//   // scale to interval 0, 1
//   double input_max = std::max(std::max(r, g), b);
//   double R = static_cast<double>(r)/input_max;
//   double G = static_cast<double>(g)/input_max;
//   double B = static_cast<double>(b)/input_max;

//   double M = std::max(std::max(R, G), B);
//   double m = std::min(std::min(R, G), B);
//   double C = M - m;

//   double H = 0;
//   if(M == R)
//   {
//     H = std::fmod((G - B) / C, 6.0);
//   }
//   else if(M == G)
//   {
//     H = (B - R) / C + 2;
//   }
//   else
//   {
//     H = (R - G) + 4;
//   }

//   H = 60*H;

//   if(H < 20 || H > 340)
//   {
//     return "red";
//   }
//   else if(H < 40)
//   {
//     return "orange";
//   }
//   else if(H < 90)
//   {
//     return "yellow";
//   }
//   else if(H < 180)
//   {
//     return "green";
//   }
//   else if (H < 270)
//   {
//     return "blue";
//   }
//   else
//   {
//     return "purple";
//   }
// }
