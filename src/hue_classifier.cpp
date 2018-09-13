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
}

void HueClassifier::cb_classify(sensor_msgs::PointCloud2 cloud)
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
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;

  int numClouds = clouds.size();
  if(!clouds.empty()) {
    int cloudCounter = 0;
    for(auto eachCloud = clouds.begin();
        eachCloud != clouds.end(); eachCloud++)
    {
      if(eachCloud->width < 3) {
        continue;
      }
      pcl::PointCloud<ORPPoint>::Ptr thisCluster(
          new pcl::PointCloud<ORPPoint>);
      pcl::fromROSMsg(*eachCloud, *thisCluster);

      std::string color = "unknown";
      M.release();
      M = cv::Mat(thisCluster->points.size(), 1, CV_8UC3, cv::Scalar(0,0,0));
      int i = 0;

      pcl::PointCloud<ORPPoint>::iterator point;
      float r=0, g=0, b=0;

      for(point = thisCluster->points.begin();
          point < thisCluster->points.end(); ++point, ++i)
      {
        r += point->r;
        g += point->g;
        b += point->b;
      }

      // std::cout << "M has " << eachCloud->width << " elements." << std::endl;

      color = getColor(r, g, b);

      orp::WorldObject thisObject;
      thisObject.label = "obj_" + color;
      thisObject.pose.header.frame_id = eachCloud->header.frame_id;


      Eigen::Vector4f clusterCentroid;
      pcl::compute3DCentroid(*thisCluster, clusterCentroid);

      Eigen::Affine3d finalPose;
      finalPose(0,3) = clusterCentroid(0);
      finalPose(1,3) = clusterCentroid(1);
      finalPose(2,3) = clusterCentroid(2);
      tf::poseEigenToMsg(finalPose, thisObject.pose.pose);

      thisObject.pose.pose.orientation.x = 0;
      thisObject.pose.pose.orientation.y = 0;
      thisObject.pose.pose.orientation.z = 0;
      thisObject.pose.pose.orientation.w = 1;

      thisObject.probability = 0.75;
      classRes.result.push_back(thisObject);
      cloudCounter++;
      // ROS_INFO_STREAM("processed cloud " << cloudCounter<< " of " << numClouds);
    }
  }
  // ROS_INFO_STREAM("Finished processing " << numClouds << " clouds");
  classification_pub_.publish(classRes);
}

inline uchar reduceVal(const uchar val)
{
  if (val < 64) return 0;
  if (val < 128) return 64;
  return 255;
}

void HueClassifier::processColors(cv::Mat& img)
{
  uchar* pixelPtr = img.data;
  for (int i = 0; i < img.rows; i++)
  {
    for (int j = 0; j < img.cols; j++)
    {
      const int pi = i*img.cols*3 + j*3;
      pixelPtr[pi + 0] = reduceVal(pixelPtr[pi + 0]); // B
      pixelPtr[pi + 1] = reduceVal(pixelPtr[pi + 1]); // G
      pixelPtr[pi + 2] = reduceVal(pixelPtr[pi + 2]); // R
      if(pixelPtr[pi+0] == 64) pixelPtr[pi+0] = 127;
      if(pixelPtr[pi+1] == 64) pixelPtr[pi+1] = 127;
      if(pixelPtr[pi+2] == 64) pixelPtr[pi+2] = 127;
    }
  }
}

std::string HueClassifier::getColor(float r, float g, float b)
{
  // TODO(Kukanani):
  // This function used to take a cv::Mat and use the cv::sum function to
  // calculate the r, g, and b values in one line each. However, this began
  // giving me segfaults after migrating to a new machine. I don't know the
  // exact issue, but I suspect a conflict with OpenCV versions 2 and 3.
  // Whatever the reason, you now have the pass the r/g/b values directly to
  // this function.

  // Eigen::Vector3f input_color(std::static_cast<float>(r),
  //                             std::static_cast<float>(g),
  //                             std::static_cast<float>(b));
  // input_color = input_color.normalized();

  // Eigen::Vector3f red(255, 0, 0);
  // Eigen::Vector3f green(0, 255, 0);
  // Eigen::Vector3f yellow(255, 255, 0);
  // Eigen::Vector3f blue(0, 0, 255);

  // std::vector<std::pair<Eigen::Vector3f, std::string>> colors;
  // colors.append(std::pair<Eigen::Vector3f, std::string>(red, "red"));
  // colors.append(std::pair<Eigen::Vector3f, std::string>(green, "green"));
  // colors.append(std::pair<Eigen::Vector3f, std::string>(blue, "blue"));
  // colors.append(std::pair<Eigen::Vector3f, std::string>(yellow, "yellow"));

  // double best_distance = 1e300;
  // std::string best_color = "null";
  // for(const auto& p: colors)
  // {
  //   double distance =
  //   if distance < best_distance
  //   {
  //     best_distance = distance;
  //     best_color = p.second;
  //   }
  // }
  // return best_color;

  // scale to interval 0, 1
  double input_max = std::max(std::max(r, g), b);
  double R = static_cast<double>(r)/input_max;
  double G = static_cast<double>(g)/input_max;
  double B = static_cast<double>(b)/input_max;

  double M = std::max(std::max(R, G), B);
  double m = std::min(std::min(R, G), B);
  double C = M - m;

  double H = 0;
  if(M == R)
  {
    H = std::fmod((G - B) / C, 6.0);
  }
  else if(M == G)
  {
    H = (B - R) / C + 2;
  }
  else
  {
    H = (R - G) + 4;
  }

  H = 60*H;

  if(H < 20 || H > 340)
  {
    return "red";
  }
  else if(H < 40)
  {
    return "orange";
  }
  else if(H < 90)
  {
    return "yellow";
  }
  else if(H < 180)
  {
    return "green";
  }
  else if (H < 270)
  {
    return "blue";
  }
  else
  {
    return "purple";
  }
}

///////////////////////////////
// Example of OpenCV-based point cloud filtering
//   pcl::PointCloud<ORPPoint>::Ptr pclCloud = pcl::PointCloud<ORPPoint>::Ptr(new pcl::PointCloud<ORPPoint>());
//   pcl::fromROSMsg(cloud, *pclCloud);
//
//   uint8_t* pixelPtr = (uint8_t*)cv_ptr->image.data;
//   int cn = cv_ptr->image.channels();
//   cv::Scalar_<uint8_t> bgrPixel;
//   int i= 0;
//   for (size_t u = 0; u < cloud.height; ++u)   // rows
//   {
//     for (size_t v = 0; v < cloud.width; ++v, ++i)  // cols
//     {
//       if(cv_ptr->image.at<cv::Vec3b>(u,v).val[0] > 128) { //blue channel check
//         pclCloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
//         pclCloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
//         pclCloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
//       }
//     }
//   }
//   pcl::toROSMsg(*pclCloud, cloud);
//   filterPub.publish(cloud);
//
// } //classify
//
//
// ///http://stackoverflow.com/questions/5906693/how-to-reduce-the-number-of-colors-in-an-image-with-opencv-in-python
// inline uchar reduceVal(const uchar val)
// {
//     if (val < 64) return 0;
//     if (val < 128) return 64;
//     return 255;
// }
//
// void OpenCVClassifier::process(cv::Mat& img)
// {
//     uchar* pixelPtr = img.data;
//     for (int i = 0; i < img.rows; i++)
//     {
//         for (int j = 0; j < img.cols; j++)
//         {
//             const int pi = i*img.cols*3 + j*3;
//             pixelPtr[pi + 0] = reduceVal(pixelPtr[pi + 0]); // B
//             pixelPtr[pi + 1] = reduceVal(pixelPtr[pi + 1]); // G
//             pixelPtr[pi + 2] = reduceVal(pixelPtr[pi + 2]); // R
//
//
//             if(pixelPtr[pi+0] == 64) pixelPtr[pi+0] = 127;
//             if(pixelPtr[pi+1] == 64) pixelPtr[pi+1] = 127;
//             if(pixelPtr[pi+2] == 64) pixelPtr[pi+2] = 127;
//         }
//     }
// }
