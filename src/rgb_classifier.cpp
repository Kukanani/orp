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
#include "orp/classifier/rgb_classifier.h"

#include <sstream>

int main(int argc, char **argv)
{
  // Start up the name and handle command-line arguments.
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "rgb_classifier");

  ROS_INFO("Starting RGB Classifier");
  RGBClassifier v;
  v.init();

  // for cluster visualization
  //cv::namedWindow( "RGBCluster", cv::WINDOW_NORMAL );
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  //cv::destroyAllWindows();
  return 1;
}

RGBClassifier::RGBClassifier():
  Classifier3D()
{
}

void RGBClassifier::cb_classify(sensor_msgs::PointCloud2 cloud)
{
  orp::ClassificationResult classRes;
  classRes.method = "rgb";

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  segmentation_client_.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;

  if(!clouds.empty()) {
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

      std::cout << "M has " << eachCloud->width << " elements." << std::endl;

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

      classRes.result.push_back(thisObject);
    }
  }
  classification_pub_.publish(classRes);
}

inline uchar reduceVal(const uchar val)
{
  if (val < 64) return 0;
  if (val < 128) return 64;
  return 255;
}

void RGBClassifier::processColors(cv::Mat& img)
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

std::string RGBClassifier::getColor(float r, float g, float b)
{
  // TODO(Kukanani):
  // This function used to take a cv::Mat and use the cv::sum function to
  // calculate the r, g, and b values in one line each. However, this began
  // giving me segfaults after migrating to a new machine. I don't know the
  // exact issue, but I suspect a conflict with OpenCV versions 2 and 3.
  // Whatever the reason, you now have the pass the r/g/b values directly to
  // this function.

  //which is greater?
  if(r > g && r > b) return "red";
  else if(g > r && g > b) return "green";
  return "blue";
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
