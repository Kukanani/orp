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

#include "collector/point_cloud_processor.h"

/////////////////////////////////////////////////////////////////////////////////////////

/**
 * Program entry point
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_processor");
  if(!ros::master::check()) { //uh-oh, ROS core not running
    std::cerr << "Oops, ROS isn't running. Please start roscore and try again." << std::endl;
    return -2;
  }

  if(argc < 4) {
    ROS_ERROR("Usage is point_cloud_processor input_path output_directory object_name [bigbird]");
    return -1;
  }

  bool bigbird = true;

  if(argc > 4) {
    bigbird = atoi(argv[4]);
  }

  ROS_INFO("Starting APC point cloud processor");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  PointCloudProcessor* pdc = new PointCloudProcessor(nh, argv[1], argv[2], argv[3], bigbird);

  return 0;
} //main

/////////////////////////////////////////////////////////////////////////////////////////

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle nh, std::string inputPath, std::string outputDir, 
                                          std::string outputName, bool bb) :
  bigBird(bb)
{
  this->n = nh;
  boost::filesystem::path filename = inputPath;
  std::string inputFilename = filename.filename().string();
  if(inputPath.substr(0,4) != "NP1_" && bigBird) return;
  int angle = 0;

  int begin = inputFilename.rfind("_");
  int end = inputFilename.find(".");
  angle = atoi(inputFilename.substr(begin, end-begin).c_str());
  ROS_INFO("begin: %i, end: %i, angle: (%s) %i", begin, end, inputFilename.substr(begin, end-begin).c_str(), angle);

  //ROS_INFO("publishing file %s", theFile.string().c_str());

  pcl::PointCloud<ORPPoint>::Ptr cloud (new pcl::PointCloud<ORPPoint>);

  if (pcl::io::loadPCDFile<ORPPoint> (filename.string(), *cloud) == -1) //* load the file
  {
    ROS_ERROR ("Couldn't read file %s", filename.string().c_str());
    return;
  }

  //PROCESSING/////////////////////////////////////////////


  //stuff for APC
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  float theta = M_PI/4;
  //theta = 0.1;

  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud (*cloud, *cloud, transform);














  //END PROCESSING/////////////////////////////////////////////

  std::stringstream fileName_ss;
  fileName_ss << outputDir << outputName.c_str() << "_" << angle << ".pcd";

  char thepath3[500];
  realpath(fileName_ss.str().c_str(), thepath3);

  //Write raw pcd file (objecName_angle.pcd)
  ROS_INFO_STREAM("writing raw cloud to file '" << thepath3 << "'");
  pcl::io::savePCDFile(thepath3, *cloud);
  ROS_DEBUG("done");
} //publishFile