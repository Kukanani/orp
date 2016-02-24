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

#include "collector/apc_data_collector.h"

/////////////////////////////////////////////////////////////////////////////////////////

/**
 * Program entry point
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "apc_data_collector");
  if(!ros::master::check()) { //uh-oh, ROS core not running
    std::cerr << "Oops, ROS isn't running. Please start roscore and try again." << std::endl;
    return -2;
  }

  if(argc < 2) {
    ROS_ERROR("Usage is apc_data_collector directory_name [object name] [bigbird, 0 or 1]");
    return -1;
  }

  bool bigbird = true;

  std::string name = "";
  if(argc > 2) {
    name = argv[2];
  }
  if(argc > 3) {
    bigbird = atoi(argv[3]);
  }

  ROS_INFO("Starting APC Data collector");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  APCDataCollector* pdc = new APCDataCollector(nh, argv[1], name, bigbird);


  return 0;
} //main

/////////////////////////////////////////////////////////////////////////////////////////

APCDataCollector::APCDataCollector(ros::NodeHandle nh, std::string path, std::string objName,
                                    bool bb) :
  bigBird(bb)
{
  name = objName;
  this->n = nh;
  //call services
  histogram_client = n.serviceClient<orp::SaveCloud>("save_cloud");

  ROS_INFO("APC Data collector initialized. Publishing the clouds found in subdirectories of path"
    "%s", path.c_str());

  if(!boost::filesystem::is_directory (path)) {
    ROS_FATAL("target path is not a directory!");
    return;
  }
  if(!boost::filesystem::exists (path)) {
    ROS_FATAL("target folder does not exist");
    return;
  }
  loadModelsRecursive(path);

} //APCDataCollector

void APCDataCollector::loadModelsRecursive(std::string path) {
  std::string extension=".pcd";

  for(boost::filesystem::directory_iterator it (path); 
      it != boost::filesystem::directory_iterator (); ++it) {
    if(boost::filesystem::is_directory(it->status ())) {
      ROS_INFO("traversing into directory %s.", it->path().c_str());
      loadModelsRecursive(it->path().c_str());
    }
    if(boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path())
      == extension) {

      publishFile(*it);
    }
  }
} //loadModelsRecursive

void APCDataCollector::publishFile(boost::filesystem::path theFile)
{
  std::string filename = theFile.filename().string();
  std::string modelName = name;

  int angle = 0;

  int begin = filename.rfind("_")+1;
  int end = filename.find(".");
  angle = atoi(filename.substr(begin, end-begin).c_str());
  ROS_INFO("begin: %i, end: %i, angle: (%s) %i", begin, end, filename.substr(begin, end-begin).c_str(), angle);


  if(bigBird) {
    if(filename.substr(0,4) != "NP1_") return;
    boost::filesystem::path container = theFile.parent_path().parent_path();
    if(name == "") modelName = container.filename().string();
  } else {
    if(name == "") modelName = filename.substr(0, begin-1);
  }
  srv.request.objectName = modelName;

  //ROS_INFO("publishing file %s", theFile.string().c_str());

  pcl::PointCloud<ORPPoint>::Ptr cloud (new pcl::PointCloud<ORPPoint>);

  if (pcl::io::loadPCDFile<ORPPoint> (theFile.string(), *cloud) == -1) //* load the file
  {
    ROS_ERROR ("Couldn't read file %s", theFile.string().c_str());
    return;
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " << theFile.string().c_str() << " with " << cloud->points.size () << " points"
            << std::endl;

  pcl::toROSMsg(*cloud, srv.request.in_cloud);
  srv.request.in_cloud.header.frame_id = "camera_depth_optical_frame";
  ROS_INFO("message size %i x %i", srv.request.in_cloud.width, srv.request.in_cloud.height);
  srv.request.angle = angle;
  histogram_client.call(srv);
} //publishFile