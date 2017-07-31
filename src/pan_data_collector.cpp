// Copyright (c) 2015, Adam Allevato
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

#include "orp/collector/pan_data_collector.h"

/**
 * Program entry point
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pan_data_collector");
  if(!ros::master::check()) { //uh-oh, ROS core not running
    std::cerr << "Oops, ROS isn't running. Please start roscore and try again."
              << std::endl;
    return -2;
  }

  ros::NodeHandle nh;
  PanDataCollector* pdc = new PanDataCollector(nh);

  ROS_INFO("Starting Pan-Tilt Table");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}; //main

PanDataCollector::PanDataCollector(ros::NodeHandle nh)  {
  this->n = nh;

  //make a service
  dataCollectSrv =
    n.advertiseService(
      "pan360_data_collect",
      &PanDataCollector::rotate_cb,
      this);

  saveClient = n.serviceClient<orp::SaveCloud>("/save_cloud");

  panPub    = n.advertise<std_msgs::Int32>("/pan_command",1);
  centerPointPub = n.advertise<geometry_msgs::Vector3>("/set_center_point",1);
  //listen to a topic
  cloudSub =
    n.subscribe(
      "/camera/depth_registered/points",
      10,
      &PanDataCollector::cb_cloud,
      this);


  ROS_INFO("Pan Data Collector initialized");
}

void PanDataCollector::cb_cloud(sensor_msgs::PointCloud2 cloud)
{
  currentCloud = cloud;
}

bool PanDataCollector::rotate_cb(orp::DataCollect::Request& req,
  orp::DataCollect::Response& res) {

  int pan_pos = 0;
  int delta = req.delta;

  std_msgs::Int32 panMessage;
  orp::SaveCloud saveRequest;
  saveRequest.request.objectName = req.objectName;
  ros::Rate loop_rate(.2);

  panMessage.data = req.delta;
  publishCenterPoint();

  while(pan_pos + delta <= 360)
  {
    ros::spinOnce();

    saveRequest.request.in_cloud = currentCloud;
    saveRequest.request.angle = pan_pos;
    saveClient.call(saveRequest);

    panPub.publish(panMessage);
    ROS_INFO_STREAM("Current angle: " << pan_pos << " degrees");
    loop_rate.sleep();
    pan_pos += delta;
  }
  return true;
}

void PanDataCollector::publishCenterPoint() {
  //This function used to use AR tags attached directly to the table surface.
  // But these can easily be occluded  by objects on the table, so we built a
  // new table that has the tags on the ground next to the table base. See
  // publishCenterPointFromARTags()

  // The function assumes that the pan table is centered in the world frame.
  // This just accounts for the physical height of the pan table, as measured
  // from the top of the ar tags (which are used to determine the calibration)
  geometry_msgs::Vector3 centerPoint;
  centerPoint.x = 0;
  centerPoint.y = 0.1317625; // height of table in m
  centerPoint.z = 0;
  centerPointPub.publish(centerPoint);

  ROS_INFO_STREAM("Published center point: " << centerPoint.x << ", "
      << centerPoint.y << ", " << centerPoint.z);
}

//UNUSED
void PanDataCollector::publishCenterPointFromARTags() {
  //use TF to get the center position
  //TODO: fix the AR tags to not be hard coded.
  tf::StampedTransform centerTransform;
  try {
    tfListener.lookupTransform("/world", "/ar_marker_16", ros::Time(0),
        centerTransform);
  } catch(tf::TransformException ex) {
    ROS_ERROR_STREAM("Pan data collector encountered error while trying to "
      << "get the table center point:" << ex.what());
    ROS_ERROR_STREAM("We're going to assume that the center isn't being "
      << "published, and use the camera point as the center point. This will "
      << "probably end badly. You need to put the AR tag fiducial on the pan "
      << "table.");
  }

  //send this center position to the topic that cares about it
  geometry_msgs::Vector3 centerPoint;
  centerPoint.x = centerTransform.getOrigin().x();
  centerPoint.y = centerTransform.getOrigin().y();
  centerPoint.z = centerTransform.getOrigin().z();
  centerPointPub.publish(centerPoint);
  ROS_INFO_STREAM("Published center point: " << centerPoint.x << ", "
      << centerPoint.y << ", " << centerPoint.z);
}