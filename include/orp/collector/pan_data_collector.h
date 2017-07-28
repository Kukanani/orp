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

#ifndef _PAN_DATA_COLLECTOR_H_
#define _PAN_DATA_COLLECTOR_H_

#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <orp/DataCollect.h>
#include <orp/SaveCloud.h>

/**
 * @brief Sends pan commands to the pan table
 * and pipes sensor point cloud data off to other topics
 *
 * @version 1.2
 * @ingroup objectrecognition
 * 
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 */
class PanDataCollector {
public:
	ros::NodeHandle n;  					/// Standard ROS node handle.

	ros::Publisher panPub; 					/// Send pan messages to be read by the pan table
	ros::Subscriber /*Thunder*/cloudSub; 	/// Waits for point cloud data
	ros::Publisher centerPointPub; 			/// Spits out the center point
	ros::ServiceClient saveClient; 			/// Calls the service to save data to file
	ros::ServiceServer dataCollectSrv; 		/// Listens for service calls to kic things off
	
	tf::TransformListener tfListener; 		/// Used to get the center point

	/**
	 * The stored depth points, which are passed on to the point cloud processor by the main worker.
	 */
	sensor_msgs::PointCloud2 currentCloud;

	PanDataCollector(ros::NodeHandle nh);

	/**
	 * Stores the incoming sensor data.
	 * @param cloud The point cloud from the sensor
	 */
	void cb_cloud(sensor_msgs::PointCloud2 cloud);

	/**
	 * Rotate the table 360 degrees in increments as specified by the message, and process the
	 * point cloud data at each point.
	 */
	bool rotate_cb(orp::DataCollect::Request &req,
		orp::DataCollect::Response &res);

	/**
	 * Get the center of the pan table, and publish that center to a ROS topic.
	 */
	void publishCenterPoint();

	/**
	 * Determine the pan table center just from a set of AR tags, and publish to ROS topic
	 */
	void publishCenterPointFromARTags();

};

#endif