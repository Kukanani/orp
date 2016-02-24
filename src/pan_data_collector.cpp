///////////////////////////////////////////////////////////////////////////////
//      Title     : pan data collector
//      Project   : NRG ORP
//      Created   : 1/21/2015
//      Author    : Adam Allevato
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include "collector/pan_data_collector.h"

/**
 * Program entry point
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pan_data_collector");
	if(!ros::master::check()) { //uh-oh, ROS core not running
		std::cerr << "Oops, ROS isn't running. Please start roscore and try again." << std::endl;
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
	std_msgs::Int32 panPosition;
	orp::SaveCloud srv;
	srv.request.objectName = req.objectName;
	ros::Rate loop_rate(.2);

	panPosition.data = 0;
	ROS_INFO("moving pan table to home position");
	panPub.publish(panPosition);
	ros::Duration(5.0f).sleep();

	publishCenterPoint();

	while(panPosition.data < 360)
	{
		ros::spinOnce();

		srv.request.in_cloud = currentCloud;
		srv.request.angle = panPosition.data;
		saveClient.call(srv);

		panPub.publish(panPosition);
		ROS_INFO("Angle: %lfdeg", srv.request.angle);
		loop_rate.sleep();
		panPosition.data += req.delta;
	}
	return true;
}

void PanDataCollector::publishCenterPoint() {
	//use TF to get the center position
	//TODO: fix the frame names to be meaningful/allow the user to set them.
	tf::StampedTransform centerTransform;
	try {
		tfListener.lookupTransform("/world", "/ar_marker_16", ros::Time(0), centerTransform);
	} catch(tf::TransformException ex) {
		ROS_ERROR_STREAM("Pan data collector encountered error while trying to get the table center point:" <<
			ex.what());
		ROS_ERROR_STREAM("We're going to assume that the center isn't being published, and use the camera point" <<
			"as the center point. This will probably end badly. You need to put the AR tag fiducial on the pan table.");
	}

	//send this center position to the topic that cares about it
	geometry_msgs::Vector3 centerPoint;
	centerPoint.x = centerTransform.getOrigin().x();
	centerPoint.y = centerTransform.getOrigin().y();
	centerPoint.z = centerTransform.getOrigin().z();
	centerPointPub.publish(centerPoint);
	ROS_INFO_STREAM("Published center point: " << centerPoint.x << ", " << centerPoint.y << ", " << centerPoint.z);
}