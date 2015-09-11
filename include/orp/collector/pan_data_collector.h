#ifndef _PAN_DATA_COLLECTOR_H_
#define _PAN_DATA_COLLECTOR_H_


#include <sstream>

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include "orp/DataCollect.h"   
#include "orp/SaveCloud.h" 

/**
 * @brief Sends pan commands and pipes sensor point cloud data off to the SaveCloud service.
 *
 *
 * @version 1.1
 * @ingroup objectrecognition
 * 
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class PanDataCollector {
public:
	/**
	 * Standard ROS node handle.
	 */
	ros::NodeHandle n;
	/**
	 * Used to publish pan commands, which control the pan/tilt table.
	 */
	ros::Publisher panCommandPublisher;
	ros::Subscriber pan_sub;

	/**
	 * Used to call the pan service.
	 *   Not sure if both pan_client and panCommandPublisher are required for full functionality.
	 */
	ros::ServiceClient pan_client;
	ros::ServiceServer pan_serv;
	/**
	 * The stored depth points, which are passed on to the point cloud processor by the main worker.
	 */
	sensor_msgs::PointCloud2 cloud_to_process;

	/**
	 * Constructor
	 */
	PanDataCollector(ros::NodeHandle nh);

	/**
	 * Stores the incoming sensor data.
	 * @param fromKinect The point cloud from the sensor
	 */
	void depthPoints_cb(sensor_msgs::PointCloud2 fromKinect);

	/**
	 * Rotate the table 360 degrees in increments as specified by the message, and process the
	 * point cloud data at each point.
	 * 
	 * @param req [description]
	 * @param res [description]
	 */
	bool rotate_cb(orp::DataCollect::Request &req,
		orp::DataCollect::Response &res);
}; //PanDataCollector


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
	ros::spin();

	return 0;
}; //main

#endif // _PAN_DATA_COLLECTOR_H_