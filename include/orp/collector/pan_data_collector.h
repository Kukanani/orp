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
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class PanDataCollector {
public:
	ros::NodeHandle n;  /// Standard ROS node handle.

	ros::Publisher panPub; ///Send pan messages to be read by the pan table
	ros::Subscriber /*Thunder*/cloudSub; ///Waits for point cloud data
	ros::Publisher centerPointPub; ///Spits out the center point
	ros::ServiceClient saveClient; ///calls the service to save data to file
	ros::ServiceServer dataCollectSrv; ///listens for service calls to kic things off
	
	tf::TransformListener tfListener; ///Used to get the center point

	/**
	 * The stored depth points, which are passed on to the point cloud processor by the main worker.
	 */
	sensor_msgs::PointCloud2 currentCloud;

	/**
	 * Constructor
	 */
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
}; //PanDataCollector

#endif // _PAN_DATA_COLLECTOR_H_