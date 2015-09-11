#include "orp/collector/pan_data_collector.h"

PanDataCollector::PanDataCollector(ros::NodeHandle nh)  {
	this->n = nh;

	//make a service
	pan_serv = 
		n.advertiseService(
			"pan360_data_collect",
			&PanDataCollector::rotate_cb,
			this);
	//call services
	pan_client = n.serviceClient<orp::SaveCloud>("save_cloud");

	//make a topic
	panCommandPublisher    = n.advertise<std_msgs::Int32>("/pan_command",1);
	//listen to a topic
	pan_sub =
		n.subscribe(
			"/camera/depth_registered/points",
			10,
			&PanDataCollector::depthPoints_cb,
			this);

	ROS_INFO("Pan Data Collector initialized");
}

void PanDataCollector::depthPoints_cb(sensor_msgs::PointCloud2 fromKinect)
{
	cloud_to_process = fromKinect;
}

bool PanDataCollector::rotate_cb(orp::DataCollect::Request& req,
	orp::DataCollect::Response& res) {
	std_msgs::Int32 command;
	command.data = 0;
	orp::SaveCloud srv;

	srv.request.objectName = req.objectName;

	while(command.data < 360)
	{
		ros::Rate loop_rate(.2);

		srv.request.in_cloud = cloud_to_process;
		srv.request.angle = command.data;
		pan_client.call(srv);

		panCommandPublisher.publish(command);
		ROS_INFO("Angle: %lfdeg", srv.request.angle);
		ros::spinOnce();
		loop_rate.sleep();
		command.data += req.delta;
	}

	res.result = 1;
	ROS_INFO("Status: %d", srv.response.result);
	return true;
}