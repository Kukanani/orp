#!/bin/bash
rosrun pcl_ros pcd_to_pointcloud tabletop.pcd \
	/cloud_pcd:=/camera/depth_registered/points \
	_frame_id:=/camera_link