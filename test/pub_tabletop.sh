#!/bin/bash
rosrun pcl_ros pcd_to_pointcloud tabletop.pcd \
	/cloud_pcd:=/camera/depth_regd/points \
	_frame_id:=/base_linkrame_id:=/base_link