#ifndef _SHELF_MANAGER_H_
#define _SHELF_MANAGER_H_

#include <string>

#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <orp/ChooseShelf.h>
#include <orp/ReloadParams.h>

#include "orp/core/orp_utils.h"

/**
 * @brief  sets segmentation parameters based on the shelf being viewed.
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date      3/31/2015
 */
class ShelfManager {
protected:
	ros::NodeHandle node;
	std::string shelf;
	ros::ServiceServer shelfServer;
public:
	ShelfManager();
	~ShelfManager();

	bool setShelf(std::string &shelfName);
	
	bool cb_setShelf(
		orp::ChooseShelf::Request &request,
	  orp::ChooseShelf::Response &response);

	void run();
};

#endif // _SHELF_MANAGER_H_