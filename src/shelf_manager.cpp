#include "orp/app/shelf_manager.h"

int main(int argc, char** argv){
  if (argc < 1) {
    ROS_ERROR("1 argument expected, %i arguments provided", argc);
    ROS_ERROR("syntax: shelf_manager");
    return -1;
  };

  ros::init(argc, argv, "shelf_manager");
  ShelfManager sm;
  sm.run();

  return 0;
};

ShelfManager::ShelfManager() : node("shelf_manager"), shelf("A"), shelfServer() {
  shelfServer = node.advertiseService("set_shelf", &ShelfManager::cb_setShelf, this);
}

ShelfManager::~ShelfManager() {
}

void ShelfManager::run() {
  ROS_INFO("Shelf Manager running...");
  ros::spin();
}

bool ShelfManager::setShelf(std::string &shelfName) {
  ROS_INFO("set shelf to %s", shelfName.c_str());
  boost::to_upper(shelfName);
  if(shelfName.length() != 1 || shelfName < "A" || shelfName > "Z") {
    ROS_ERROR("shelf %s not found", shelfName.c_str());
    throw std::runtime_error("desired shelf not found.");
  }
  shelf = shelfName;

  bool worked = true;
  try {
    worked &= ORPUtils::attemptToCopyFloatParam(node, "/shelf" + shelf + "_x_min", "/segmentation/spatial_min_x");
    worked &= ORPUtils::attemptToCopyFloatParam(node, "/shelf" + shelf + "_x_max", "/segmentation/spatial_max_x");
    worked &= ORPUtils::attemptToCopyFloatParam(node, "/shelf" + shelf + "_y_min", "/segmentation/spatial_min_y");
    worked &= ORPUtils::attemptToCopyFloatParam(node, "/shelf" + shelf + "_y_max", "/segmentation/spatial_max_y");
    worked &= ORPUtils::attemptToCopyFloatParam(node, "/shelf" + shelf + "_z_min", "/segmentation/spatial_min_z");
    worked &= ORPUtils::attemptToCopyFloatParam(node, "/shelf" + shelf + "_z_max", "/segmentation/spatial_max_z");
  } catch(std::exception e) {
    worked = false;
  }
  orp::ReloadParams reload;
  if(!ros::service::call("/reload_params", reload))
  {
    ROS_ERROR("reload_params service not available, segmentation maybe not running");
    throw std::runtime_error("reload_params service not available, segmentation maybe not running");
  }

  if(!worked) {
    ROS_ERROR("completed setting shelf, but some params could not be found");
    throw std::runtime_error("completed setting shelf, but some params could not be found");
  }
  return true;
}

bool ShelfManager::cb_setShelf(
    orp::ChooseShelf::Request &request,
    orp::ChooseShelf::Response &response) {
  return setShelf(request.shelf);
}