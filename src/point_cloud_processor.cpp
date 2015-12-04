#include "collector/point_cloud_processor.h"

/////////////////////////////////////////////////////////////////////////////////////////

/**
 * Program entry point
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_processor");
  if(!ros::master::check()) { //uh-oh, ROS core not running
    std::cerr << "Oops, ROS isn't running. Please start roscore and try again." << std::endl;
    return -2;
  }

  if(argc < 4) {
    ROS_ERROR("Usage is point_cloud_processor input_path output_directory object_name [bigbird]");
    return -1;
  }

  bool bigbird = true;

  if(argc > 4) {
    bigbird = atoi(argv[4]);
  }

  ROS_INFO("Starting APC point cloud processor");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  PointCloudProcessor* pdc = new PointCloudProcessor(nh, argv[1], argv[2], argv[3], bigbird);

  return 0;
} //main

/////////////////////////////////////////////////////////////////////////////////////////

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle nh, std::string inputPath, std::string outputDir, 
                                          std::string outputName, bool bb) :
  bigBird(bb)
{
  this->n = nh;
  boost::filesystem::path filename = inputPath;
  std::string inputFilename = filename.filename().string();
  if(inputPath.substr(0,4) != "NP1_" && bigBird) return;
  int angle = 0;

  int begin = inputFilename.rfind("_");
  int end = inputFilename.find(".");
  angle = atoi(inputFilename.substr(begin, end-begin).c_str());
  ROS_INFO("begin: %i, end: %i, angle: (%s) %i", begin, end, inputFilename.substr(begin, end-begin).c_str(), angle);

  //ROS_INFO("publishing file %s", theFile.string().c_str());

  pcl::PointCloud<ORPPoint>::Ptr cloud (new pcl::PointCloud<ORPPoint>);

  if (pcl::io::loadPCDFile<ORPPoint> (filename.string(), *cloud) == -1) //* load the file
  {
    ROS_ERROR ("Couldn't read file %s", filename.string().c_str());
    return;
  }

  //PROCESSING/////////////////////////////////////////////

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  float theta = M_PI/4;
  //theta = 0.1;

  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud (*cloud, *cloud, transform);














  //END PROCESSING/////////////////////////////////////////////

  std::stringstream fileName_ss;
  fileName_ss << outputDir << outputName.c_str() << "_" << angle << ".pcd";

  char thepath3[500];
  realpath(fileName_ss.str().c_str(), thepath3);

  //Write raw pcd file (objecName_angle.pcd)
  ROS_INFO_STREAM("writing raw cloud to file '" << thepath3 << "'");
  pcl::io::savePCDFile(thepath3, *cloud);
  ROS_DEBUG("done");
} //publishFile