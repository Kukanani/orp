#ifndef _ICP_CLASSIFIER_H_
#define _ICP_CLASSIFIER_H_

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/regex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

//NRG internal files
#include "orp/core/world_object.h"
#include "orp/core/orp_utils.h"

#include "orp/Segmentation.h"

typedef std::pair<std::string, pcl::PointCloud<ORPPoint>::Ptr > ICPModel;

/**
 * @brief   Uses the ICP method to attempt to match a known 3D model to a point cloud.
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date      Mar 27, 2015
 */
class ICPClassifier {
protected:
  std::vector<std::string> testingSubset;
  std::vector<ICPModel> knownObjects;
  std::vector<std::string> typeList;            /// All known objects that can be detected.

  ros::Subscriber depthInfoSubscriber;          /// Collects depth camera point clouds
  ros::Publisher classificationPub;             /// Outputs classification result
  ros::ServiceClient segmentationClient;        /// Calls the segmentation node
  ros::NodeHandle n;
  tf::TransformListener transformListener;

  float threshold;                              /// knn distance threshold
  std::string name;                             /// classifier type (cph, vfh, etc.)
  std::string dataFolder;                       /// folder to load feature vectors from
  std::string path;                             /// path to list file
  std::string fileExtension;                    /// file extension to load feature vectors from.

public:
  /**
   * Constructor
   */
  ICPClassifier(ros::NodeHandle nh, std::string directory, std::string path);
  ICPClassifier(const ICPClassifier& rhs);
  ~ICPClassifier();


  void init();


  void subscribe();
  void unsubscribe();

  void loadModelsRecursive(
    const boost::filesystem::path &base_dir,
    const std::string &extension);

  /**
   * Takes the incoming point cloud and runs classification on it, passing
   * the result into the output topic.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  void cb_classify(sensor_msgs::PointCloud2 cloud);

}; //VFHClassifier

#endif //_VFH_CLASSIFIER_H_