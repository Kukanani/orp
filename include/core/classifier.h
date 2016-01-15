#ifndef _CLASSIFIER_H_
#define _CLASSIFIER_H_

#include <iostream>
#include <fstream>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/regex.hpp>
#include <boost/bind.hpp>

#include <flann/flann.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Empty.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <orp/Segmentation.h>
#include <orp/DetectionSet.h>

#include "world_object.h"
#include "core/orp_utils.h"

//http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)
typedef pcl::Histogram<90> CRH90;
POINT_CLOUD_REGISTER_POINT_STRUCT (CRH90, (float[90], histogram, histogram) )

struct KnownPose {
  std::string name;                /// The object name (types/classification)
  std::string dataName;            /// the original source of the data (filename)
  float angle;                     /// Rotation around vertical axis (for 4DOF pose)
  Eigen::Affine3d pose;            /// 6DOF pose
  pcl::PointCloud<CRH90>::Ptr crh; /// camera roll histogram as loaded from file
  Eigen::Vector4f centroid;        /// centroid of point cloud
  pcl::PointCloud<ORPPoint>::Ptr cloud; /// raw point cloud

  KnownPose() : cloud(new pcl::PointCloud<ORPPoint>), crh(new pcl::PointCloud<CRH90>) {};
};

//save the fingers
typedef std::pair<KnownPose, std::vector<float> > FeatureVector;
typedef std::vector<FeatureVector, Eigen::aligned_allocator<FeatureVector> > FeatureVectorVector;

/**
 * @brief   A generic classifier object.
 *
 * Given incoming point clouds, a classifier will generate a best guess
 * as to the result and publish that result to a topic. It is designed
 * to compare against feature vectors that have been previously saved
 * in a file, with a different feature vector for each object pose.
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @copyright BSD 3-paragraph
 * @date    Feb 24, 2015
 */
class Classifier {
protected:
  std::vector<std::string> fullTypeList;        /// All known objects that can be detected.
  std::vector<std::string> subTypeList;        /// Subset currently detectable.
  FeatureVectorVector loadedModels;      /// The list of models as loaded from files
  FeatureVectorVector subModels;         /// The subset of models for better FLANN searching

  flann::Index<flann::ChiSquareDistance<float> >* kIndex; /// The indexed data for knn search.
  flann::Matrix<int> kIndices;                  /// The indices for the knn search.
  flann::Matrix<float> kDistances;              /// The resulting distances for the knn search.
  flann::Matrix<float>* kData;                  /// The data for the knn search.

  bool autostart;                               /// If true, then classifier will automatically subscribe to depth data and begin producing classifications.
  float threshold;                              /// knn distance threshold
  std::string name;                             /// classifier type (cph, vfh, etc.)
  std::string dataFolder;                       /// folder to load feature vectors from
  std::string fileExtension;                    /// file extension to load feature vectors from.

  boost::mutex rosMutex;                        /// Protects ROS node interfaces

  ros::Subscriber startSub;                     /// to start recognition loop
  ros::Subscriber stopSub;                      /// to stop recognition loop
  ros::Subscriber detectionSetSub;              /// Listens for a subset of objects to detect.
  ros::Subscriber depthInfoSubscriber;          /// Collects depth camera point clouds
  ros::ServiceClient segmentationClient;        /// Calls the segmentation node
  ros::Publisher classificationPub;             /// Outputs classification results

  ros::NodeHandle n;

  /// set the subset of items to detect. This is a costly function, avoid calling it if possible.
  void setDetectionSet(std::vector<std::string> set);
  void cb_detectionSet(orp::DetectionSet msg);

  /** 
   * Search for the closest k neighbors
   * 
   * @param index the tree
   * @param model the query model
   * @param k the number of neighbors to search for
   * @param indices the resultant neighbor indices
   * @param distances the resultant neighbor distances
   *
   * @return number of matches found
   */
  virtual int nearestKSearch (
    flann::Index<flann::ChiSquareDistance<float> > &index,
    const FeatureVector &model, 
    int k,
    flann::Matrix<int> &indices,
    flann::Matrix<float> &distances);


  /**
   * Load multiple files wherever they're found in a specified directory.
   * @param base_dir  the directory to load files from
   * @param extension the file extension to search for
   * @param models    the list to populate with the files
   */
  virtual void loadModelsRecursive(
    const boost::filesystem::path &base_dir,
    const std::string &extension, 
    FeatureVectorVector &models);

  /**
   * Load one histogram from a file, as long as it matches the known list of objects.
   * @param  path path to the histogram
   * @param  cph  the model to fill with the data
   * @return      true, unless there was an error reading the file
   */
  virtual bool loadHist(const boost::filesystem::path &path, FeatureVector &vec) = 0;

  /**
   * Connect to all the appropriate topics for this classifier.
   * This is in its own method so that if no nodes are subscribed to this one, no
   * processing will take place. Necessary because vision is such a CPU-intensive
   * process.
   */
  void subscribe();

  /**
   * Undoes whatever is done in subscribe().
   */
  void unsubscribe();

public:
  /**
   * Constructor. Don't forget to call init() afterwards.
   */
  Classifier(
    float thresh,
    std::string _name,
    std::string dataFolder,
    std::string fileExt,
    bool autostart = false);

  Classifier(const Classifier& other);

  ~Classifier();

  /**
   * Call this to set up the classifier. This has to be its own method because otherwise
   * Classifier would call virtual methods in the constructor. See this page for more info:
   * http://www.parashift.com/c++-faq-lite/calling-virtuals-from-ctors.html
   */
  void init();

  /**
   * Takes the incoming point cloud and stores it.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  virtual void cb_classify(sensor_msgs::PointCloud2 cloud) = 0;
  virtual void cb_subscribe(std_msgs::Empty msg); 
  virtual void cb_unsubscribe(std_msgs::Empty msg);

}; //Classifier

#endif //_CLASSIFIER_H_
