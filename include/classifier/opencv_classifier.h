#ifndef _RGB_CLASSIFIER_H_
#define _RGB_CLASSIFIER_H_

#include <pcl/features/cvfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
//NRG internal files
#include "core/classifier.h"

/**
 * @brief   General OpenCV-based classifier using depth and color
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date      Oct 9, 2015
 */
class OpenCVClassifier : public Classifier {
protected:
  tf::TransformListener listener;
  
  ros::Publisher filterPub;

  cv::Mat M; ///For visualization
public:
  /**
   * Constructor
   */
  OpenCVClassifier(std::string directory, bool autostart = false);
  
  /**
   * Load one histogram from a file, as long as it matches the known list of objects.
   * @param  path path to the histogram
   * @param  vec  the model to fill with the data
   * @return      true, unless there was an error reading the file
   */
  virtual bool loadHist(const boost::filesystem::path &path, FeatureVector &vec);

  /**
   * Takes the incoming point cloud and runs classification on it, passing
   * the result into the output topic.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  void cb_classify(sensor_msgs::PointCloud2 cloud);

  /**
   * Process with OpenCV.
   */
  void process(cv::Mat& img);

  /**
   * Get a string name that represents the most common color in this image.
   */
  std::string getColor(cv::Mat& img);
};

#endif