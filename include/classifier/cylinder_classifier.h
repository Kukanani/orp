#ifndef CYLINDER_CLASSIFIER_H
#define CYLINDER_CLASSIFIER_H

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <orp/CylinderClassifierConfig.h>

//NRG internal files
#include "core/classifier.h"

/**
 * @brief   Cylinder-only classification - posterizing colors into main color groups
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date      Oct 9, 2015
 */
class CylinderClassifier : public Classifier {
protected:
  pcl::SACSegmentationFromNormals<ORPPoint, pcl::Normal> seg; 
  
  double normalDistanceWeight, maxIterations, distanceThreshold,
    minRadius, maxRadius;
    
  /// Enables usage of dynamic_reconfigure.
  dynamic_reconfigure::Server<orp::CylinderClassifierConfig> reconfigureServer;
  dynamic_reconfigure::Server<orp::CylinderClassifierConfig>::CallbackType reconfigureCallbackType;
public:
  /**
   * Constructor
   */
  CylinderClassifier(bool autostart = false);
  void paramsChanged(orp::CylinderClassifierConfig &config, uint32_t level);
  
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
};

#endif