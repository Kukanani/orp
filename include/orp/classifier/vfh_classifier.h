#ifndef _VFH_CLASSIFIER_H_
#define _VFH_CLASSIFIER_H_

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

//NRG internal files
#include "orp/core/classifier.h"

/**
 * @brief   Uses the VFH classifier to identify a point cloud from a list of known objects.
 *
 * @version 2.1
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @copyright BSD 3-paragraph
 * @date    Feb 3, 2015
 */
class VFHClassifier : public Classifier {
protected:
public:
  /**
   * Constructor
   */
  VFHClassifier(std::string directory);
  
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

}; //VFHClassifier

#endif //_VFH_CLASSIFIER_H_