#ifndef _CPH_CLASSIFIER_H_
#define _CPH_CLASSIFIER_H_

//NRG internal files
#include "core/classifier.h"

#include "classifier/cph.h"

/**
 * @brief   Uses the CPH classifier to identify a point cloud from a list of known objects.
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
class CPHClassifier : public Classifier {
private:
  CPH cph; /// Used to calculate the CPH histograms

  int yBins; /// The number of horizontal bins/slices
  int rBins; /// The number of radial bins/pie slices
  /**
   * The number of bins in the overall histogram. = num_rbins*num_ybins+3
   * The extra three are for the spatial extents of the point cloud
   * (for scale invariance)
   */
  int cphSize;

public:
  /**
   * Constructor
   */
  CPHClassifier(std::string directory, bool _autostart);

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
}; //CPHClassifier

#endif //_CPH_CLASSIFIER_H_