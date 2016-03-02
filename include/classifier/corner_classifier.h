///////////////////////////////////////////////////////////////////////////////
//      Title     : corner_classifier
//      Project   : NRG ORP
//      Created   : 2/29/2016
//      Author    : Meredith Pitsch
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//                 
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CORNER_CLASSIFIER_H
#define CORNER_CLASSIFIER_H

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>


#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <orp/CornerClassifierConfig.h>

//NRG internal files
#include "core/classifier.h"
#include "classifier/intersections.h"

typedef pcl::PointCloud<ORPPoint> PC;
typedef pcl::PointCloud<ORPPoint>::Ptr PCP;

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
class CornerClassifier : public Classifier {
protected:
  pcl::SACSegmentation<ORPPoint> seg; 
  
  double  maxIterations, distanceThreshold;
    
  /// Enables usage of dynamic_reconfigure.
  dynamic_reconfigure::Server<orp::CornerClassifierConfig> reconfigureServer;
  dynamic_reconfigure::Server<orp::CornerClassifierConfig>::CallbackType reconfigureCallbackType;
public:
  /**
   * Constructor
   */
  CornerClassifier(bool autostart = false);
  void paramsChanged(orp::CornerClassifierConfig &config, uint32_t level);
  
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
