///////////////////////////////////////////////////////////////////////////////
//      Title     : sia5-nrg
//      Project   : NRG ORP
//      Created   : 2/3/2015
//      Author    : Adam Allevato
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

#ifndef _VFH_CLASSIFIER_H_
#define _VFH_CLASSIFIER_H_

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

//NRG internal files
#include "core/classifier.h"

/**
 * @brief   Uses the VFH classifier to identify a point cloud from a list of known objects.
 *
 * @version 2.1
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
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