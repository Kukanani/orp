// Copyright (c) 2015, Adam Allevato
// Copyright (c) 2017, The University of Texas at Austin
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _NN_CLASSIFIER_H_
#define _NN_CLASSIFIER_H_

#include <iostream>
#include <fstream>
#include <memory>

#include <flann/flann.h>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include <orp/Segmentation.h>
#include <orp/ClassificationResult.h>

#include "orp/core/classifier3d.h"
#include "orp/core/world_object.h"
#include "orp/core/orp_utils.h"

/**
 * An NN Classifier is a Nearest Neighbor classifier - that is, it uses a
 * descriptor that can be saved to a file, loads a bunch of those descriptors
 * from a file, and then does nearest-neighbor matching with FLANN to do
 * joint inference of object class and pose. This is the approach taken by
 * Brian O'Neil's thesis, using the CPH object descriptor.
 *
 * Much of this class, especially the part about CRH (Camera Roll Histogram,
 * note the difference from CPH, Circular Projection Histogram), was inspired
 * by the web page:
 * http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)
 */

/**
 * A Camera Roll Histogram. See http://pointclouds.org/news/2011/08/04/cvfh/
 * for a brief description. See also
 * http://ieeexplore.ieee.org/document/6630859/
 */
typedef pcl::Histogram<90> CRH90;
POINT_CLOUD_REGISTER_POINT_STRUCT (CRH90, (float[90], histogram, histogram) )

/**
 * This represents a single pose that will be associated with a descriptor,
 * usually as loaded from a file.
 */
struct KnownPose {
  /// The object name (types/classification)
  std::string name;
  /// the original source of the data (filename)
  std::string dataName;
  /// Rotation around vertical axis (for 4DOF pose)
  float angle;
  /// 6DOF pose
  Eigen::Affine3d pose;
  /// camera roll histogram as loaded from file (if this descriptor type
  /// supports CRH)
  /// TODO(Kukanani): maybe make this a union?
  pcl::PointCloud<CRH90>::Ptr crh;
  /// centroid of point cloud
  Eigen::Vector4f centroid;
  /// raw point cloud
  pcl::PointCloud<ORPPoint>::Ptr cloud;

  /**
   * TODO(Kukanani): structs don't usually have constructors, do they? Maybe
   * make this a class.
   */
  KnownPose() :
    cloud(new pcl::PointCloud<ORPPoint>),
    crh(new pcl::PointCloud<CRH90>) {};
};

/// A known pose and its descriptor
typedef std::pair<KnownPose, std::vector<float> > FeatureVector;

/// Multiple feature vectors. Cool name bro.
typedef std::vector<FeatureVector, Eigen::aligned_allocator<FeatureVector> >
    FeatureVectorVector;

/**
 * @brief   A generic nearest-neighbor classifier object.
 *
 * Given incoming point clouds, a classifier will generate a best guess
 * as to the result by matching to a nearest neighbor via FLANN.
 * It will then publish that result to a topic. It is designed
 * to compare against feature vectors that have been previously saved
 * in a file, with a different feature vector for each object pose.
 */
class NNClassifier : public Classifier3D {
protected:
  /// All known objects that can be detected.
  std::vector<std::string> fullTypeList;
  /// Subset currently detectable.
  std::vector<std::string> subTypeList;
  /// The list of models as loaded from files
  FeatureVectorVector loadedModels;
  /// The subset of models for better FLANN searching
  FeatureVectorVector subModels;

  /// The indexed data for knn search.
  flann::Index<flann::ChiSquareDistance<float> >* kIndex;
  /// The indices for the knn search.
  flann::Matrix<int> kIndices;
  /// The resulting distances for the knn search.
  flann::Matrix<float> kDistances;
  /// The data for the knn search.
  flann::Matrix<float>* kData;

  /// knn distance threshold
  float threshold;
  /// folder to load feature vectors from
  std::string dataFolder;
  /// file extension to load feature vectors from.
  std::string fileExtension;

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
   * Load the list of objects and their properties from the parameter server. Blocks until the correct parameters become available.
   */
  virtual void loadTypeList();

public:
  /**
   * Constructor. Don't forget to call init() afterwards.
   */
  NNClassifier();
  virtual ~NNClassifier();

  /**
   * Actually set things up.
   */
  virtual void init();

  /**
   * Takes the incoming point cloud and stores it.
   * @param cloud the incoming cloud supplied from the topic publisher
   */
  virtual void cb_classify(sensor_msgs::PointCloud2 cloud) = 0;
};

#endif
