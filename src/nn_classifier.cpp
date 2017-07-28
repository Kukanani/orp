// Copyright (c) 2017, Adam Allevato
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

#include "orp/core/nn_classifier.h"

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

NNClassifier::NNClassifier() :
  Classifier3D()
{
  srand (static_cast <unsigned> (time(0)));

  node_private_.param<std::string>("data_folder", dataFolder, "/");
  node_private_.param<std::string>("file_extension", fileExtension, "/");
  node_private_.param<float>("threshold", threshold, 10000.0f);
}

void NNClassifier::init() {
  //parse the params on the parameter server
  loadTypeList();
  if(!ros::isShuttingDown()) {

    loadModelsRecursive(dataFolder, fileExtension, loadedModels);
    ROS_INFO("Loaded %d models.\n", (int)loadedModels.size());

    if(loadedModels.size() < 1)
    {
      ROS_WARN ("No models loaded from %s.", dataFolder.c_str());
    } else {
      subModels = loadedModels;

      // Convert data into FLANN format
      kData = new flann::Matrix<float>(
        new float[subModels.size() * subModels[0].second.size()],
        subModels.size(),
        subModels[0].second.size());

      ROS_INFO_STREAM("data size: [" << kData->rows << " , " << kData->cols << "]");
      for(size_t i = 0; i < kData->rows; ++i)
      {
        for(size_t j = 0; j < kData->cols; ++j)
        {
          *(kData->ptr()+(i*kData->cols + j)) = subModels[i].second[j];
        }
      }

      kIndex = new flann::Index<flann::ChiSquareDistance<float> >(*kData, flann::LinearIndexParams ());
      kIndex->buildIndex();
    }
    ROS_INFO("Training data loaded.");
  }
  Classifier::init();
}

void NNClassifier::loadTypeList() {
  XmlRpc::XmlRpcValue paramMap;
  std::vector<WorldObjectType> objects;
  while(!node_.getParam("/items", paramMap)) {
    ROS_INFO_THROTTLE(5.0, "Waiting for object type list on parameter server...");
    ros::Duration(1.0).sleep();
  }
  for(XmlRpc::XmlRpcValue::iterator it = paramMap.begin(); it != paramMap.end(); ++it) {
    fullTypeList.push_back(it->first);
  }
}

NNClassifier::~NNClassifier() {
  delete[] kData->ptr();
  delete kIndex;
}

int NNClassifier::nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index,
    const FeatureVector &homeFeature,
    int k,
    flann::Matrix<int> &indices,
    flann::Matrix<float> &distances)
{
  int rows = 1;
  // Query point
  //flann::Matrix<float> home = flann::Matrix<float>(const_cast<float*>(&(homeFeature.second[0])), home.rows, homeFeature.second.size ());
  flann::Matrix<float> home = flann::Matrix<float>(new float[homeFeature.second.size ()], rows, homeFeature.second.size ());
  memcpy (&home.ptr ()[0], &homeFeature.second.at(0), home.cols * home.rows * sizeof (int));

  indices = flann::Matrix<int>(new int[home.rows*k], home.rows, k);
  distances = flann::Matrix<float>(new float[home.rows*k], home.rows, k);
  int foundCount = index.knnSearch (home, indices, distances, k, flann::SearchParams (128));

  return foundCount;
}

void NNClassifier::loadModelsRecursive(
  const boost::filesystem::path &base_dir,
  const std::string &extension,
  FeatureVectorVector &loadedModels)
{
  ROS_INFO("loading files from %s", base_dir.string().c_str());
  if(!boost::filesystem::is_directory (base_dir)) {
    ROS_FATAL("target path %s is not a directory!", base_dir.string().c_str());
    return;
  }
  if(!boost::filesystem::exists (base_dir)) {
    ROS_FATAL("target folder %s does not exist", base_dir.string().c_str());
    return;
  }

  for(boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it) {
    if(boost::filesystem::is_directory(it->status ())) {
      std::stringstream ss;
      ss << it->path();
      ROS_INFO("NOT traversing into directory %s.", ss.str().c_str());
    }
    if(boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == extension) {
      FeatureVector m;
      boost::regex pattern("");
      boost::cmatch what;
      for(std::vector<std::string>::iterator types = fullTypeList.begin(); types != fullTypeList.end(); ++types) {
        //match just the filename against the regex
        pattern = boost::regex("(" + *types + ")(.*)");
        if(boost::regex_match(it->path().filename().string().c_str(), what, pattern)) {
          if (loadHist(it->path(), m)) {
            loadedModels.push_back(m);
            ROS_INFO("Loading file %s", it->path().filename().string().c_str());
          } else {
            ROS_INFO_STREAM("histogram loader rejected file " << it->path().filename().string().c_str());
          }
        }
      }
    }
  }
}
