///////////////////////////////////////////////////////////////////////////////
//      Title     : Classifer
//      Project   : NRG ORP
//      Created   : 1/21/2015
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

#include "core/classifier.h"

Classifier::Classifier(float thresh, std::string _name,
  std::string folder, std::string fileExt, bool _autostart) :
    name(_name),
    dataFolder(folder),
    fileExtension(fileExt),
    threshold(thresh),
    autostart(_autostart)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
} //Classifier

Classifier::Classifier(const Classifier& other) {
  n = other.n;
  name = other.name;
  dataFolder = other.dataFolder;
  fileExtension = other.fileExtension;
  threshold = other.threshold;
  autostart = other.autostart;
  //may have to add other copies here 
}

void Classifier::init() {
  ROS_INFO("%s: Reading list file", name.c_str());

  //parse the params on the parameter server
  loadTypeList();
  if(!ros::isShuttingDown()) {

    loadModelsRecursive(dataFolder, fileExtension, loadedModels);
    ROS_INFO("%s: Loaded %d models.\n", name.c_str(), (int)loadedModels.size());

    if(loadedModels.size() < 1)
    {
      ROS_WARN ("%s: No models loaded from %s.", name.c_str(), dataFolder.c_str());
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

    startSub = n.subscribe("orp_start_recognition", 1, &Classifier::cb_subscribe, this);
    stopSub = n.subscribe("orp_stop_recognition", 1, &Classifier::cb_unsubscribe, this);

    if(autostart) {
      ROS_INFO("Autostarting classification");
      subscribe();
    }
  }
} //Classifier

void Classifier::loadTypeList() {
    std::vector<std::string> paramMap;
  while(!n.getParam("/items/list", paramMap) && !ros::isShuttingDown()) {
    ROS_INFO_THROTTLE(5.0, "Waiting for object type list on parameter server...");
    ros::Duration(1.0).sleep();
  }
  for(std::vector<std::string>::iterator it = paramMap.begin(); it != paramMap.end(); ++it) {
    fullTypeList.push_back(*it);
  }
}

Classifier::~Classifier() {
  //delete[] kData->ptr();
  //delete kIndex;
} //~Classifier

int Classifier::nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index,
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
  //delete[] home.ptr();

  return foundCount;
} //nearestKSearch

void Classifier::loadModelsRecursive(
  const boost::filesystem::path &base_dir,
  const std::string &extension, 
  FeatureVectorVector &loadedModels)
{
  ROS_INFO("%s: loading files from %s", name.c_str(), base_dir.string().c_str());
  if(!boost::filesystem::is_directory (base_dir)) {
    ROS_FATAL("%s: target path %s is not a directory!", name.c_str(), base_dir.string().c_str());
    return;
  }
  if(!boost::filesystem::exists (base_dir)) {
    ROS_FATAL("%s: target folder %s does not exist", name.c_str(), base_dir.string().c_str());
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
        else {
          //ROS_INFO("Skipping file %s", it->path().filename().string().c_str());
        }
      }
    }
    else {
      //ROS_INFO("Skipping file %s", it->path().filename().string().c_str());
    }
  }
} //loadModelsRecursive

void Classifier::subscribe()
{
  ROS_INFO("Classifier subscribing");
  classificationPub = n.advertise<orp::ClassificationResult>("/classification", 1);
  depthInfoSubscriber = n.subscribe(
    "/camera/depth_registered/points", 1, &Classifier::cb_classify, this);

  segmentationClient = n.serviceClient<orp::Segmentation>("/segmentation");
} //subscribe

void Classifier::unsubscribe()
{
  if(depthInfoSubscriber != NULL)
  {
    depthInfoSubscriber.shutdown();
  }
  if(segmentationClient != NULL)
  {
    segmentationClient.shutdown();
  }
} //unsubscribe


void Classifier::cb_subscribe(std_msgs::Empty msg) {
  subscribe();
} //cb_subscribe

void Classifier::cb_unsubscribe(std_msgs::Empty msg) {
  unsubscribe();
} //cb_unsubscribe
