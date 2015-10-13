#include "orp/core/classifier.h"

Classifier::Classifier(ros::NodeHandle nh, float thresh, std::string _name,
  std::string _path, std::string folder, std::string fileExt, bool _autostart) :
    n(nh),
    name(_name),
    dataFolder(folder),
    path(_path),
    fileExtension(fileExt),
    threshold(thresh),
    autostart(_autostart)
{
} //Classifier

Classifier::Classifier(const Classifier& other) {
  n = other.n;
  name = other.name;
  dataFolder = other.dataFolder;
  path = other.path;
  fileExtension = other.fileExtension;
  threshold = other.threshold;
  autostart = other.autostart;
  //may have to add other copies here 
}

void Classifier::init() {
  ROS_INFO("%s: Reading list file %s", name.c_str(), path.c_str());

  FloatLookupTable table;
  //parse
  std::ifstream objectListFile;
  std::string tempString, eachLine, label;

  objectListFile.open(path.c_str(), std::ios::in);  
  std::getline(objectListFile, eachLine);
  std::istringstream nameSS(eachLine);
  while(nameSS >> eachLine)
  {
    fullTypeList.push_back(eachLine);
  }
  if(fullTypeList.size() < 1)
  {
    ROS_FATAL ("%s: No types loaded from %s. This is gonna be bad.", name.c_str(), path.c_str());
    return;
  }
  subTypeList = fullTypeList;

  objectListFile.close();

  loadModelsRecursive(dataFolder, fileExtension, loadedModels);
  ROS_INFO("%s: Loaded %d models.\n", name.c_str(), (int)loadedModels.size());

  if(loadedModels.size() < 1)
  {
    ROS_FATAL ("%s: No models loaded from %s. This is going to be worse.", name.c_str(), dataFolder.c_str());
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

  detectionSetSub = n.subscribe(
    "/detection_set",
    50,
    &Classifier::cb_detectionSet,
    this);

  startSub = n.subscribe("orp_start_recognition", 1, &Classifier::cb_subscribe, this);
  stopSub = n.subscribe("orp_stop_recognition", 1, &Classifier::cb_unsubscribe, this);

  if(autostart) {
    ROS_INFO("Autostarting classification");
    subscribe();
  }
} //Classifier

Classifier::~Classifier() {
  delete[] kData->ptr();
  delete kIndex;
} //~Classifier


void Classifier::setDetectionSet(std::vector<std::string> set) {
  ROS_INFO("%s classifier's detection set is being subselected", name.c_str());
  subTypeList.clear();
  for(std::vector<std::string>::iterator it = set.begin(); it != set.end(); ++it) {
    if(std::find(fullTypeList.begin(), fullTypeList.end(), *it) != fullTypeList.end()) { //exists in full type list
      subTypeList.push_back(*it);
    } else { //doesn't exist in full sensor model
      ROS_ERROR("detection subset item %s not found in full type list", it->c_str());
    }
  }
  if(subTypeList.empty()) {
    ROS_FATAL("No items in Classifier detection type list subset!");
  }
  subTypeList.push_back("unknown");

  subModels.clear(); //subset of models
  for(FeatureVectorVector::iterator known = loadedModels.begin(); known != loadedModels.end(); ++known) {
    for(std::vector<std::string>::iterator allowed = set.begin(); allowed != set.end(); ++allowed) {
      if(known->first.name.find(*allowed) == 0 || known->first.name.find("unknown") == 0) {
        //ROS_INFO("\tPushing back %s into subset of models for FLANN.", known->first.name.c_str());
        subModels.push_back(*known);
      } 
    }
  }
  if(subModels.empty()) {
    ROS_ERROR("no models in feature vector list after subsetting");
    return;
  }
  //ROS_INFO("downselected to %i models", subModels.size());

  if(kData) {
    delete[] kData->ptr();
    delete kData;
  }
  if(kIndex) {
    delete kIndex;
  }
  // Convert data into FLANN format
  kData = new flann::Matrix<float>(
    new float[subModels.size() * subModels[0].second.size()],
    subModels.size(),
    subModels[0].second.size());

  //ROS_INFO_STREAM("data size: [" << kData->rows << " , " << kData->cols << "]");
  for(size_t i = 0; i < kData->rows; ++i)
  {
    for(size_t j = 0; j < kData->cols; ++j)
    {
      ROS_INFO_STREAM("" << i << ", " << j << ": " << subModels[i].second[j]);
      *(kData->ptr()+(i*kData->cols + j)) = subModels[i].second[j];
    }
  }

  kIndex = new flann::Index<flann::ChiSquareDistance<float> >(*kData, flann::LinearIndexParams ());
  kIndex->buildIndex();

  //ROS_INFO("data size: %d x %d", kData->rows, kData->cols);
} //setDetectionSet

void Classifier::cb_detectionSet(orp::DetectionSet msg)
{
  setDetectionSet(msg.objects);
} //cb_detectionSet

int  Classifier::nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index,
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
