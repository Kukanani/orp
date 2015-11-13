#include "orp/classifier/opencv_classifier.h"

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  srand (static_cast <unsigned> (time(0)));

  ros::init(argc, argv, "rgb_classifier");

  if(argc < 3) {
    ROS_FATAL("proper usage is 'opencv_classifier data_directory object_list_file [autostart]");
    return -1;
  }
  std::string directory = argv[1];
  std::string listFile = argv[2];
  bool autostart = false;
  if(argc >= 4) {
    if(std::string(argv[3])  == "true") autostart = true;
  }

  ROS_INFO("Starting OpenCV Classifier");
  OpenCVClassifier v(directory, autostart);
  v.init();

  cv::namedWindow( "Visualization", cv::WINDOW_NORMAL );
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  //ros::Duration(10.0).sleep();
  //ros::shutdown();
  ros::waitForShutdown();
  cv::destroyAllWindows();
  return 1;
} //main

OpenCVClassifier::OpenCVClassifier(std::string dataFolder, bool autostart):
  Classifier(10000, "opencv", dataFolder, ".rgb", autostart),
  filterPub(n.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 100))
{

} //OpenCVClassifier

bool OpenCVClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &RGB) {
  //no histogram loading
  return true;
} //loadHist

double testLast = 0; // used for debug testing

void OpenCVClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {
  //ROS_INFO_STREAM("Camera classification callback with " << cloud.width*cloud.height << " points.");
  orp::ClassificationResult classRes;

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  segmentationClient.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;
  if(clouds.size() == 0) return;
  for(std::vector<sensor_msgs::PointCloud2>::iterator eachCloud = clouds.begin(); eachCloud != clouds.end(); eachCloud++) {
    
  }
  cloud = clouds[0];
  
  sensor_msgs::Image image_;
  pcl::toROSMsg (cloud, image_);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_);
  M.release();
  cv::inRange(cv_ptr->image, cv::Scalar(128, 0, 0), cv::Scalar(255, 255, 255), M);
  cv::imshow("Visualization", M);
  cv::waitKey(3);
  
  
  pcl::PointCloud<ORPPoint>::Ptr pclCloud = pcl::PointCloud<ORPPoint>::Ptr(new pcl::PointCloud<ORPPoint>());
  pcl::fromROSMsg(cloud, *pclCloud);
  
  uint8_t* pixelPtr = (uint8_t*)cv_ptr->image.data;
  int cn = cv_ptr->image.channels();
  cv::Scalar_<uint8_t> bgrPixel;
  int i= 0;
  for (size_t u = 0; u < cloud.height; ++u)   // rows
  {
    for (size_t v = 0; v < cloud.width; ++v, ++i)  // cols
    {
      if(cv_ptr->image.at<cv::Vec3b>(u,v).val[0] > 128) { //blue channel check
        pclCloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
        pclCloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
        pclCloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  pcl::toROSMsg(*pclCloud, cloud);
  filterPub.publish(cloud);
  
} //classify


///http://stackoverflow.com/questions/5906693/how-to-reduce-the-number-of-colors-in-an-image-with-opencv-in-python
inline uchar reduceVal(const uchar val)
{
    if (val < 64) return 0;
    if (val < 128) return 64;
    return 255;
}

void OpenCVClassifier::process(cv::Mat& img)
{
    uchar* pixelPtr = img.data;
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            const int pi = i*img.cols*3 + j*3;
            pixelPtr[pi + 0] = reduceVal(pixelPtr[pi + 0]); // B
            pixelPtr[pi + 1] = reduceVal(pixelPtr[pi + 1]); // G
            pixelPtr[pi + 2] = reduceVal(pixelPtr[pi + 2]); // R
            
            
            if(pixelPtr[pi+0] == 64) pixelPtr[pi+0] = 127;
            if(pixelPtr[pi+1] == 64) pixelPtr[pi+1] = 127;
            if(pixelPtr[pi+2] == 64) pixelPtr[pi+2] = 127;
        }
    }
}

std::string OpenCVClassifier::getColor(cv::Mat& img) {
  /*int channels[] = {0,1,2}; //histogram of R,G,B

  cv::MatND hist;

  int histSize[] = {3,3,3};
  float rranges[] = {0,256}; float granges[] = {0,256}; float branges[] = {0,256}; 
  const float* ranges[] = {rranges, granges, branges};
  cv::calcHist(&img, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, false);

  float max_value = 0.0f;
  int best_index = 0;
  float hist_value = 0.0f;
  for (int i=0; i<27; i++)
  {
    hist_value = hist.at<float>(i);
    if(hist_value > max_value) {
      max_value = hist_value;
      best_index = i;
    }
  }

  std::stringstream stream;
  stream << max_value;
  return stream.str();*/

  //sum the red channel (BGR pixel representation)
  double r = cv::sum(img)[2];

  //sum the green channel
  double g = cv::sum(img)[1];

  //sum the blue channel
  double b = cv::sum(img)[0];

  //which is greater?
  if(r > g && r > b) return "red";
  else if(g > r && g > b) return "green";
  return "blue";
}