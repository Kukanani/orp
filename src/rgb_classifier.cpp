#include "orp/classifier/rgb_classifier.h"

#include <opencv2/highgui/highgui.hpp>

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
  ros::NodeHandle n;

  if(argc < 3) {
    ROS_FATAL("proper usage is 'rgb_classifier data_directory object_list_file [autostart]");
    return -1;
  }
  std::string directory = argv[1];
  std::string listFile = argv[2];
  bool autostart = false;
  if(argc >= 4) {
    if(std::string(argv[3])  == "true") autostart = true;
  }

  ROS_INFO("Starting RGB Classifier");
  RGBClassifier v(n, directory, listFile, autostart);
  v.init();

  //cv::namedWindow( "RGBCluster", cv::WINDOW_NORMAL );
  ros::AsyncSpinner spinner(2);
  spinner.start();
  while(ros::ok()) {

  }
  //cv::destroyAllWindows();
  return 1;
} //main

RGBClassifier::RGBClassifier(ros::NodeHandle nh, std::string dataFolder, std::string path, bool autostart):
  Classifier(nh, 10000, "rgb", path, dataFolder, ".rgb", autostart)
{

} //RGBClassifier

bool RGBClassifier::loadHist(const boost::filesystem::path &path, FeatureVector &RGB) {
  //no histogram loading
  return true;
} //loadHist

double testLast = 0; // used for debug testing

void RGBClassifier::cb_classify(sensor_msgs::PointCloud2 cloud) {
  //ROS_INFO_STREAM("Camera classification callback with " << cloud.width*cloud.height << " points.");
  orp::ClassificationResult classRes;

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  //ROS_INFO("RGB classifier calling segmentation");
  segmentationClient.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;
  //ROS_INFO("RGB classifier finished calling segmentation");
  //ROS_INFO("data size: %d x %d", kData->rows, kData->cols);

  for(std::vector<sensor_msgs::PointCloud2>::iterator eachCloud = clouds.begin(); eachCloud != clouds.end(); eachCloud++) {
    //ROS_INFO("Processing one cloud");
    if(eachCloud->width < 3) {
      //ROS_INFO("Cloud too small!");
      continue;
    }
    //ROS_INFO("cloud acceptable size");
    pcl::PointCloud<ORPPoint>::Ptr thisCluster (new pcl::PointCloud<ORPPoint>);
    pcl::fromROSMsg(*eachCloud, *thisCluster);

    Eigen::Vector4f clusterCentroid;
    pcl::compute3DCentroid(*thisCluster, clusterCentroid);
   
    Eigen::Affine3d finalPose;
    finalPose(0,3) = clusterCentroid(0);
    finalPose(1,3) = clusterCentroid(1);
    finalPose(2,3) = clusterCentroid(2);


    std::string color = "unknown";
    M.release();
    int DISP_HEIGHT=1;
    M = cv::Mat(eachCloud->width, DISP_HEIGHT, CV_8UC3, cv::Scalar(0,0,0));
    int i = 0;

    //if(pcl::getFieldIndex(*thisCluster, "rgb") != -1) { //if contains rgb, fill the opencv mat
      pcl::PointCloud<ORPPoint>::iterator point;
      for(point = thisCluster->points.begin(); point < thisCluster->points.end(); ++point, ++i) {
        cv::Vec3b color = cv::Vec3b(point->b, point->g, point->r);
        for(int y = 0; y < DISP_HEIGHT; ++y) {
          M.at<cv::Vec3b>(cv::Point(y, i)) = color;
        }
      }
      //processColors(M);
      //cv::imshow("RGBCluster", M);
      //cout << "M = " << endl << " " << M << endl << endl;
      cv::waitKey(10);
    //} else {
    //  ROS_WARN_THROTTLE(10, "No color information in cloud");
    //}

    color = getColor(M);
    classRes.result.label = "cube_" + color;
    tf::poseEigenToMsg(finalPose, classRes.result.pose.pose);

    classRes.method = "rgb";
    // ROS_INFO("RGB: position is %f %f %f",
    //   classRes.result.pose.pose.position.x,
    //   classRes.result.pose.pose.position.y,
    //   classRes.result.pose.pose.position.z);
    classificationPub.publish(classRes);

    delete[] kIndices.ptr();
    delete[] kDistances.ptr();
  }

  //ROS_INFO("classification call over");
} //classify


///http://stackoverflow.com/questions/5906693/how-to-reduce-the-number-of-colors-in-an-image-with-opencv-in-python
inline uchar reduceVal(const uchar val)
{
    if (val < 64) return 0;
    if (val < 128) return 64;
    return 255;
}

void RGBClassifier::processColors(cv::Mat& img)
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

std::string RGBClassifier::getColor(cv::Mat& img) {
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