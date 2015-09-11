#ifndef _ORP_UTILS_H_
#define _ORP_UTILS_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <iomanip>


#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

typedef pcl::PointXYZ ORPPoint;

class file_error: public std::runtime_error
{
public:
  explicit file_error(const char* message) : runtime_error(message)
    {
    }
};

/**
 * Utility methods
 */

namespace ORPUtils {
//public:
  static double radFromDeg(double deg) {
    return deg * M_PI / 180.0;
  }
  static double degFromRad(double rad) {
    return rad * 180.0 / M_PI;
  }
  ///Load a point cloud from the input file, whether that file is an STL, PCD, or PLY file.
  static pcl::PointCloud<ORPPoint>::Ptr loadCloudFrom(std::string path) {
    pcl::PointCloud<ORPPoint>::Ptr cloud_out (new pcl::PointCloud<ORPPoint>);
    pcl::PolygonMesh testMesh;
    std::string arg1 = path;
    std::string filetype1 = path.substr(path.length()-3);
    if(filetype1 == "stl") {
      if (pcl::io::loadPolygonFileSTL(path.c_str(), testMesh) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input STL file\n");
        throw file_error("Couldn't read input STL file");
      }
      else {
        pcl::fromPCLPointCloud2(testMesh.cloud, *cloud_out);
      }
    }

    else if(filetype1 == "ply") {
      if (pcl::io::loadPolygonFilePLY(path.c_str(), testMesh) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input PLY file\n");
        throw file_error("Couldn't read input PLY file");
      }
      else {
        pcl::fromPCLPointCloud2(testMesh.cloud, *cloud_out);
      }
    }

    else if(filetype1 == "pcd") {
      if (pcl::io::loadPCDFile<ORPPoint>(path.c_str(), *cloud_out) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input PCD file\n");
        throw file_error("Couldn't read input PCD file");
      }
    }
    else {
      PCL_ERROR("Argument filetype not recognized.");
       throw std::invalid_argument("Argument filetype not recognized.");
    }
    return cloud_out;
  }

  static visualization_msgs::Marker makeBox(float xsize, float ysize, float zsize, float r, float g, float b)
  {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.5*xsize;
    marker.scale.y = 0.5*ysize;
    marker.scale.z = 0.5*zsize;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    return marker;
  }

  static visualization_msgs::InteractiveMarkerControl& makeBoxControl(
    visualization_msgs::InteractiveMarker &msg,
    float xsize, float ysize, float zsize, float r = 0.5f, float g = 0.5f, float b = 0.5f)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(xsize, ysize, zsize, r, g, b));
    msg.controls.push_back( control );
    return msg.controls.back();
  }

  static bool attemptToCopyFloatParam(const ros::NodeHandle &node, std::string from, std::string to) {
    double temp;
    if(!node.hasParam(from)) {
      ROS_ERROR("source param %s not found while attempting to copy to %s", from.c_str(), to.c_str());
      throw std::runtime_error("Source param does not exist");
      return false;
    } if(!node.hasParam(to)) {
      ROS_ERROR("target param %s not found while attempting to copy from %s", to.c_str(), from.c_str());
      throw std::runtime_error("Target param does not exist");
      return false;
    }
    node.getParam(from, temp);
    node.setParam(to, temp);
    return true;
  }

  static bool attemptToSetFloatParam(const ros::NodeHandle &node, float val, std::string to) {
    double temp;if(!node.hasParam(to)) {
      ROS_ERROR("target param %s not found while attempting to set value=%f", to.c_str(), val);
      throw std::runtime_error("Target param does not exist");
      return false;
    }
    node.setParam(to, (double)val);
    return true;
  }

  static bool attemptToReloadFloatParam(const ros::NodeHandle &node, std::string paramName, float &toFill)
  {
    double temp;
    if(!node.hasParam(paramName)) {
      ROS_ERROR("can't find parameter %s", paramName.c_str());
      throw std::runtime_error("Source param does not exist");
      return false;
    }
    node.getParam(paramName, temp);
    //ROS_INFO("\tloaded parameter %s, value = %f", paramName.c_str(), temp);
    toFill = temp;
    return true;
  } //attemptToReloadFloatParam

  static bool attemptToReloadDoubleParam(const ros::NodeHandle &node, std::string paramName, double &toFill)
  {
    if(!node.hasParam(paramName)) {
      ROS_ERROR("can't find parameter %s", paramName.c_str());
      throw std::runtime_error("Source param does not exist");
      return false;
    }
    node.getParam(paramName, toFill);
    //ROS_INFO("\tloaded parameter %s, value = %lf", paramName.c_str(), toFill);
    return true;
  } //attemptToReloadFloatParam

  static bool attemptToReloadStringParam(const ros::NodeHandle &node, std::string paramName, std::string &toFill)
  {
    if(!node.hasParam(paramName)) {
      ROS_ERROR("can't find parameter %s", paramName.c_str());
      throw std::runtime_error("Source param does not exist");
      return false;
    }
    node.getParam(paramName, toFill);
    //ROS_INFO("\tloaded parameter %s, value = %s", paramName.c_str(), toFill.c_str());
    return true;
  } //attemptToReloadFloatParam

  static void saveEigenMatrix4f(std::string file, Eigen::Matrix4f mat) 
  {
    std::ofstream matFile;
    std::stringstream thisLine;

    matFile.open(file.c_str());
   // if(!matFile) 
    //{
    //  ROS_INFO("Can't save matrix file %s", file.c_str());
      ///return;
    //}
    ROS_INFO("Saving matrix to file %s...", file.c_str());
    for(int i=0; i<mat.rows(); i++) {
      thisLine.str("");
      thisLine.clear();
      for(int j=0; j<mat.cols(); j++) {
        thisLine << ((float)mat(i,j)) << " ";
      }
      matFile << thisLine.str() << std::endl;
    }
    matFile.close();
  }; //saveEigenMatrix4f

  static Eigen::Matrix4f loadEigenMatrix4f(std::string file) {
    Eigen::Matrix4f mat;

    // Read the transformation file
    std::ifstream transformationFile;
    std::string thisLine;
    // Try to open the transformation file. If it does not open, set transformation to identity matrix
    transformationFile.open(file.c_str(), std::ios::in);
    if(!transformationFile.is_open()) 
    {
      ROS_ERROR("Transformation file %s does not exist. Initializing transform to identity matrix", file.c_str());
    }
    else 
    {
      //ROS_INFO_STREAM("Reading transformation file %s..." << file.c_str());
      int i = 0;
      while(std::getline(transformationFile, thisLine) && i<4) 
      {
        std::istringstream objectSS(thisLine);
        //Get pose sigma
        objectSS >> mat(i, 0);
        objectSS >> mat(i, 1);
        objectSS >> mat(i, 2);
        objectSS >> mat(i, 3);
        i++;
      }
    }
    transformationFile.close();
    //ROS_INFO("done");
    return mat;
  }

  static std::string zeroPad(int num, int fixedWidth)
  {
      std::ostringstream ss;
      ss << std::setw( fixedWidth ) << std::setfill( '0' ) << num;
      return ss.str();
  }
};

#endif //_ORP_UTILS_H_