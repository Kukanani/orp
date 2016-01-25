#ifndef _ORP_UTILS_H_
#define _ORP_UTILS_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <fstream>

#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

typedef pcl::PointXYZRGB ORPPoint;

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

  ///Convert degrees to radians.
  static double radFromDeg(double deg) {
    return deg * M_PI / 180.0;
  }

  ///Convert radians to degrees.
  static double degFromRad(double rad) {
    return rad * 180.0 / M_PI;
  }

  ///Load a point cloud from the input file, whether that file is an STL, PCD, or PLY file.
  static pcl::PointCloud<ORPPoint>::Ptr loadCloudFrom(std::string path) {
    pcl::PointCloud<ORPPoint>::Ptr cloud_out (new pcl::PointCloud<ORPPoint>);
    //pcl::PolygonMesh testMesh;
    std::string arg1 = path;
    std::string filetype1 = path.substr(path.length()-3);
    if(filetype1 == "stl") {
      std::cout << "PLY file loading is no longer supported due to deprecated loader libraries. Please load PCD files." << std::endl;
    }

    else if(filetype1 == "ply") {
      std::cout << "PLY file loading is no longer supported due to deprecated loader libraries. Please load PCD files." << std::endl;
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

  ///Create a box marker with the specified dimensions and color.
  static visualization_msgs::Marker makeBoxMarker(float xsize, float ysize, float zsize, float r, float g, float b)
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

  ///Create an interactive 6DOF box marker with the specified color and dimensions.
  static visualization_msgs::InteractiveMarkerControl& makeBoxControl(
    visualization_msgs::InteractiveMarker &msg,
    float xsize, float ysize, float zsize, float r = 0.5f, float g = 0.5f, float b = 0.5f)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBoxMarker(xsize, ysize, zsize, r, g, b));
    msg.controls.push_back( control );
    return msg.controls.back();
  }

  ///Save a 4x4 matrix to a file.
  static void saveEigenMatrix4f(std::string file, Eigen::Matrix4f mat) 
  {
    std::ofstream matFile;
    std::stringstream thisLine;

    matFile.open(file.c_str());
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

  ///Load a 4x4 matrix from a file.
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
    return mat;
  }

  ///Return a string representation of an integer with a fixed width, padded by leading zeros.
  static std::string zeroPad(int num, int fixedWidth)
  {
      std::ostringstream ss;
      ss << std::setw( fixedWidth ) << std::setfill( '0' ) << num;
      return ss.str();
  }
};

#endif //_ORP_UTILS_H_