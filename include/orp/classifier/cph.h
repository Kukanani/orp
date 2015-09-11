#ifndef _CPH_H_
#define _CPH_H_

#include <iostream>
#include <cmath>

#include <flann/flann.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "orp/core/orp_utils.h"

#define CPH_DEBUG 0

/**
 * @brief   Performs CPH analyses and creates CPH histograms
 *
 * @version 1.2
 * @ingroup objectrecognition
 * 
 * @author  Brian O'Neil <brian.oneil@lanl.gov>
 * @author  Adam Allevato <adam.d.allevato@gmail.com>
 * @copyright BSD 3-paragraph
 * @date    Jan 21, 2015
 */
class CPH{
private:
  /** The histogram storing the CPH. */
  std::vector<float> hist;
  /** The cloud used to compute the CPH. Set this before calling compute(). */
  pcl::PointCloud<ORPPoint>::Ptr cloud;

  /** The number of vertical bins (slices) on the cylindrical projection mesh */
  int num_ybins;
  /** The number of angular bins on the cylindrical projection mesh */
  int num_cbins;

  /** The size of the point cloud bounding box in the x-direction. */
  float x_size;
  /** The size of the point cloud bounding box in the y-direction. */
  float y_size;
  /** The size of the point cloud bounding box in the z-direction. */
  float z_size;

  /** Maximum spatial extent */
  float max_size;

  /** X maximum spatial extent */
  float x_max;
  /** X minimum spatial extent */
  float x_min;
  /** Y maximum spatial extent */
  float y_max;
  /** Y minimum spatial extent */
  float y_min;
  /** Z maximum spatial extent */
  float z_max;
  /** Z minimum spatial extent */
  float z_min;

  /** The 3D cartesian point representing the centroid of the point cloud data. */
  std::vector<float> centroid;

  /**
   * Compute the spatial bounds of the point cloud to get the dimensions of the bounding box
   * and the centroid location.
   */
  void computeBounds() {
    if(cloud->points.empty()) return;
    x_max = cloud->points.at(0).x;
    x_min = cloud->points.at(0).x;
    y_max = cloud->points.at(0).y;
    y_min = cloud->points.at(0).y;
    z_max = cloud->points.at(0).z;
    z_min = cloud->points.at(0).z;

    for(unsigned int i=0; i<cloud->size(); i++) {
      if(cloud->points.at(i).x > x_max)
        x_max = cloud->points.at(i).x;
      if(cloud->points.at(i).x < x_min)
        x_min = cloud->points.at(i).x;
      if(cloud->points.at(i).y > y_max)
        y_max = cloud->points.at(i).y;
      if(cloud->points.at(i).y < y_min)
        y_min = cloud->points.at(i).y;
      if(cloud->points.at(i).z > z_max)
        z_max = cloud->points.at(i).z;
      if(cloud->points.at(i).z < z_min)
        z_min = cloud->points.at(i).z;
    }

    x_size = (x_max - x_min);
    y_size = (y_max - y_min);
    z_size = (z_max - z_min);

    max_size = x_size;
    if(y_size > max_size)
      max_size = y_size;
    if(z_size > max_size)
      max_size = z_size;

    centroid.at(0) = x_min + x_size/2;
    centroid.at(1) = y_min + y_size/2;
    centroid.at(2) = z_min + z_size/2;
  } //computeBounds

public:
  /**
   * Constructor with specified dataset size.
   * @param num_y the number of y-bins in the data (how many vertical slices)
   * @param num_c the number of c-bins in the data (how many lognitudinal sections)
   */
  CPH(int num_y, int num_c){
    num_ybins = num_y;
    num_cbins = num_c;
    hist.resize(num_ybins * num_cbins,0);
    centroid.resize(3,0);
  }; //CPH

  /**
   * Set the input data to use for computation of the CPH.
   * @param inputCloud The PointCloud to store internally and use for computations.
   */
  void setInputCloud(pcl::PointCloud<ORPPoint>::Ptr inputCloud){
    cloud = inputCloud;
    computeBounds();
  }; //setInputCloud

  /**
   * Compute the Circular Projection Histogram on the point cloud data stored in this this class.
   * @param result A vector of floats representing the calculated histogram
   * @return       The length (size) of the result.
   */
  int compute(std::vector<float> &result){
    //compute feature
    hist.clear();
    hist.resize(num_cbins*num_ybins, 0.0f);
    int y,c;
    float dz = z_size/num_ybins;
    float dc = 2*M_PI/num_cbins;

    if(CPH_DEBUG == 1) {
      std::cout << "size of the histogram is " << (int)hist.size() << ".\n";

      //test to find if y ever = 0.
      int minz = 10000, maxz = -1, minc = 10000, maxc = -1;
      for(unsigned int i=0; i<cloud->size(); i++){
        y = floor((cloud->points.at(i).z-y_min)/dz);
        c = floor((M_PI+atan2(cloud->points.at(i).z-centroid.at(2),cloud->points.at(i).x-centroid.at(0)))/dc);
        if(c < minc) minc = c;
        if(c > maxc) maxc = c;
        if(y < minz) minz = y;
        if(y > maxz) maxz = y;
      }
      std::cout << "C: [" << minc << ", " << maxc << "]. Y: [" << minz << ", " << max_size << "]. ";
    }

    //cycle through the points in the cloud
    for(unsigned int i=0; i<cloud->size(); i++){
      //for each cloud, find the corresponding y-c bin.
      y = trunc((cloud->points.at(i).y-z_min)/dz);
      c = trunc((M_PI+atan2(cloud->points.at(i).z-centroid.at(1),cloud->points.at(i).x-centroid.at(0)))/dc);

      //clamp values to expected. Because of small rounding errors there may be overflow.
      if(y > num_ybins-1) y = num_ybins-1;
      if(c > num_cbins-1) c = num_cbins-1;
      
      if(y*num_cbins+c > hist.size()-1) {
        if(CPH_DEBUG == 1) {
          std::cout << "CPH histogram structure is incorrect size for the attempted new point.";
          std::cout << "While binning point " << i << ". y: " << y << "  c: " << c;
          std::cout << "dy = "<<y_size<<"/"<<num_ybins;
          std::cout << "Y = trunc(("<<cloud->points.at(i).y<<" - "<<z_min<<")/"<<dz<<")";
        }
      }
      else {
        //increment the histogram value at this point.
        hist.at(y*num_cbins+c)+=1.0f;
      }
    }

    if(hist.size() > 0) {
      //Find tallest peak (largest spatial extent)
      float max_peak = 0;
      for(unsigned int i=0; i<hist.size(); i++){
        if(hist.at(i) > max_peak)
          max_peak = (float)hist.at(i); 
      }

      //Rescale cph to largest spatial extent:
      float scaleFactor = max_size*100/max_peak;
      for(unsigned int i=0; i<hist.size(); i++){
        hist.at(i)*=scaleFactor;
      }
    }

    //Stick size onto the end and BAM! scale variance.
    hist.push_back(x_size*100);
    hist.push_back(y_size*100); 
    hist.push_back(z_size*100);

    //load the data
    result.clear();
    result = hist;

    //return data size
    return result.size();
  }; //compute
}; //CPH

#endif //_CPH_H_