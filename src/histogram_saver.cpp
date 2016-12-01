// Copyright (c) 2015, Adam Allevato
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "collector/histogram_saver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "histogram_saver");
  ros::NodeHandle n;

  std::string location = argc > 1 ? argv[1] : ".";

  ROS_INFO("Starting Histogram Saver");
  HistogramSaver* hs = new HistogramSaver(n, location);
  ros::spin();
  return 0;
}; //main

HistogramSaver::HistogramSaver(ros::NodeHandle nh, std::string location) : n(nh), feature(),
  savePCD(false), saveCVFH(false), outDir(location), tableCenterPoint()
{
  //Set up ROS
  saveCloudSrv = n.advertiseService("save_cloud", &HistogramSaver::cb_saveCloud, this );
  segClient = n.serviceClient<orp::Segmentation>("segmentation");
  tableCenterPointSub = n.subscribe("set_center_point", 1, &HistogramSaver::cb_setTableCenterPoint, this);

  reconfigureCallbackType = boost::bind(&HistogramSaver::paramsChanged, this, _1, _2);
  reconfigureServer.setCallback(reconfigureCallbackType);
}

void HistogramSaver::paramsChanged(
  orp::HistogramSaverConfig &config, uint32_t level)
{

  savePCD = config.save_pcd;
  saveCVFH = config.save_cvfh;
  save6DOF = config.save_sixdof;

  cvfhRadiusSearch = config.cvfh_radius_search;
} //paramsChanaged

void HistogramSaver::setTableCenterPoint(float x, float y, float z) {
  tableCenterPoint(0) = x;
  tableCenterPoint(1) = y;
  tableCenterPoint(2) = z;
}

bool HistogramSaver::cb_saveCloud(orp::SaveCloud::Request &req,
  orp::SaveCloud::Response &res)   
{
  //Segment cloud
  std::vector<pcl::PointCloud<ORPPoint>::Ptr> clouds;
  orp::Segmentation segSrvCall;
  segSrvCall.request.scene = req.in_cloud;

  segClient.call(segSrvCall);
  
  if(segSrvCall.response.clusters.size() < 1) {
    ROS_ERROR("no points returned from segmentation node.");
    return false;
  }
  pcl::PointCloud<ORPPoint>::Ptr cluster (new pcl::PointCloud<ORPPoint>);
  pcl::fromROSMsg(segSrvCall.response.clusters.at(0), *cluster);
  return saveCloud(cluster, req.objectName, req.angle);
} //cb_saveCloud

void HistogramSaver::cb_setTableCenterPoint(geometry_msgs::Vector3 _tableCenterPoint) {
  setTableCenterPoint(_tableCenterPoint.x, _tableCenterPoint.y, _tableCenterPoint.z);
}

bool HistogramSaver::saveCloud(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int angle) {
  ROS_INFO_STREAM(name << " cluster has " << cluster->height*cluster->width << " points.");
  if(!savePCD && !saveCVFH && !save6DOF) {
    ROS_WARN("Not saving any types of output files. Use rqt_reconfigure to turn on output.");
    return false;
  }

  if(savePCD) {
    writeRawCloud(cluster, name, angle);
  }
  if(saveCVFH) {
    writeCVFH(cluster, name, angle);
  }
  if(save6DOF) {
    write6DOF(cluster, name, angle);
  }
  return true;
} //saveCloud


void HistogramSaver::writeCVFH(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name,
  int angle)
{
  pcl::CVFHEstimation<ORPPoint, pcl::Normal, pcl::VFHSignature308> cvfh;
  cvfh.setInputCloud (cluster);

  //Estimate normals:
  pcl::NormalEstimation<ORPPoint, pcl::Normal> ne;
  ne.setInputCloud (cluster);
  pcl::search::KdTree<ORPPoint>::Ptr tree (new pcl::search::KdTree<ORPPoint> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (cvfhRadiusSearch);
  ne.compute (*cloud_normals);
  cvfh.setInputNormals (cloud_normals);

  //Estimate cvfh:
  cvfh.setSearchMethod (tree);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the feature
  cvfh.compute (*cvfhs);

  //Write to file: (objectName_angle.cvfh)
  std::stringstream fileName_ss;
  fileName_ss << outDir.c_str() << name.c_str() << "_" << angle << ".cvfh";

  ROS_INFO_STREAM("Writing CVFH to file '" << fileName_ss.str().c_str() << "'...");
  pcl::io::savePCDFile(fileName_ss.str(), *cvfhs);
} //writeCVFH

void HistogramSaver::writeRawCloud(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name,
  int angle)
{
  std::stringstream fileName_ss;
  fileName_ss << outDir << "/" << name.c_str() << "_" << angle << ".pcd";

  char thepath3[200];
  realpath(fileName_ss.str().c_str(), thepath3);

  //Write raw pcd file (objecName_angle.pcd)
  ROS_INFO_STREAM("writing raw cloud to file '" << thepath3 << "'");
  pcl::io::savePCDFile(thepath3, *cluster);
  ROS_DEBUG("done");
}

//http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)
typedef pcl::Histogram<90> CRH90;
POINT_CLOUD_REGISTER_POINT_STRUCT (CRH90, (float[90], histogram, histogram) )

void HistogramSaver::write6DOF(pcl::PointCloud<ORPPoint>::Ptr cluster, std::string name, int num)
{
  pcl::CVFHEstimation<ORPPoint, pcl::Normal, pcl::VFHSignature308> cvfh;
  cvfh.setInputCloud (cluster);

  //Estimate normals:
  pcl::NormalEstimation<ORPPoint, pcl::Normal> ne;
  ne.setInputCloud (cluster);
  pcl::search::KdTree<ORPPoint>::Ptr tree(new pcl::search::KdTree<ORPPoint>());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
  ne.setRadiusSearch (cvfhRadiusSearch);
  ne.compute (*cloud_normals);
  cvfh.setInputNormals (cloud_normals);

  //Estimate cvfh:
  cvfh.setSearchMethod (tree);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhs(new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the feature
  cvfh.compute (*cvfhs);

  //Write to file: (objectName_angle.cvfh)
  std::stringstream fileName_ss;
  fileName_ss << outDir << "/sixdof/" << name.c_str() << "_" << num << ".cvfh";

  ROS_INFO_STREAM("Writing 6DOF CVFH to file '" << fileName_ss.str().c_str() << "'...");
  pcl::io::savePCDFile(fileName_ss.str(), *cvfhs);

  //CRH/////////////////////////////////////////////////////////////
  // CRH estimation object.
  pcl::CRHEstimation<ORPPoint, pcl::Normal, CRH90> crh;
  crh.setInputCloud(cluster);
  crh.setInputNormals(cloud_normals);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);
  crh.setCentroid(centroid);

  pcl::PointCloud<CRH90>::Ptr histogram(new pcl::PointCloud<CRH90>);
  crh.compute(*histogram);

  //http://www.pcl-users.org/Save-pcl-Pointcloud-lt-pcl-Histogram-lt-N-gt-gt-td4035239.html
  //pcl::PointCloud< CRH90 > histogram_cloud; 
  //histogram_cloud.push_back(*histogram); 

  fileName_ss.str("");
  fileName_ss.clear();
  fileName_ss << outDir << "/sixdof/" << name << "_" << num << ".crh";
  ROS_INFO_STREAM("Writing 6DOF CRH to '" << fileName_ss.str().c_str() << "'...");
  pcl::io::savePCDFile(fileName_ss.str(), *histogram); 

  Eigen::Vector4f cloudCentroid;
  pcl::compute3DCentroid(*cluster, cloudCentroid);

  ROS_INFO_STREAM("Cloud centroid: " << cloudCentroid(0) << ", " << cloudCentroid(1) << ", " << cloudCentroid(2));
  ROS_INFO_STREAM("Table enter point " << tableCenterPoint(0) << ", " << tableCenterPoint(1) << ", " << tableCenterPoint(2));

  Eigen::Vector4f cloudToCenter = tableCenterPoint - cloudCentroid;
  ORPPoint minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);
  float objectHeight = maxPoint.z - minPoint.z;
  cloudToCenter(2) += objectHeight / 2.0f;
  ROS_INFO_STREAM("From cloud centroid to estimated object centroid: " << cloudToCenter(0) << ", " << cloudToCenter(1) << ", " << cloudToCenter(2));

  Eigen::Matrix4f cloudToCenterMatrix = Eigen::Matrix4f();
  cloudToCenterMatrix << 1,0,0,cloudToCenter(0),
                        0,1,0,cloudToCenter(1),
                        0,0,1,cloudToCenter(2),
                        0,0,0,1;

  fileName_ss.str("");
  fileName_ss.clear();
  fileName_ss << outDir << "/sixdof/" << name << "_" << num << ".mat4";
  ORPUtils::saveEigenMatrix4f(fileName_ss.str(), cloudToCenterMatrix);

  fileName_ss.str("");
  fileName_ss.clear();
  fileName_ss << outDir << "/sixdof/" << name << "_" << num << ".pcd";
  char thepath3[500];
  realpath(fileName_ss.str().c_str(), thepath3);

  //Write raw pcd file (objecName_angle.pcd)
  ROS_INFO_STREAM("writing raw cloud to file '" << thepath3 << "'");
  pcl::io::savePCDFile(thepath3, *cluster);
  ROS_DEBUG("done");
} //write6DOF