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

#include "orp/classifier/sixdof_classifier.h"

#include <pcl/features/cvfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>

#include <boost/filesystem.hpp>

/**
 * Starts up the name and handles command-line arguments.
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sixdof_classifier");

  ROS_INFO("Starting SixDOF Classifier");
  SixDOFClassifier v;
  v.init();

  ros::spin();
  return 1;
} //main

SixDOFClassifier::SixDOFClassifier():
  NNClassifier()
{
  NNClassifier::init();
}

bool SixDOFClassifier::loadHist(
  const boost::filesystem::path &path, FeatureVector &sixdof)
{
  //ROS_INFO("Loading histogram %s", path.string().c_str());
  //path is the location of the file being read.
  int sixdof_idx;
  KnownPose kp;
  // Load the file as a PCD

  // Treat the SixDOF signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  if(pcl::io::loadPCDFile(path.string(), point) == -1) {
    ROS_ERROR("Couldn't read file %s", path.string().c_str());
    return false;
  }
  sixdof.second.resize(308);

  for (size_t i = 0; i < sixdof.second.size(); ++i)
  {
    sixdof.second[i] = point.points[0].histogram[i];
  }

  kp.name = path.filename().string();
  kp.dataName.assign(kp.name.begin()+kp.name.rfind("/")+1, kp.name.end());
  kp.name.assign(kp.name.begin()+kp.name.rfind("/")+1,
                kp.name.begin()+kp.name.rfind("_"));
  std::string cloud_name = path.string();

  std::string cloudName;
  cloudName.assign(cloud_name.begin(),
                   cloud_name.begin()+cloud_name.rfind("."));
  cloudName += ".pcd";
  // load the file
  if (pcl::io::loadPCDFile<ORPPoint> (cloudName, *(kp.cloud)) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", cloudName.c_str());
    return false;
  }

  std::string matName;
  matName.assign(cloud_name.begin(), cloud_name.begin()+cloud_name.rfind("."));
  matName += ".mat4";
  kp.pose = ORPUtils::loadEigenMatrix4f(matName.c_str()).cast<double>();

  kp.centroid = Eigen::Vector4f(kp.pose(0,3),
                                kp.pose(1,3),
                                kp.pose(2,3),
                                kp.pose(3,3));

  std::string crhName;
  crhName.assign(cloud_name.begin(), cloud_name.begin()+cloud_name.rfind("."));
  crhName += ".crh";
  if(pcl::io::loadPCDFile(crhName, *(kp.crh)) == -1) {
    ROS_ERROR("Couldn't read file %s", crhName.c_str());
    return false;
  }

  sixdof.first = kp;
  return true;
}

double testLast = 0; // used for debug testing

void SixDOFClassifier::cb_classify(const sensor_msgs::PointCloud2& cloud) {
  orp::ClassificationResult classRes;
  classRes.method = "sixdof";

  orp::Segmentation seg_srv;
  seg_srv.request.scene = cloud;
  segmentation_client_.call(seg_srv);
  std::vector<sensor_msgs::PointCloud2> clouds = seg_srv.response.clusters;

  if(!clouds.empty()) {
    for(auto eachCloud = clouds.begin(); eachCloud != clouds.end();
      eachCloud++)
    {
      if(eachCloud->width < 3)
      {
        continue;
      }
      orp::WorldObject thisObject;
      pcl::PointCloud<ORPPoint>::Ptr thisCluster(
        new pcl::PointCloud<ORPPoint>);
      pcl::fromROSMsg(*eachCloud, *thisCluster);

      //Compute sixdof:
      pcl::CVFHEstimation<ORPPoint, pcl::Normal, pcl::VFHSignature308> cvfh;
      cvfh.setInputCloud (thisCluster);
      //Estimate normals:
      pcl::NormalEstimation<ORPPoint, pcl::Normal> ne;
      ne.setInputCloud (thisCluster);
      pcl::search::KdTree<ORPPoint>::Ptr treeNorm(
        new pcl::search::KdTree<ORPPoint> ());
      ne.setSearchMethod (treeNorm);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch (0.03);
      ne.compute (*cloud_normals);

      //SixDOF estimation
      cvfh.setInputNormals(cloud_normals);
      cvfh.setSearchMethod(treeNorm);
      pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhs(
        new pcl::PointCloud<pcl::VFHSignature308> ());
      cvfh.compute(*cvfhs);

      //Nearest neighbor algorigthm
      FeatureVector histogram;
      histogram.second.resize(308);

      for (size_t i = 0; i < 308; ++i) {
        // TODO(Kukanani): does the fact that we only look at index 0 here
        // matter? I think it might.
        histogram.second[i] = cvfhs->points[0].histogram[i];
      }

      int numNeighbors = 1;
      // KNN classification find nearest neighbors based on histogram. Only
      // concerned with the first one, since our dataset only includes one
      // data point for each classification/pose pair
      int numFound = 0;

      numFound = nearestKSearch (*kIndex, histogram, numNeighbors,
        kIndices, kDistances);

      if(numFound == 0) {
        ROS_ERROR("KNN search found 0 nearby feature vectors");
      }

      int limit = std::min<int>(numNeighbors, numFound);

      for(int j=0; j<limit; j++) {
        ROS_DEBUG("SixDOF: Dist(%s@%.2f): %f",
          subModels.at(kIndices[0][j]).first.name.c_str(),
          subModels.at(kIndices[0][j]).first.angle, kDistances[0][j]);
      }

      Eigen::Vector4f clusterCentroid;
      pcl::compute3DCentroid(*thisCluster, clusterCentroid);

      pcl::PointCloud<CRH90>::Ptr clusterCRH(new pcl::PointCloud<CRH90>);
      pcl::CRHEstimation<ORPPoint, pcl::Normal, CRH90> clusterCRHGen;
      clusterCRHGen.setInputCloud(thisCluster);
      clusterCRHGen.setInputNormals(cloud_normals);
      clusterCRHGen.setCentroid(clusterCentroid);
      clusterCRHGen.compute(*clusterCRH);

      Eigen::Vector4f viewCentroid =
        subModels.at(kIndices[0][0]).first.centroid;

      pcl::PointCloud<CRH90>::Ptr viewCRH =
        subModels.at(kIndices[0][0]).first.crh;

      pcl::CRHAlignment<ORPPoint, 90> alignment;
      alignment.setInputAndTargetView(thisCluster,
        subModels.at(kIndices[0][0]).first.cloud);
      // CRHAlignment works with Vector3f, not Vector4f.
      Eigen::Vector3f viewCentroid3f(viewCentroid[0],
                                     viewCentroid[1],
                                     viewCentroid[2]);
      Eigen::Vector3f clusterCentroid3f(clusterCentroid[0],
                                        clusterCentroid[1],
                                        clusterCentroid[2]);
      alignment.setInputAndTargetCentroids(clusterCentroid3f, viewCentroid3f);

      // Compute the roll angle(s).
      std::vector<float> angles;
      alignment.computeRollAngle(*clusterCRH, *viewCRH, angles);

      Eigen::Affine3d finalPose;
      finalPose(0,3) = clusterCentroid(0)+viewCentroid(0);
      finalPose(1,3) = clusterCentroid(1)+viewCentroid(1);
      finalPose(2,3) = clusterCentroid(2)+viewCentroid(2);

      if (angles.size() == 0)
      {
        ROS_WARN("[sixdof] No angles correlated.");
      }
      else {
        // CRH rotation

        // get the rotation vector - just the vector to the centroid in object
        // space
        Eigen::Vector3d rotVec = Eigen::Vector3d(0.0f, 0.0f, 1.0f);
        rotVec.normalize();

        double radRotationAmount = 2*M_PI - angles.at(0) * M_PI/180;

        // create a quaternion that represents rotation around an axis
        Eigen::Quaterniond quat;
        quat = Eigen::AngleAxisd(radRotationAmount, rotVec);
        quat.normalize();
        Eigen::Matrix3d rotMat; rotMat = quat;

        // rotate the object using the quaternion.
        Eigen::Affine3d crhRot; crhRot =
          Eigen::Affine3d(Eigen::AngleAxisd(radRotationAmount, rotVec));
        finalPose.linear() = rotMat;
      }

      finalPose.linear() *= subModels.at(kIndices[0][0]).first.pose.linear();

      thisObject.label = subModels.at(kIndices[0][0]).first.name;
      tf::poseEigenToMsg(finalPose, thisObject.pose.pose);

      classRes.result.push_back(thisObject);
      delete[] kIndices.ptr();
      delete[] kDistances.ptr();
    }
  }
  classification_pub_.publish(classRes);
} //classify