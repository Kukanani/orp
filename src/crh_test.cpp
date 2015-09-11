#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/crh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/histogram_visualizer.h>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *scene_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file 1\n");
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *model) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file 2\n");
    return (-1);
  }

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_; 
  normal_estimation_.setRadiusSearch(0.015f); 
  pcl::PointCloud<pcl::Normal>::Ptr scene_normal(new pcl::PointCloud<pcl::Normal>); 
  normal_estimation_.setInputCloud(scene_); 
  normal_estimation_.compute(*scene_normal); 

  pcl::PointCloud<pcl::Normal>::Ptr model_normal(new pcl::PointCloud<pcl::Normal>); 
  normal_estimation_.setInputCloud(model); 
  normal_estimation_.compute(*model_normal); 

  pcl::PointIndices::Ptr indices(new pcl::PointIndices); 
  pcl::removeNaNNormalsFromPointCloud<pcl::Normal>(*scene_normal, *scene_normal, indices->indices); 
  pcl::ExtractIndices<pcl::PointXYZ> extract; 
  extract.setInputCloud(scene_); 
  extract.setIndices(indices); 
  extract.setNegative(false); 
  extract.filter(*scene_); 

  indices->indices.clear(); 
  pcl::removeNaNNormalsFromPointCloud<pcl::Normal>(*model_normal, *model_normal, indices->indices); 
  extract.setInputCloud(model); 
  extract.setIndices(indices); 
  extract.setNegative(false); 
  extract.filter(*model); 

  //estimate centroid 
  Eigen::Vector4f scene_centroid; 
  Eigen::Vector4f model_centroid; 
  pcl::compute3DCentroid<pcl::PointXYZ>(*scene_, scene_centroid); 
  pcl::compute3DCentroid<pcl::PointXYZ>(*model, model_centroid); 

  //estimate camera roll histogram 
  pcl::CRHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<90> > crh_; 
  pcl::PointCloud<pcl::Histogram<90> >::Ptr scene_crh(new pcl::PointCloud<pcl::Histogram<90> >); 
  crh_.setInputCloud(scene_); 
  crh_.setInputNormals(scene_normal); 
  crh_.setCentroid(scene_centroid); 
  crh_.compute(*scene_crh); 

  pcl::PointCloud<pcl::Histogram<90> >::Ptr model_crh(new pcl::PointCloud<pcl::Histogram<90> >); 
  crh_.setInputCloud(model); 
  crh_.setInputNormals(model_normal); 
  crh_.setCentroid(model_centroid); 
  crh_.compute(*model_crh); 

  pcl::visualization::PCLHistogramVisualizer crh_visualizer; 
  crh_visualizer.addFeatureHistogram<pcl::Histogram<90> >(*scene_crh, 90, "scene feature"); 
  crh_visualizer.addFeatureHistogram<pcl::Histogram<90> >(*model_crh, 90, "model feature"); 
  crh_visualizer.spin();
}