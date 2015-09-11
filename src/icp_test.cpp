#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>

#include "orp/core/orp_utils.h"

int
 main (int argc, char** argv)
{
  bool userand = false;
  int iterations = 0;
  if(argc < 3) {
    if(argc == 2) {
      std::cout << "Usage is: icp_test pcd_file stl_file" << std::endl;
    } else {
      userand = true;
      std::cout << "No files specified, using randomly generated point clouds. " << std::endl;
    }
  }
  if(argc == 4) {
    iterations = atoi(argv[3]);
  }

  pcl::PointCloud<ORPPoint>::Ptr cloud_in (new pcl::PointCloud<ORPPoint>);
  pcl::PointCloud<ORPPoint>::Ptr cloud_out (new pcl::PointCloud<ORPPoint>);

  if(userand) {
    // Fill in the CloudIn data
    cloud_in->width    = 5;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
      cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
        << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
        cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
        cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
      cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    cloud_out->width    = 6;
    cloud_out->points.resize (cloud_out->width * cloud_out->height);
    cloud_out->points[5].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_out->points[5].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_out->points[5].z = 1024 * rand () / (RAND_MAX + 1.0f);
    std::cout << "Transformed " << cloud_out->points.size () << " data points:"
        << std::endl;
    for (size_t i = 0; i < cloud_out->points.size (); ++i)
      std::cout << "    " << cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  }
  else { //load from files
    pcl::PolygonMesh testMesh1;
    std::string arg1 = std::string(argv[1]);
    std::string filetype1 = arg1.substr(arg1.length()-3);
    if(filetype1 == "stl") {
      if (pcl::io::loadPolygonFileSTL(argv[1], testMesh1) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input STL file\n");
        return (-1);
      }
      pcl::fromPCLPointCloud2(testMesh1.cloud, *cloud_in);
    } else if(filetype1 == "ply") {
      if (pcl::io::loadPolygonFilePLY(argv[1], testMesh1) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input PLY file\n");
        return (-1);
      }
      pcl::fromPCLPointCloud2(testMesh1.cloud, *cloud_in);
    } else if(filetype1 == "pcd") {
      if (pcl::io::loadPCDFile<ORPPoint>(argv[1], *cloud_in) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input PCD file\n");
        return (-1);
      }
    }
    else {
      PCL_ERROR("second argument filetype not recognized.");
      return -1;
    }

    pcl::PolygonMesh testMesh;
    std::string arg2 = std::string(argv[2]);
    std::string filetype = arg2.substr(arg2.length()-3);
    if(filetype == "stl") {
      if (pcl::io::loadPolygonFileSTL(argv[2], testMesh) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input STL file\n");
        return (-1);
      }
      pcl::fromPCLPointCloud2(testMesh.cloud, *cloud_out);
    } else if(filetype == "ply") {
      if (pcl::io::loadPolygonFilePLY(argv[2], testMesh) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input PLY file\n");
        return (-1);
      }
      pcl::fromPCLPointCloud2(testMesh.cloud, *cloud_out);
    } else if(filetype == "pcd") {
      if (pcl::io::loadPCDFile<ORPPoint>(argv[2], *cloud_out) == -1) // load input
      {
        PCL_ERROR ("Couldn't read input PCD file\n");
        return (-1);
      }
    }
    else {
      PCL_ERROR("second argument filetype not recognized.");
      return -1;
    }


    if(argc == 5) {
      pcl::PointCloud<ORPPoint>::Ptr cloud_append (new pcl::PointCloud<ORPPoint>);
      pcl::PolygonMesh testMesh5;
      std::string arg4 = std::string(argv[4]);
      std::string filetype = arg4.substr(arg4.length()-3);
      if(filetype == "stl") {
        if (pcl::io::loadPolygonFileSTL(argv[4], testMesh5) == -1) // load input
        {
          PCL_ERROR ("Couldn't read input STL file\n");
          return (-1);
        }
        pcl::fromPCLPointCloud2(testMesh5.cloud, *cloud_append);
      } else if(filetype == "ply") {
        if (pcl::io::loadPolygonFilePLY(argv[4], testMesh5) == -1) // load input
        {
          PCL_ERROR ("Couldn't read input PLY file\n");
          return (-1);
        }
        pcl::fromPCLPointCloud2(testMesh5.cloud, *cloud_append);
      } else if(filetype == "pcd") {
        if (pcl::io::loadPCDFile<ORPPoint>(argv[4], *cloud_append) == -1) // load input
        {
          PCL_ERROR ("Couldn't read input PCD file\n");
          return (-1);
        }
      }
      else {
        PCL_ERROR("fifth argument filetype not recognized.");
        return -1;
      }
      *cloud_out += *cloud_append;
    }
  }

  pcl::IterativeClosestPoint<ORPPoint, ORPPoint> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<ORPPoint>::Ptr cloud_final (new pcl::PointCloud<ORPPoint>);
  icp.setMaximumIterations(iterations);
  icp.align(*cloud_final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<ORPPoint> rgb_in(cloud_in);
  pcl::visualization::PointCloudColorHandlerRGBField<ORPPoint> rgb_out (cloud_out);
  pcl::visualization::PointCloudColorHandlerCustom<ORPPoint> rgb_final(cloud_final, 0, 255, 0);
  viewer->addPointCloud<ORPPoint> (cloud_out, rgb_out, "stl");
  viewer->addPointCloud<ORPPoint> (cloud_in, rgb_in, "pcd");
  viewer->addPointCloud<ORPPoint> (cloud_final, rgb_final, "final");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stl");


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

 return (0);
}