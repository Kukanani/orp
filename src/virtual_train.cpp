// Copyright (c) 2016, Adam Allevato
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

#include <string>
#include <iostream>
#include <sstream>

#include <vtkPolyDataMapper.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/recognition/crh_alignment.h>

#include <pcl_ros/transforms.h>

#include "orp/core/orp_utils.h"

int
main(int argc, char** argv)
{
	if(argc < 2) {
		std::cout << "syntax: virtual_train input_file object_name output_directory" << std::endl;
		return -1;
	}
	std::string inputFile = argv[1];
	std::string objName = argv[2];
	std::string outDir = argv[3];
	// Load the PLY model from a file.
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(inputFile.c_str());
	reader->Update();
 
	// VTK is not exactly straightforward...
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();
 
	vtkSmartPointer<vtkPolyData> object = mapper->GetInput();
 
	// Virtual scanner object.
	pcl::apps::RenderViewsTesselatedSphere render_views;
	render_views.addModelFromPolyData(object);
	// Pixel width of the rendering window, it directly affects the snapshot file size.
	render_views.setResolution(150);
	// Horizontal FoV of the virtual camera.
	render_views.setViewAngle(57.0f);
	// If true, the resulting clouds of the snapshots will be organized.
	render_views.setGenOrganized(true);
	// How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
	render_views.setTesselationLevel(1);
	// If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
	// This will affect the number of snapshots produced (if true, less will be made).
	// True: 42 for level 1, 162 for level 2, 642 for level 3...
	// False: 80 for level 1, 320 for level 2, 1280 for level 3...
	render_views.setUseVertices(true);
	// If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
	render_views.setComputeEntropies(true);
 
	render_views.generateViews();
 
	// Object for storing the rendered views.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
	// Object for storing the poses, as 4x4 transformation matrices.
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	// Object for storing the entropies (optional).
	std::vector<float> entropies;
	render_views.getViews(views);
	render_views.getPoses(poses);
	render_views.getEntropies(entropies);

	ROS_INFO_STREAM("the output directory is " << outDir);
	ROS_INFO_STREAM("the object name is  " << objName);

	std::stringstream stream;
	for(int i = 0; i < views.size(); ++i)
	{
		stream << outDir << objName << "_" << i << ".pcd";
		ROS_INFO_STREAM("outputting to " << stream.str());
		pcl::io::savePCDFileASCII (stream.str().c_str(), *views[i]);
		stream.str("");
		stream.clear();

    Eigen::Vector4f cloudCentroid;// = subModels.at(kIndices[0][0]).first.centroid;
    pcl::compute3DCentroid(*(views[i]), cloudCentroid);
		ROS_INFO_STREAM("pose center is at coordinates " << poses[i](0,3) << ", " <<
							poses[i](1,3) << ", " << poses[i](2,3));
		poses[i](0,3) -= cloudCentroid(0);
		poses[i](1,3) -= cloudCentroid(1);
		poses[i](2,3) -= cloudCentroid(2);
		ROS_INFO_STREAM("centroid of point cloud is at coordinates " << cloudCentroid(0) << ", " <<
							cloudCentroid(1) << ", " << cloudCentroid(2));
    pcl::compute3DCentroid(*(views[i]), cloudCentroid);
		ROS_INFO_STREAM("saving the difference: " << poses[i](0,3) << ", " <<
							poses[i](1,3) << ", " << poses[i](2,3));

		stream << outDir << objName << "_" << i << ".mat4";
		ORPUtils::saveEigenMatrix4f(stream.str(), poses[i]);
		stream.str("");
		stream.clear();
	}
}