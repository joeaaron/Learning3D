/****************************************************************
// Copyright © 2022, iTech. Co., Ltd.
// File name: MyCloud.h
// Author: Z
// Version: 1.0.0
// Date: 2022/04/11
// Description: 点云数据结构
// History:
// <version>    <date>	    <author>	<desc>
// 1.0.0	    2022/04/11	   Z 	    build this module
****************************************************************/

#pragma once
//设置中文编码
#pragma execution_character_set("utf-8")

// !C/C++
#include <iostream>
#include <string>
#include <vector>

// !PCL
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>	
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
// !VTK
#include <vtkRenderWindow.h>	
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using PointT = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud<PointT>;

namespace MyCloud
{
	typedef struct CloudStructure
	{
		Cloud::Ptr ptrCloud;			// 点云指针
		std::string strPathName;		// 点云的全路径名
		std::string strDirName;			// 点云文件目录名
		std::string strFileName;		// 点云的文件名
		bool bVisible;					// 点云在Viewer中是否可见
	}CloudStructure, *PtrCloudStructure;
}


