// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#include "targetver.h"
//C++
//#define _USE_MATH_DEFINES // for C++
//#include <stdio.h>
//#include <atlbase.h>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <cmath>
//#include <ctime>
#include <chrono>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost\enable_shared_from_this.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl\io\ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl\surface\mls.h>
#include <pcl\surface\gp3.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\filters\statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl\range_image\range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ros/conversions.h>

#include "typeDef.h"

// TODO: プログラムに必要な追加ヘッダーをここで参照してください
