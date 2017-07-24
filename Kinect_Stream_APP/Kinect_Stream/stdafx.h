// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#include "targetver.h"

//C++
#define _USE_MATH_DEFINES // for C++
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <atlbase.h>
#include <algorithm>
#include <cmath>

//OpenCV
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2\core\ocl.hpp"
#include "opencv2\objdetect.hpp"

//Kinect
#include <Kinect.h>

//Boost
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/property_tree/xml_parser.hpp>

//PCL
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/mls.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

//Common
#include "typeDef.h"


// TODO: プログラムに必要な追加ヘッダーをここで参照してください
