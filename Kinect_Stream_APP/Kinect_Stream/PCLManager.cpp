#include "stdafx.h"
#include "typedef.h"
#include "PCLManager.h"
#include "BoostUdpServer.h"


PCLManager::PCLManager() {
	
	loadRangeSettings();
}

void PCLManager::update(pcl::PointCloud<PointType>::Ptr inputPoints)
{
	//-----座標系変換
	pcl::PointCloud<PointType>::Ptr zeroPointCloud(new pcl::PointCloud<PointType>());
	transformToZeroPoint(inputPoints, zeroPointCloud);

	//-----フィルタリング
	//範囲で切り分け
	passThroughFilter(zeroPointCloud, "x", -screenWidth / 2.0, screenWidth / 2.0);
	passThroughFilter(zeroPointCloud, "y", minHeight, maxHeight);
	passThroughFilter(zeroPointCloud, "z", minDepth, maxDepth);

	//間引く
	voxelGridFilter(0.01f, zeroPointCloud);

	//ノイズ除去
	nanRemovalFilter(zeroPointCloud);
	statisticalOutlierFilter(zeroPointCloud);

	//保存
	*inputPoints = *zeroPointCloud;
	zeroPointCloud.reset();
}



void PCLManager::passThroughFilter(pcl::PointCloud<PointType>::Ptr inputCloud, const string &fieldName, float min, float max) {
	pcl::PassThrough<PointType> pass;
	pass.setInputCloud(inputCloud);
	pass.setFilterFieldName(fieldName);
	pass.setFilterLimits(min, max);

	pcl::PointCloud<PointType>::Ptr filterdCloud(new pcl::PointCloud<PointType>());
	pass.filter(*filterdCloud);
	*inputCloud = *filterdCloud;
}

void PCLManager::nanRemovalFilter(pcl::PointCloud<PointType>::Ptr cloud) {
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
}

void PCLManager::statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud)
{
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(5);				//近接何ポイントを使うか
	sor.setStddevMulThresh(1.0);	//この標準偏差以上をフィルターして切る

	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
	sor.filter(*cloud_filtered);

	pcl::copyPointCloud(*cloud_filtered, *cloud);
}

void PCLManager::radiusOutlinerFilter(pcl::PointCloud<PointType>::Ptr cloud) {
	pcl::RadiusOutlierRemoval<PointType> ror;
	ror.setInputCloud(cloud);
	ror.setRadiusSearch(0.05);
	ror.setMinNeighborsInRadius(2);
	pcl::PointCloud<PointType>::Ptr filterdCloud(new pcl::PointCloud<PointType>());
	ror.filter(*filterdCloud);
	*cloud = *filterdCloud;
	filterdCloud.reset();
}

void PCLManager::voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud)
{
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize(leaf, leaf, leaf);		//フィルター範囲設定
	grid.setInputCloud(cloud);
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
	grid.filter(*cloud_filtered);
	//pcl::copyPointCloud(*cloud_filtered, *cloud);
	*cloud = *cloud_filtered;
}

void PCLManager::loadRangeSettings()
{
	boost::property_tree::ptree pt;
	read_xml("Range_Setting.xml", pt);
	screenWidth = pt.get_optional<double>("sensing_range.range_width").get();
	minHeight = pt.get_optional<double>("sensing_range.range_height.min").get();
	maxHeight = pt.get_optional<double>("sensing_range.range_height.max").get();
	minDepth = pt.get_optional<double>("sensing_range.range_depth.min").get();
	maxDepth = pt.get_optional<double>("sensing_range.range_depth.max").get();
}

void PCLManager::transformToZeroPoint(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	pcl::PointCloud<PointType>::Ptr calcPoints(new pcl::PointCloud<PointType>());
	
	//RotateX
	float pitchRad = kinectPosture.getPitch() / 180.0 * M_PI;
	Eigen::Affine3f transformRotateX = Eigen::Affine3f::Identity();
	transformRotateX.rotate(Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*inputCloud, *outputCloud, transformRotateX);

	//RotateY
	float yawRad = kinectPosture.getYaw() / 180.0 * M_PI;
	Eigen::Affine3f transformRotateY = Eigen::Affine3f::Identity();
	transformRotateY.rotate(Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*outputCloud, *outputCloud, transformRotateY);

	//RotateZ
	float rollRad = kinectPosture.getRoll() / 180.0 * M_PI;
	Eigen::Affine3f transformRotateZ = Eigen::Affine3f::Identity();
	transformRotateZ.rotate(Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*outputCloud, *outputCloud, transformRotateZ);

	//移動
	Eigen::Affine3f transformMove = Eigen::Affine3f::Identity();
	transformMove.translation() << kinectPosture.getX(), kinectPosture.getY(), kinectPosture.getZ();
	pcl::transformPointCloud(*outputCloud, *outputCloud, transformMove);
}


void PCLManager::movingLeastSquares(pcl::PointCloud<PointType>::Ptr inputCloud)
{
	// Testing upsampling
	pcl::PointCloud<PointNormalType>::Ptr mls_normals(new pcl::PointCloud<PointNormalType>());
	pcl:: MovingLeastSquares<PointType, PointNormalType> mls_upsampling;
	// Set parameters
	mls_upsampling.setInputCloud(inputCloud);
	mls_upsampling.setComputeNormals(true);
	mls_upsampling.setPolynomialFit(true);
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	mls_upsampling.setSearchMethod(tree);
	mls_upsampling.setSearchRadius(0.06);
	//mls_upsampling.setUpsamplingMethod(pcl::MovingLeastSquares<PointType, PointNormalType>::SAMPLE_LOCAL_PLANE);
	//mls_upsampling.setUpsamplingRadius(0.005);
	//mls_upsampling.setUpsamplingStepSize(0.005);

	mls_normals->clear();
	mls_upsampling.process(*mls_normals);

	pcl::copyPointCloud(*mls_normals, *inputCloud);
}


