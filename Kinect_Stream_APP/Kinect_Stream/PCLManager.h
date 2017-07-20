#pragma once
#include "Singleton.h"
#include "KinectPosture.h"


using namespace std;

class PCLManager: public Singleton<PCLManager>
{
public:
	void update(pcl::PointCloud<PointType>::Ptr inputPoints);
	
private:
	friend class Singleton< PCLManager >;
	PCLManager();
	~PCLManager() {}

	//Kinect Settings
	KinectPosture &kinectPosture = KinectPosture::GetInstance();

	//Range
	double screenWidth, minHeight, maxHeight, minDepth, maxDepth;
	void loadRangeSettings();

	//à⁄ìÆÅEâÒì]
	void transformToZeroPoint(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud);

	//Filter
	void passThroughFilter(pcl::PointCloud<PointType>::Ptr inputCloud, const string &fieldName, float min, float max);
	void nanRemovalFilter(pcl::PointCloud<PointType>::Ptr cloud);
	void statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud);
	void radiusOutlinerFilter(pcl::PointCloud<PointType>::Ptr cloud);
	void voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud);

	//ïΩääâª
	void movingLeastSquares(pcl::PointCloud<PointType>::Ptr inputCloud);

};

