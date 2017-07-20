#pragma once
#include "KinectManager.h"
#include "VisualizerManager.h"

class App
{
public:
	App(){}
	~App(){}

	HRESULT init();
	void run();

	void sendPointCloud(pcl::PointCloud<PointType>::Ptr cloud);

	pcl::PointCloud<PointType>::Ptr convertDepthToPointCloud(vector<UINT16>& depthData);

	vector<DepthSpacePoint> convertPointCloudToDepthSpace(pcl::PointCloud<PointType>::Ptr inputCloud);

	pcl::PointCloud<PointType>::Ptr convertCameraSpaceToPointCloud(vector<CameraSpacePoint>& points);

	vector<unsigned char> createSendBuffer(pcl::PointCloud<PointType>::Ptr inputCloud);

private:

	void handleKey(char key);

	void workBegin();
	void workEnd();
	
	//Event
	bool running;

	//Kinect
	KinectManager &kinect = KinectManager::GetInstance();

	//XML
	string ipAddress;
	int depthPort;
	void loadAppSettings();

	//Visualizer
	VisualizerManager visualizer;

	//Util
	boost::posix_time::ptime workBeginTime;
	boost::posix_time::time_duration workDuration;
	boost::mutex mutex_lock;
};