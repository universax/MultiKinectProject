#include "stdafx.h"
#include "App.h"
#include "BoostUdpServer.h"
#include "PCLManager.h"

boost::shared_ptr<boost::asio::io_service> _iosvc(new boost::asio::io_service());
BoostUdpServer flowServer(_iosvc);
BoostUdpServer pointCloudServer(_iosvc);

HRESULT App::init() {
	HRESULT hr = S_FALSE;
	running = false;

	//Kinect Setting
	hr = kinect.init();

	//Load AppSettings
	loadAppSettings();

	if (SUCCEEDED(hr) && ipAddress != "" && depthPort > 0)
	{
		running = true;
	}
	
	return hr;
}

void App::run() {
	PCLManager &pclManager = PCLManager::GetInstance();
	//-----------------------------------
	//Thread Group
	//-----------------------------------
	boost::thread_group threads;
	
	while (running)
	{
		workBegin();
		kinect.update();
		if (kinect.getIsFrameNew())
		{
			//-----------------------------------
			//kinect
			//-----------------------------------
			//depth取得
			vector<UINT16> depthBuf;
			kinect.getDepthBuf(depthBuf);

			//-----------------------------------
			//PCL
			//-----------------------------------
			pcl::PointCloud<PointType>::Ptr kinectRawCloud = convertDepthToPointCloud(depthBuf);
			pclManager.update(kinectRawCloud);

			if (kinectRawCloud->size() < 100)
			{
				continue;
			}
			
			//送信
			pcl::PointCloud<PointType>::Ptr sendPCLPtr(new pcl::PointCloud<PointType>());
			*sendPCLPtr = *kinectRawCloud;
			threads.create_thread(boost::bind(&App::sendPointCloud, this, sendPCLPtr));

			//Visualizer
			visualizer.updateVisualizer(kinectRawCloud);

			threads.join_all();
			kinectRawCloud.reset();
		}
		Sleep(1);
		workEnd();
	}
}


//--------------------------------------------------------------------------------------------
pcl::PointCloud<PointType>::Ptr App::convertDepthToPointCloud(vector<UINT16> &depthData) {
	//Kinectのデプス画像をリアル座標に変換
	vector<CameraSpacePoint> cameraSpacePoints(depthData.size());
	kinect.pCoordinateMapper->MapDepthFrameToCameraSpace(depthData.size(), &depthData[0], cameraSpacePoints.size(), &cameraSpacePoints[0]);

	pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());
	for (size_t i = 0; i < cameraSpacePoints.size(); i++)
	{
		PointType point;
		if (cameraSpacePoints[i].Z < 0)
		{
			continue;
		}
		point.x = cameraSpacePoints[i].X;
		point.y = cameraSpacePoints[i].Y;
		point.z = cameraSpacePoints[i].Z;
		ptr->points.push_back(point);
	}
	return ptr;
}

vector<DepthSpacePoint> App::convertPointCloudToDepthSpace(pcl::PointCloud<PointType>::Ptr inputCloud) {
	vector<DepthSpacePoint> depthPoints(0);
	for (int i = 0; i < inputCloud->size(); i++)
	{
		CameraSpacePoint cp;
		cp.X = inputCloud->points[i].x;
		cp.Y = inputCloud->points[i].y;
		cp.Z = inputCloud->points[i].z;

		DepthSpacePoint dp;
		kinect.pCoordinateMapper->MapCameraPointToDepthSpace(cp, &dp);
		depthPoints.push_back(dp);
	}

	return depthPoints;
}

pcl::PointCloud<PointType>::Ptr App::convertCameraSpaceToPointCloud(vector<CameraSpacePoint> &points) {
	pcl::PointCloud<PointType>::Ptr ptr(new pcl::PointCloud<PointType>());
	for (int i = 0; i < points.size(); i++)
	{
		PointType p;
		p.x = points[i].X;
		p.y = points[i].Y;
		p.z = points[i].Z;

		ptr->push_back(p);
	}
	return ptr;
}

//--------------------------------------------------------------------------------------------
vector<unsigned char> App::createSendBuffer(pcl::PointCloud<PointType>::Ptr inputCloud)
{
	vector<unsigned char> buf(0);
	for (int i = 0; i < inputCloud->size(); i++)
	{
		DataConverter converter;
		//X
		converter.sValue[0] = inputCloud->points[i].x * 1000;
		for (int j = 0; j < 2; j++)
		{
			buf.push_back(converter.data[j]);
		}
		//Y
		converter.sValue[0] = inputCloud->points[i].y * 1000;
		for (int j = 0; j < 2; j++)
		{
			buf.push_back(converter.data[j]);
		}
		//Z
		converter.sValue[0] = inputCloud->points[i].z * 1000;
		for (int j = 0; j < 2; j++)
		{
			buf.push_back(converter.data[j]);
		}
	}
	
	return buf;
}

void App::sendPointCloud(pcl::PointCloud<PointType>::Ptr cloud) {
	vector<unsigned char> sendPCLBuf = createSendBuffer(cloud);
	cloud.reset();
	pointCloudServer.send(_iosvc, sendPCLBuf, ipAddress.data(), depthPort);
}

//--------------------------------------------------------------------------------------------
void App::handleKey(char key) {
	switch (key)
	{
	case 27:
		running = false;
		break;
	case 'd':
	{

	}
	break;
	default:
		break;
	}
}

void App::workBegin() {
	workBeginTime = boost::posix_time::microsec_clock::local_time();
}

void App::workEnd() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	workDuration = curTime - workBeginTime;
	mutex_lock.lock();
	cout << "*** Work Duration: " << workDuration.total_milliseconds() << " ***" << endl;
	mutex_lock.unlock();
}

void App::loadAppSettings()
{
	boost::property_tree::ptree pt;
	read_xml("App_Setting.xml", pt);
	ipAddress = pt.get_optional<string>("app_setting.send_to.ip_address").get();
	depthPort = pt.get_optional<int>("app_setting.send_to.depth_port").get();
}
