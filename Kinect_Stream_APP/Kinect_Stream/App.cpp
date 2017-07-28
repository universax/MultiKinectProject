#include "stdafx.h"
#include "App.h"
#include "BoostUdpServer.h"
#include "PCLManager.h"
#include "OpticalFlow.h"

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
	OpticalFlow flow;
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
			//depth�擾
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

			//���M
			pcl::PointCloud<PointType>::Ptr sendPCLPtr(new pcl::PointCloud<PointType>());
			*sendPCLPtr = *kinectRawCloud;
			threads.create_thread(boost::bind(&App::sendPointCloud, this, sendPCLPtr));

			//Visualizer
			visualizer.updateVisualizer(kinectRawCloud);


			//-----------------------------------
			//OPENCV
			//-----------------------------------
			//��������摜��̃C���f�b�N�X
			vector<int> indicesOnImage;
			//�|�C���g�N���E�h��̃C���f�b�N�X
			vector<int> indicesOnPointCloud;
			//�摜�𐶐�
			Size depthImageSize(300, 200);
			Mat depthMat = createMatAndOrganizedPointCloud(kinectRawCloud, depthImageSize, indicesOnImage, indicesOnPointCloud);
			Mat culcFlowMat;
			//depthMat.copyTo(culcFlowMat);
			//for (int i = 0; i < depthMat.size().width * depthMat.size().height; i++)
			//{
			//	culcFlowMat.data[i] = midianFilter(i, depthMat, 1);
			//}
			//imshow("MFilter", culcFlowMat);
			//Flow�̌v�Z
			UMat flowMat;
			depthMat.copyTo(flowMat);


			//---------- Flow�v�Z�i�t���[�����[�g��������Ɨ�����̂œK�X�X���[�v�j
			//if (workDuration.total_milliseconds() < 17)
			//{
			//	Sleep(17 - workDuration.total_milliseconds());
			//}
			flow.update(flowMat);
			UMat drawMat;
			cvtColor(flowMat, drawMat, CV_GRAY2BGR);
			flow.draw(drawMat);

			//�f�[�^�擾�i�摜���X,Y���W�Ŏ擾�ł���j
			vector<Point2f> flowBeginPoints = flow.getFlowBeginPoints();
			vector<Point2f> flowEndPoints = flow.getFlowEndPoints();
			vector<Point2f> forces;
			if (flowBeginPoints.size() == flowEndPoints.size())
			{
				//�������擾
				for (int i = 0; i < flowBeginPoints.size(); i++)
				{
					Point2f force = flowEndPoints[i] - flowBeginPoints[i];
					forces.push_back(force);
				}

				//�摜��̍��W����A���ۂ̃|�C���g�N���E�h�̒l���擾����
				pcl::PointCloud<PointType>::Ptr flowPoints(new pcl::PointCloud<PointType>());
				for (int i = 0; i < flowEndPoints.size(); i++)
				{
					Point2f fp = flowEndPoints[i];
					Point2f f = forces[i];
					//�������������Ȃ��
					int targetIndex = depthImageSize.width * fp.y + fp.x;
					if (fp.x < 0 || fp.y < 0)
					{
						continue;
					}

					//�����炳�܂Ƀm�C�Y���ۂ����
					float distance = sqrt(f.x * f.x + f.y * f.y);
					if (distance > 15 || distance < 1)
					{
						continue;
					}

					//���̃C���f�b�N�X�t�߂ŁA��ԗL�������Ȓl��T��
					for (int j = 0; j < indicesOnImage.size(); j++)
					{
						int indexOnImage = indicesOnImage[j];
						if (targetIndex > indexOnImage - 2 && targetIndex < indexOnImage + 2)
						{
							//�|�C���g�N���E�h��̓_�𑗐M�p�N���E�h�ɒǉ�
							PointType targetPoint = kinectRawCloud->points[indicesOnPointCloud[j]];
							flowPoints->push_back(targetPoint);

							//flow�̋����𑗐M�p�N���E�h�ɒǉ�
							PointType force(f.x, f.y, 0);
							flowPoints->push_back(force);
						}
					}
				}

				//���M
				threads.create_thread(boost::bind(&App::sendFlow, this, flowPoints));
			}

			threads.join_all();
			kinectRawCloud.reset();
		}
		Sleep(1);
		workEnd();
	}
}


//--------------------------------------------------------------------------------------------
pcl::PointCloud<PointType>::Ptr App::convertDepthToPointCloud(vector<UINT16> &depthData) {
	//Kinect�̃f�v�X�摜�����A�����W�ɕϊ�
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

void App::sendFlow(pcl::PointCloud<PointType>::Ptr cloud) {
	vector<unsigned char> sendFlowBuf = createSendBuffer(cloud);
	flowServer.send(_iosvc, sendFlowBuf, ipAddress.data(), flowPort);
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
	boost::property_tree::ptree ptApp;
	read_xml("App_Setting.xml", ptApp);
	ipAddress = ptApp.get_optional<string>("app_setting.send_to.ip_address").get();
	depthPort = ptApp.get_optional<int>("app_setting.send_to.depth_port").get();
	flowPort = ptApp.get_optional<int>("app_setting.send_to.flow_port").get();

	boost::property_tree::ptree ptKinect;
	read_xml("Range_Setting.xml", ptKinect);
	rangeWidth = ptKinect.get_optional<float>("sensing_range.range_width").get();

	float hMin = ptKinect.get_optional<float>("sensing_range.range_height.min").get();
	float hMax = ptKinect.get_optional<float>("sensing_range.range_height.max").get();
	rangeHeight = hMax - hMin;

	float dMin = ptKinect.get_optional<float>("sensing_range.range_depth.min").get();
	float dMax = ptKinect.get_optional<float>("sensing_range.range_depth.max").get();
	rangeDepth = dMax - dMin;
}

Mat App::createMatAndOrganizedPointCloud(pcl::PointCloud<PointType>::Ptr inputCloud, Size imageSize, vector<int> &indicesOnImage, vector<int> &indicesOnPointCloud) {
	//�Y�ꂸ���Z�b�g����
	indicesOnImage.clear();
	indicesOnPointCloud.clear();

	Mat outMat(imageSize.height, imageSize.width, CV_8UC1);
	rectangle(outMat, Rect(0, 0, imageSize.width, imageSize.height), Scalar(255, 255, 255), -1);

	for (int i = 0; i < inputCloud->size(); i++)
	{
		int x = imageSize.width / 2 + (inputCloud->points[i].x / (rangeWidth)) * imageSize.width;
		if (x > imageSize.width - 1 || x < 0)
		{
			continue;
		}
		int y = imageSize.height - (inputCloud->points[i].y / (rangeHeight)) * imageSize.height;
		//int y = (depthData[i].Y + 350) * 0.3;
		if (y > imageSize.height - 1 || y < 0)
		{
			continue;
		}
		int index = y * outMat.size().width + x;

		int color = (inputCloud->points[i].z / rangeDepth) * 255 - 50;
		if (color >= 255)
		{
			color = 255;
		}
		if (color < 0)
		{
			color = 0;
		}
		outMat.data[index] = color;

		//�摜��̃C���f�b�N�X�ƁA�|�C���g�N���E�h��̃C���f�b�N�X��ۑ����Ƃ�
		indicesOnImage.push_back(index);
		indicesOnPointCloud.push_back(i);
	}






	////�܂�DepthSpace��PointCloud��ϊ����āA
	//vector<DepthSpacePoint> depthData = convertPointCloudToDepthSpace(inputCloud);

	////�Y�ꂸ���Z�b�g����
	//indicesOnImage.clear();
	//indicesOnPointCloud.clear();

	//Mat outMat(imageSize.height, imageSize.width, CV_8UC1);
	//rectangle(outMat, Rect(0, 0, imageSize.width, imageSize.height), Scalar(255, 255, 255), -1);

	//for (int i = 0; i < depthData.size(); i += 1)
	//{
	//	int x = imageSize.width / 2 + (depthData[i].X / (rangeWidth * 1000)) * imageSize.width;
	//	//int x = (depthData[i].X + 200) * 0.3;
	//	if (x > imageSize.width - 1 || x < 0)
	//	{
	//		continue;
	//	}
	//	int y = imageSize.height / 2 + (depthData[i].Y / (rangeHeight * 1000)) * imageSize.height;
	//	//int y = (depthData[i].Y + 350) * 0.3;
	//	if (y > imageSize.height - 1 || y < 0)
	//	{
	//		continue;
	//	}
	//	int index = y * outMat.size().width + x;
	//	outMat.data[index] = 0;
	//
	//	//�摜��̃C���f�b�N�X�ƁA�|�C���g�N���E�h��̃C���f�b�N�X��ۑ����Ƃ�
	//	indicesOnImage.push_back(index);
	//	indicesOnPointCloud.push_back(i);
	//}

	//imshow("test", outMat);
	return outMat;
}

bool App::midianNoiseFilter(int bufIndex, Mat &inputMat, int filterLevel) {
	if (filterLevel <= 0)
	{
		return true;
	}

	if (inputMat.data[bufIndex] < 0)
	{
		return false;
	}

	//�܂�x,y���W�ɒ���
	int x = bufIndex % inputMat.size().width;
	int y = bufIndex / inputMat.size().width;

	//�����s�N�Z�����ӂ̐[�x�̍��v�l���Ƃ�
	int counter = 0;
	int calcPixelCount = 0;

	//���̑O�ɖ����Ȓl�̓X���[
	if (
		x <= filterLevel ||
		x >= inputMat.size().width - filterLevel ||
		y <= filterLevel ||
		y >= inputMat.size().height - filterLevel
		) {
		return false;
	}

	vector<int> pixels(0);
	for (int arX = -filterLevel; arX <= filterLevel; arX++)
	{
		for (int arY = -filterLevel; arY <= filterLevel; arY++)
		{
			int index = (y + arY) * inputMat.size().width + (x + arX);
			if (inputMat.data[index] < 255) {
				counter += 1;
			}

			calcPixelCount += 1;

			pixels.push_back(inputMat.data[index]);
		}
	}
	sort(pixels.begin(), pixels.end());
	int numPixel = pixels.size();
	int midianIndex = numPixel / 2;


	//���v�v�Z�Ώۃs�N�Z�����ɂ���āA�����s�N�Z�����m�C�Y���ĂȂ����̂��ǂ����`�F�b�N
	float ratio = 0.2;
	if (counter >= calcPixelCount * ratio)
	{
		return true;
	}
	return false;
}


int App::midianFilter(int bufIndex, Mat &inputMat, int filterLevel) {
	if (filterLevel <= 0)
	{
		return 255;
	}

	if (inputMat.data[bufIndex] < 0)
	{
		return 0;
	}

	//�܂�x,y���W�ɒ���
	int x = bufIndex % inputMat.size().width;
	int y = bufIndex / inputMat.size().width;

	//���̑O�ɖ����Ȓl�̓X���[
	if (
		x <= filterLevel ||
		x >= inputMat.size().width - filterLevel ||
		y <= filterLevel ||
		y >= inputMat.size().height - filterLevel
		) {
		return 255;
	}

	vector<int> pixels(0);
	for (int arX = -filterLevel; arX <= filterLevel; arX++)
	{
		for (int arY = -filterLevel; arY <= filterLevel; arY++)
		{
			int index = (y + arY) * inputMat.size().width + (x + arX);
			pixels.push_back(inputMat.data[index]);
		}
	}
	sort(pixels.begin(), pixels.end());
	int numPixel = pixels.size();
	int midianIndex = numPixel / 2;

	return inputMat.data[midianIndex];
}