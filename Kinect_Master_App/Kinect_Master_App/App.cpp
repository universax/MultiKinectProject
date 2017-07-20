#include "stdafx.h"
#include "PCLManager.h"
#include "BoostUdpServer.h"
#include "App.h"


HRESULT App::init() {
	HRESULT hr = S_OK;
	_running = false;

	if (SUCCEEDED(hr) && loadNeworkSettings())
	{
		_running = true;

	}
	return hr;
}

bool App::loadNeworkSettings()
{
	boost::property_tree::ptree pt;
	read_xml("App_Setting.xml", pt);
	sendIpaddress = pt.get_optional<string>("app_setting.send_to.ip_address").get();
	depthSendtPort = pt.get_optional<int>("app_setting.send_to.depth_port").get();
	positionSendPort = pt.get_optional<int>("app_setting.send_to.position_port").get();

	if (sendIpaddress != "" && depthSendtPort != 0 && positionSendPort != 0)
	{
		return true;
	}
	return false;
}

void App::run() {
	//-----------------------------------
	//Viewer
	//-----------------------------------
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGB>());
	viewer.reset(new pcl::visualization::PCLVisualizer("Kinect Master"));
	viewer->addPointCloud(c);
	viewer->initCameraParameters();
	viewer->setCameraPosition(3.0, 3.0, -3.0, 0, 0, 0);	//3D


	//-----------------------------------
	//UDP Server
	//-----------------------------------
	//��M
	boost::shared_ptr<boost::asio::io_service> io_service(new boost::asio::io_service);
	//�|�[�g�ʂɃX���b�h�𕪂��Ď�M
	//�|�[�g���ƂɃT�[�o�[���Ă�
	vector<boost::shared_ptr<BoostUdpServer>> recieveServers;
	for (int i = 0; i < NUM_KINECT; i++)
	{
		boost::shared_ptr<BoostUdpServer> svr(new BoostUdpServer(io_service));
		recieveServers.push_back(svr);
		deathCounter[i] = 0;
		lastDataSize[i] = 0;
	}

	//�X���b�h�ŕ����Ď�M���Ă�
	//��M
	boost::thread_group receiveThreads;
	for (int i = 0; i < recieveServers.size(); i++)
	{
		receiveThreads.create_thread(boost::bind(&BoostUdpServer::startServer, &*recieveServers[i], io_service, 34011 + i));
	}


	//-----------------------------------
	//PCL
	//-----------------------------------
	PCLManager pclManager;
	vector<pcl::PointCloud<PointType>::Ptr> pointClouds;

	for (int i = 0; i < NUM_KINECT; i++)
	{
		pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>());
		pointClouds.push_back(pointCloud);
	}

	//-----------------------------------
	//Mail Loop
	//-----------------------------------
	while (_running)
	{
		workBegin();

		for (int i = 0; i < NUM_KINECT; i++)
		{
			if (recieveServers[i]->isEnablePointCloud)
			{
				//kinect���̃N���E�h
				vector<PointType> points = recieveServers[i]->getPointCloud();
				copy(points.begin(), points.end(), back_inserter(pointClouds[i]->points));
				cout << "PointCloud[" << i << "]---Size: " << pointClouds[i]->points.size() * 6 << endl;
			}
			else {
				pointClouds[i]->clear();
			}
		}

		//�L���ȃf�[�^�����邩�ǂ����`�F�b�N
		int counter = 0;
		for (int i = 0; i < NUM_KINECT; i++)
		{
			int curDataSize = pointClouds[i]->points.size();
			if (curDataSize > 100 && curDataSize != lastDataSize[i]) {
				counter += 1;
				lastDataSize[i] = pointClouds[i]->points.size();
			}
			else
			{
				deathCounter[i] += 1;
				if (deathCounter[i] > 20)
				{
					cout << "-----Points Clear: [" << i << "]" << endl;
					recieveServers[i]->clearPoints();
					deathCounter[i] = 0;
				}
			}
		}

		//�L���ȃf�[�^����ł�����Ώ����J�n
		if (counter > 0)
		{
			//-----------------------------------
			//���
			//-----------------------------------
			pclManager.update(pointClouds);

			//-----------------------------------
			//�f�[�^�擾
			//-----------------------------------
			//�}�[�W�����|�C���g�N���E�h
			pcl::PointCloud<PointType>::Ptr margedCloud(new pcl::PointCloud<PointType>());
			margedCloud = pclManager.getMargedPoints();
			//�d�S�̍��W
			vector<Eigen::Vector4f> centroids = pclManager.getCentroids();


			//-----------------------------------
			//�f�[�^���M
			//-----------------------------------
			//���M�X���b�h�ȂǗ��Ă�
			boost::thread_group sendThreads;
			vector<boost::shared_ptr<BoostUdpServer>> sendServers;
			for (int i = 0; i < 2; i++)
			{
				boost::shared_ptr<BoostUdpServer> svr(new BoostUdpServer(io_service));
				sendServers.push_back(svr);
			}


			//----------�|�C���g�N���E�h�̑��M
			vector<unsigned char> pointSendBuf = convertPoint(margedCloud);
			cout << "Point Send Size: " << pointSendBuf.size() << endl;
			sendThreads.create_thread(boost::bind(&BoostUdpServer::send, &*sendServers[0], io_service, pointSendBuf, sendIpaddress.data(), depthSendtPort, 0, 0));
			margedCloud.reset();


			//----------���̂̏d�S�ʒu���̑��M
			vector<unsigned char> objPointSendBuf(0);
			for (int i = 0; i < centroids.size(); i++)
			{
				vector<unsigned char> buf = convertPoint(centroids[i]);
				copy(buf.begin(), buf.end(), back_inserter(objPointSendBuf));
			}
			cout << "Object Position Send Size: " << objPointSendBuf.size() << endl;
			sendThreads.create_thread(boost::bind(&BoostUdpServer::send, &*sendServers[1], io_service, objPointSendBuf, sendIpaddress.data(), positionSendPort, 0, 0));



			//Visualize
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr showCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
			vector<pcl::PointCloud < pcl::PointXYZRGB>::Ptr> coloredClouds;
			for (int i = 0; i < pointClouds.size(); i++)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc(new pcl::PointCloud<pcl::PointXYZRGB>());
				pcl::copyPointCloud(*pointClouds[i], *cc);
				coloredClouds.push_back(cc);
				if (i % 2 == 0)
				{
					for (int j = 0; j < coloredClouds[i]->points.size(); j++)
					{
						coloredClouds[i]->points[j].r = 255;
						coloredClouds[i]->points[j].g = 0;
						coloredClouds[i]->points[j].b = 0;
					}
				} else {
					for (int j = 0; j < coloredClouds[i]->points.size(); j++)
					{
						coloredClouds[i]->points[j].r = 0;
						coloredClouds[i]->points[j].g = 255;
						coloredClouds[i]->points[j].b = 0;
					}
				}
				
				*showCloud += *coloredClouds[i];
				coloredClouds[i]->clear();
			}
			viewer->updatePointCloud(showCloud);
			viewer->spinOnce();
			


			//���M�X���b�h�̃W���C��
			sendThreads.join_all();
		}


		for (int i = 0; i < pointClouds.size(); i++)
		{
			pointClouds[i]->clear();
		}

		Sleep(1);
		workEnd();
	}

	//�I������
	io_service->stop();
	receiveThreads.join_all();
}

std::vector<unsigned char> App::convertPolygonData(pcl::PolygonMesh & mesh)
{
	//�f�o�b�O�p
	std::vector<float> sendPoints;

	//���M�p
	std::vector<unsigned char> sendBuf(0);
	int sendSize = mesh.polygons.size();

	for (int polygon = 0; polygon < sendSize; polygon++)
	{
		//���W���ɏ���
		for (int vertex = 0; vertex < mesh.polygons[polygon].vertices.size(); vertex++)
		{
			//���W�̃C���f�b�N�X���擾
			int index = mesh.polygons[polygon].vertices[vertex];
			//.data����Y���̃f�[�^������Ă���
			//.data�̒��́A�@���Ƃ��̏��������ĂāA���̍\����.cloud.fields�Ɋi�[����Ă�
			//PointXYZRGB�̏ꍇ�A.data��48�ň�̍��W�̏��ɊY��
			//X���̍��W��float�Ȃ̂ŁA�S������Əd��������short�Ƃ��ɕϊ����Ă��瑗��

			//�܂��C���f�b�N�X�̓���12�o�C�g�����W�n�ŁA4�o�C�g����X, Y, Z�ƂȂ��Ă�̂�
			//4��؂�ŏ������ĕϊ��p��union�^�ϐ��Ɋi�[���Ă�
			DataConverter dataConverter;
			for (int vertexOffset = 0; vertexOffset < 12; vertexOffset += 4)
			{
				for (int dataSize = 0; dataSize < 4; dataSize++)
				{
					dataConverter.data[dataSize] = mesh.cloud.data[index * mesh.cloud.point_step + vertexOffset + dataSize];
				}
				float fVal = dataConverter.fValue;
				sendPoints.push_back(fVal);
				short sVal = fVal * 1000;
				sendBuf.push_back(sVal);
				sendBuf.push_back(sVal >> 8);
			}
		}
	}

	return sendBuf;
}

vector<unsigned char> App::convertPoint(pcl::PointCloud<PointType>::Ptr cloud) {

	//���M�p
	std::vector<unsigned char> sendBuf(0);
	int sendSize = cloud->points.size();

	if (cloud->points.size() == 0)
	{
		return sendBuf;
	}

	//�f�o�b�O�p
	std::vector<float> sendPoints;

	for (int i = 0; i < cloud->points.size(); i++)
	{
		PointType p = cloud->points[i];
		//X
		short x = p.x * 1000;
		sendBuf.push_back(x);
		sendBuf.push_back(x >> 8);
		sendPoints.push_back(x);
		//Y
		short y = p.y * 1000;
		sendBuf.push_back(y);
		sendBuf.push_back(y >> 8);
		sendPoints.push_back(y);
		//Z
		short z = p.z * 1000;
		sendBuf.push_back(z);
		sendBuf.push_back(z >> 8);
		sendPoints.push_back(z);
	}
	return sendBuf;
}

vector<unsigned char> App::convertPoint(Eigen::Vector4f p) {
	//���M�p
	std::vector<unsigned char> sendBuf(0);

	//X
	short x = p.x() * 1000;
	sendBuf.push_back(x);
	sendBuf.push_back(x >> 8);
	//Y
	short y = p.y() * 1000;
	sendBuf.push_back(y);
	sendBuf.push_back(y >> 8);
	//Z
	short z = p.z() * 1000;
	sendBuf.push_back(z);
	sendBuf.push_back(z >> 8);

	return sendBuf;
}

void App::workBegin() {
	workBeginTime = boost::posix_time::microsec_clock::local_time();
}

void App::workEnd() {
	boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
	workDuration = curTime - workBeginTime;
	cout << "Work Duration: " << workDuration.total_milliseconds() << endl;
}