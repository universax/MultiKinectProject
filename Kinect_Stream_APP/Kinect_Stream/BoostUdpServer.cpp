#include "stdafx.h"
#include "BoostUdpServer.h"
#include "PCLManager.h"

void BoostUdpServer::send_handler(const boost::system::error_code & error, std::size_t len)
{
	//mutex_lock.lock();
	//std::cout << "--------------------------------Send: " << len << std::endl;
	//mutex_lock.unlock();
}

void BoostUdpServer::recv_handler(const boost::system::error_code & error, std::size_t len)
{

	if (!error || error == boost::asio::error::message_size)
	{
		//std::cout << "Receive: " << len << std::endl;
		convertToPoints(&_receivedBuf[0], len, pointCloudData);
		//std::cout << "-------------------------------!!!!: " << (short)(_receivedBuf[0] + _receivedBuf[1]*256) << std::endl;
		if (len < 60000)
		{
			pointCloudDataArray.push_back(pointCloudData);
			pointCloudData.clear();

			isEnablePointCloud = true;
		}
		startReceive();
	}
}


void BoostUdpServer::startServer(boost::shared_ptr<boost::asio::io_service> io_service, unsigned short port)
{
	_port = port;
	_sock.open(boost::asio::ip::udp::v4());
	_sock.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
	isEnablePointCloud = false;
	startReceive();
	io_service->run();
}

void BoostUdpServer::startReceive()
{
	if (!_sock.is_open())
	{
		_sock.open(boost::asio::ip::udp::v4());
		startReceive();
	}
	else {
		_sock.async_receive_from(boost::asio::buffer(_receivedBuf), _remoteEndpoint,
			boost::bind(
				&BoostUdpServer::recv_handler,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
				));
	}
}


void BoostUdpServer::send(boost::shared_ptr<boost::asio::io_service> io_service, const std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port) {

	std::vector<std::vector<unsigned char>> pendingBufGroup;
	//データを分割
	int maxDatasize = 54000;
	int dataSize = sendBuf.size();
	int numDataGroup = dataSize / maxDatasize + 1;

	//std::cout << ">>>>>>>>>>Send Start: " << numDataGroup << std::endl;
	mutex_lock.lock();
	std::cout << "-------------------------------- Total: " << sendBuf.size() << std::endl;
	mutex_lock.unlock();
	for (int i = 0; i < numDataGroup; i++)
	{
		std::vector<unsigned char> b;

		//頭の4バイトに、データ総数をいれとく
		DataConverter converter;
		converter.iValue = sendBuf.size();
		for (int i = 0; i < 4; i++)
		{
			b.push_back(converter.data[i]);
		}


		//中身をぶちこむ
		if (dataSize < maxDatasize)
		{
			std::copy(sendBuf.begin() + maxDatasize * i, sendBuf.end(), std::back_inserter(b));
		}
		else {
			std::copy(sendBuf.begin() + maxDatasize * i, sendBuf.begin() + maxDatasize * (i + 1), std::back_inserter(b));
			dataSize -= maxDatasize;
		}
		pendingBufGroup.push_back(b);
	}


	//送信
	if (!_sock.is_open())
	{
		_sock.open(boost::asio::ip::udp::v4());
		_sock.bind(boost::asio::ip::udp::endpoint());
	}

	for (int i = 0; i < pendingBufGroup.size(); i++)
	{
		mutex_lock.lock();
		std::cout << "Start Send Size: " << pendingBufGroup[i].size() << std::endl;
		mutex_lock.unlock();
		_sock.get_io_service().dispatch(boost::bind(&BoostUdpServer::sendSync, this, pendingBufGroup[i], ipaddr, port));
		_sock.get_io_service().poll();
		Sleep(1);
	}
}

void BoostUdpServer::sendAsync(const std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port) {
	_sock.async_send_to(boost::asio::buffer(sendBuf),
		boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ipaddr), port),
		boost::bind(
			&BoostUdpServer::send_handler,
			this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
			));
}

void BoostUdpServer::sendSync(std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port) {
	size_t sendSize = _sock.send_to(boost::asio::buffer(sendBuf),
		boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ipaddr), port));
	mutex_lock.lock();
	std::cout << "Finish Send Size: " << sendSize << std::endl;
	mutex_lock.unlock();
}

void BoostUdpServer::convertToPoints(unsigned char *buf, size_t len, std::vector<PointType> &outputPoints) {
	for (int i = 0; i < len; i += 6)
	{
		PointType point;
		point.x = (short)(buf[i] + buf[i + 1] * 256) * 0.001;
		point.y = (short)(buf[i + 2] + buf[i + 3] * 256) * 0.001;
		point.z = (short)(buf[i + 4] + buf[i + 5] * 256) * 0.001;
		//switch (_port)
		//{
		//case 34011:
		//	point.r = 255;
		//	point.g = 0;
		//	point.b = 0;
		//	break;
		//case 34012:
		//	point.r = 0;
		//	point.g = 255;
		//	point.b = 0;
		//	break;
		//case 34013:
		//	point.r = 0;
		//	point.g = 0;
		//	point.b = 255;
		//	break;
		//case 34014:
		//	point.r = 255;
		//	point.g = 0;
		//	point.b = 255;
		//	break;
		//default:
		//	point.r = 255;
		//	point.g = 255;
		//	point.b = 255;
		//	break;
		//}

		outputPoints.push_back(point);
	}
}