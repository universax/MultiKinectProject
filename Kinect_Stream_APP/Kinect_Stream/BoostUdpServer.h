#pragma once

using namespace boost::asio::ip;

class BoostUdpServer{
public:
	BoostUdpServer(boost::shared_ptr<boost::asio::io_service> io_service) : _sock(*io_service) {
		
	}
	~BoostUdpServer() {
		_sock.close();
		std::cout << "<< Thread End >>" << std::endl;
	}


	void startServer(boost::shared_ptr<boost::asio::io_service> io_service, unsigned short port);
	
	void send(boost::shared_ptr<boost::asio::io_service> io_service, const std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port);

	inline std::vector<PointType> getPointCloud() {
		std::vector<PointType> returnPoints = pointCloudDataArray.back();
		pointCloudDataArray.clear();
		isEnablePointCloud = false;
		return returnPoints;
	}
	bool isEnablePointCloud;


	//Point Cloud Data
	std::vector<PointType> pointCloudData;	//個別のデータ
	std::vector<std::vector<PointType>> pointCloudDataArray;	//リスト上にまとめたやつ
private:
	void startReceive();
	void send_handler(const boost::system::error_code& error, std::size_t len);
	void recv_handler(const boost::system::error_code & error, std::size_t len);
	void sendAsync(const std::vector<unsigned char> &sendBuf, const char * ipaddr, unsigned short port);
	void sendSync(std::vector<unsigned char>& sendBuf, const char * ipaddr, unsigned short port);
	void convertToPoints(unsigned char *buf, size_t len, std::vector<PointType> &outputPoints);

	udp::socket _sock;
	udp::endpoint _remoteEndpoint;
	boost::array<unsigned char, 70000> _receivedBuf;
	size_t _port;


	//Debug
	boost::mutex mutex_lock;
	int sendCounter = 0;
};

union DataConverter {
	unsigned char data[4];
	short sValue[2];
	float fValue;
	int iValue;
};