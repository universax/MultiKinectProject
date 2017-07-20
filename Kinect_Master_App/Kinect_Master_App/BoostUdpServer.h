#pragma once
using namespace boost::asio::ip;
class BoostUdpServer{
public:
	BoostUdpServer(boost::shared_ptr<boost::asio::io_service> io_service) : _sock(*io_service) {}
	~BoostUdpServer() {
		_sock.close();
		std::cout << "<< Thread End >>" << std::endl;
	}

	//-----------------------------------
	//受信
	//-----------------------------------
	//受信開始
	void startServer(boost::shared_ptr<boost::asio::io_service> io_service, unsigned short port);

	//-----------------------------------
	//Point Cloud Data
	//-----------------------------------
	//個別のデータ
	std::vector<PointType> pointCloudData;
	//グルーピングしたやつ
	std::vector<std::vector<PointType>> pointCloudDataArray;

	//受信したポイントクラウドの引き渡し
	inline std::vector<PointType> getPointCloud() {
		std::vector<PointType> returnPoints = pointCloudDataArray.back();
		pointCloudDataArray.clear();
		pointCloudDataArray.push_back(returnPoints);
		//isEnablePointCloud = false;
		return returnPoints;
	}

	//ポイントクラウドのクリア
	inline void clearPoints() {
		pointCloudDataArray.clear();
		isEnablePointCloud = false;
	}
	bool isEnablePointCloud;

	//-----------------------------------
	//送信
	//-----------------------------------
	void send(boost::shared_ptr<boost::asio::io_service> io_service, std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port, short imageWidth, short imageHeight);


private:
	void startReceive();
	void send_handler(const boost::system::error_code& error, std::size_t len);
	void recv_handler(const boost::system::error_code & error, std::size_t len);
	void sendAsync(std::vector<unsigned char> &sendBuf, const char * ipaddr, unsigned short port);
	void sendSync(std::vector<unsigned char>& sendBuf, const char * ipaddr, unsigned short port);
	void convertToPoints(unsigned char *buf, size_t len, std::vector<PointType> &outputPoints);

	udp::socket _sock;
	udp::endpoint _remoteEndpoint;
	boost::array<unsigned char, 70000> _receivedBuf;
	size_t _port;
	int lastReceivedDataSize;
};

