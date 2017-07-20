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
	//��M
	//-----------------------------------
	//��M�J�n
	void startServer(boost::shared_ptr<boost::asio::io_service> io_service, unsigned short port);

	//-----------------------------------
	//Point Cloud Data
	//-----------------------------------
	//�ʂ̃f�[�^
	std::vector<PointType> pointCloudData;
	//�O���[�s���O�������
	std::vector<std::vector<PointType>> pointCloudDataArray;

	//��M�����|�C���g�N���E�h�̈����n��
	inline std::vector<PointType> getPointCloud() {
		std::vector<PointType> returnPoints = pointCloudDataArray.back();
		pointCloudDataArray.clear();
		pointCloudDataArray.push_back(returnPoints);
		//isEnablePointCloud = false;
		return returnPoints;
	}

	//�|�C���g�N���E�h�̃N���A
	inline void clearPoints() {
		pointCloudDataArray.clear();
		isEnablePointCloud = false;
	}
	bool isEnablePointCloud;

	//-----------------------------------
	//���M
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

