#include "stdafx.h"
#include "BoostUdpServer.h"


void BoostUdpServer::send_handler(const boost::system::error_code & error, std::size_t len)
{
	//std::cout << "--------------------------------Send: " << len << std::endl;
}

void BoostUdpServer::recv_handler(const boost::system::error_code & error, std::size_t len)
{
	if (!error || error == boost::asio::error::message_size)
	{
		//最初の4バイトはデータ総数が入ってるので、それをチェック
		DataConverter converter;
		for (int i = 0; i < 4; i++)
		{
			converter.data[i] = _receivedBuf[i];
		}
		int curDataSize = converter.iValue;

		//頭の4バイトの内容が変わってたら
		if (curDataSize != lastReceivedDataSize)
		{
			//カタマリを保存してる配列に、ためたデータをいれて
			pointCloudDataArray.push_back(pointCloudData);

			//ためてたやつはこの区切りポイントでいったんクリア
			pointCloudData.clear();

			//データ取得OKのフラグをたてて
			isEnablePointCloud = true;

			//クラス変数として保存してあった前回のヘッダーの内容を更新し
			lastReceivedDataSize = curDataSize;
		}

		//受信したデータをためる何らかの処理（上記の更新のあとにこれをやらないと意味ないです）
		//ここではポイント座標系に変換してから保存する関数をたたいてます
		convertToPoints(&_receivedBuf[4], len - 4, pointCloudData);

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


void BoostUdpServer::send(boost::shared_ptr<boost::asio::io_service> io_service,  std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port, short imageWidth, short imageHeight) {

	std::vector<std::vector<unsigned char>> pendingBufGroup;
	//データを分割
	int maxDatasize = 54000;
	int dataSize = sendBuf.size();
	int numDataGroup = dataSize / maxDatasize + 1;

	//std::cout << ">>>>>>>>>>Send Start: " << numDataGroup << std::endl;
	//std::cout << "--------------------------------Count: " << sendCounter << std::endl;
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
		
		if (imageWidth > 0 && imageHeight > 0)
		{
			//その後ろに2バイトずつ、ポイントクラウドの縦横幅を入れておく
			converter.iValue = 0;
			converter.sValue[0] = imageWidth;
			converter.sValue[1] = imageHeight;
			for (int i = 0; i < 4; i++)
			{
				b.push_back(converter.data[i]);
			}
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
	cout << "Sending Gourp: " << numDataGroup << endl;
	for (int i = 0; i < pendingBufGroup.size(); i++)
	{
		//std::cout << "Send Size: " << pendingBufGroup[i].size() << std::endl;
		io_service->post(boost::bind(&BoostUdpServer::sendAsync, this, pendingBufGroup[i], ipaddr, port));
		Sleep(1);
	}
}

void BoostUdpServer::sendAsync(std::vector<unsigned char> &sendBuf, const char *ipaddr, unsigned short port) {
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
	//std::cout << "Send Size: " << sendSize << std::endl;
}

void BoostUdpServer::convertToPoints(unsigned char *buf, size_t len, std::vector<PointType> &outputPoints) {
	for (int i = 0; i < len; i += 6)
	{
		PointType point;
		point.x = (short)(buf[i] + buf[i + 1] * 256) * 0.001;
		point.y = (short)(buf[i + 2] + buf[i + 3] * 256) * 0.001;
		point.z = (short)(buf[i + 4] + buf[i + 5] * 256) * 0.001;
		outputPoints.push_back(point);
	}
}