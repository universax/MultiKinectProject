#include "stdafx.h"
#include "OpticalFlow.h"
#include "KinectManager.h"
#include "BoostUdpServer.h"

OpticalFlow::OpticalFlow() {
	//初期化
	for (int i = 0; i < 2; i++)
	{
		points[i].clear();
	}
	isNeedToReset = true;
	termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.01);

	counter = 0;
}

void OpticalFlow::update(UMat &inputMat) {
	//入力チェック
	if (inputMat.empty())
	{
		return;
	}

	////まず白黒にしてやる
	//cvtColor(inputMat, curMat, CV_BGR2GRAY);
	inputMat.copyTo(curMat);

	//初回は初期化が必要
	if (isNeedToReset)
	{
		isNeedToReset = !initPoints(inputMat);
	}
	else if(!points[0].empty()) {
		//vector<float>err;
		Mat err;
		if (prevMat.empty())
		{
			curMat.copySize(prevMat);
		}
		//必要なのは、
		//ひとつ前の画像と、その特徴点の入った配列
		//現在の画像と、その特徴点の入った配列
		try
		{
			calcOpticalFlowPyrLK(prevMat, curMat, points[0], points[1], status, err);
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			isNeedToReset = true;
		}
	}

	swap(points[1], points[0]);
	swap(curMat, prevMat);

	counter += 1;
	if (counter > 33)
	{
		//for (int i = 0; i < 2; i++)
		//{
		//	points[i].clear();
		//}
		isNeedToReset = true;
		counter = 0;
	}
}


void OpticalFlow::draw(UMat &drawMat) {
	if (status.empty())
	{
		return;
	}
	for (int i = 0; i < points[1].size(); i++)
	{
		
		if (status[i])
		{
			Point2f dist = points[1][i] - points[0][i];
			float distance = sqrt(dist.x * dist.x + dist.y * dist.y);
			if (distance < 12 && distance > 2)
			{
				line(drawMat, points[0][i], points[1][i], Scalar(0, 0, 255));
				circle(drawMat, points[0][i], 1, Scalar(0, 0, 255));
				//circle(drawMat, points[1][i], 1, Scalar(0, 255, 0));
			}
			
			
		}
	}
	imshow("Kinect", drawMat);
}

bool OpticalFlow::initPoints(UMat &inputMat) {
	//入力チェック
	if (inputMat.empty())
	{
		return false;
	}
	
	//入力画像の特徴点を算出して保持
	try
	{
		goodFeaturesToTrack(inputMat.getMat(ACCESS_FAST), points[1], maxFeatureCount, 0.01, 10);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return false;
	}
	
	if (points[1].size() > 10)
	{
		try
		{
			cornerSubPix(inputMat, points[1], Size(10, 10), Size(-1, -1), termcrit);
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			return false;
		}
		

		return true;
	}
	
	return false;
}

vector<unsigned char> OpticalFlow::createSendBuffer(vector<Point2f> inputPoints)
{
	//CameraSpaceに変換
	vector<CameraSpacePoint> cp(0);

	vector<unsigned char> buf(0);
	//for (int i = 0; i < inputPoints.size(); i++)
	//{
	//	DataConverter converter;
	//	//X
	//	converter.sValue[0] = (short)inputPoints[i].x * 1000;
	//	for (int j = 0; j < 2; j++)
	//	{
	//		buf.push_back(converter.data[j]);
	//	}
	//	//Y
	//	converter.sValue[0] = (short)inputCloud->points[i].y * 1000;
	//	for (int j = 0; j < 2; j++)
	//	{
	//		buf.push_back(converter.data[j]);
	//	}
	//	//Z
	//	converter.sValue[0] = (short)inputCloud->points[i].z * 1000;
	//	for (int j = 0; j < 2; j++)
	//	{
	//		buf.push_back(converter.data[j]);
	//	}
	//}
	return buf;
}

vector<uchar> OpticalFlow::createBytesWithCameraSpacePoints(vector<CameraSpacePoint>& points)
{
	//送信用の配列を準備
	vector<uchar> buf;

	for (int i = 0; i < points.size(); i += 1)
	{
		DataConverter converter;
		//X
		converter.sValue[0] = points[i].X * 1000;
		for (int j = 0; j < 2; j++)
		{
			buf.push_back(converter.data[j]);
		}
		//Y
		converter.sValue[0] = points[i].Y * 1000;
		for (int j = 0; j < 2; j++)
		{
			buf.push_back(converter.data[j]);
		}
		//Z
		converter.sValue[0] = points[i].Z * 1000;
		for (int j = 0; j < 2; j++)
		{
			buf.push_back(converter.data[j]);
		}
	}

	return buf;
}