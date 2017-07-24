#pragma once

using namespace std;
using namespace cv;

class OpticalFlow
{
public:
	OpticalFlow();
	~OpticalFlow() {}
	void update(UMat &inputMat);
	void draw(UMat &drawMat);
	vector<Point2f> getFlowBeginPoints() { return points[0]; }
	vector<Point2f> getFlowEndPoints() { return points[1]; }

private:
	//設定値
	int maxFeatureCount = 600;

	//計算対象の現在のフレーム
	Mat curMat;
	//フロー算出する際の一つ手前のフレーム
	Mat prevMat;

	//算出結果を入れるやつ
	vector<Point2f> points[2];
	vector<CameraSpacePoint> sendPoints;

	//リセットするようのフラグ
	bool isNeedToReset;

	//最初に入ってきた画像は初期化（トラッキングする特徴点を捕まえる）が必要
	bool initPoints(UMat &inputMat);
	TermCriteria termcrit;
	vector<uchar> status;

	//特徴点のリセットカウンタ
	int counter;

	vector<unsigned char> createSendBuffer(vector<Point2f> inputPoints);

	vector<uchar> createBytesWithCameraSpacePoints(vector<CameraSpacePoint>& points);

	//最終的なフローと各座標
};

struct FlowData {
	UINT16 x, y, z;
	unsigned char fx, fy, fz;
};