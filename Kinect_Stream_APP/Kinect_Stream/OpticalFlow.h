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
	//�ݒ�l
	int maxFeatureCount = 600;

	//�v�Z�Ώۂ̌��݂̃t���[��
	Mat curMat;
	//�t���[�Z�o����ۂ̈��O�̃t���[��
	Mat prevMat;

	//�Z�o���ʂ�������
	vector<Point2f> points[2];
	vector<CameraSpacePoint> sendPoints;

	//���Z�b�g����悤�̃t���O
	bool isNeedToReset;

	//�ŏ��ɓ����Ă����摜�͏������i�g���b�L���O��������_��߂܂���j���K�v
	bool initPoints(UMat &inputMat);
	TermCriteria termcrit;
	vector<uchar> status;

	//�����_�̃��Z�b�g�J�E���^
	int counter;

	vector<unsigned char> createSendBuffer(vector<Point2f> inputPoints);

	vector<uchar> createBytesWithCameraSpacePoints(vector<CameraSpacePoint>& points);

	//�ŏI�I�ȃt���[�Ɗe���W
};

struct FlowData {
	UINT16 x, y, z;
	unsigned char fx, fy, fz;
};