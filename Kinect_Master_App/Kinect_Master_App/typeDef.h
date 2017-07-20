#pragma once

using namespace std;
using namespace std::chrono;

//Kinect
#define NUM_KINECT 2

union DataConverter {
	unsigned char data[4];
	short sValue[2];
	float fValue;
	int iValue;
};

typedef pcl::PointXYZ PointType;
typedef pcl::PointNormal PointNormalType;
