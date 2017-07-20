#include "stdafx.h"
#include "KinectPosture.h"

using namespace std;
using namespace boost::property_tree;
using namespace boost::property_tree::xml_parser;

KinectPosture::KinectPosture() {
	if (!loadKinectPostureData())
	{
		x = 0;
		y = 0;
		z = 0;
		pitch = 0;
		yaw = 0;
		roll = 0;
	}
}

bool KinectPosture::loadKinectPostureData()
{
	ptree pt;
	read_xml("Kinect_Setting.xml", pt);
	x = pt.get_optional<double>("kinect_posture.x").get();
	y = pt.get_optional<double>("kinect_posture.y").get();
	z = pt.get_optional<double>("kinect_posture.z").get();
	pitch = pt.get_optional<double>("kinect_posture.pitch").get();
	yaw = pt.get_optional<double>("kinect_posture.yaw").get();
	roll = pt.get_optional<double>("kinect_posture.roll").get();

	return true;
}

bool KinectPosture::saveKinectPostureData()
{
	ptree pt;
	pt.put("kinect_posture.x", x);
	pt.put("kinect_posture.y", y);
	pt.put("kinect_posture.z", z);
	pt.put("kinect_posture.pitch", pitch);
	pt.put("kinect_posture.yaw", yaw);
	pt.put("kinect_posture.roll", roll);

	write_xml("Kinect_Setting.xml", pt, locale(), xml_writer_make_settings<string>(' ', 2));
	return false;
}

double KinectPosture::addX(double val)
{
	x += val;
	return x;
}

double KinectPosture::addY(double val)
{
	y += val;
	return y;
}

double KinectPosture::addZ(double val)
{
	z += val;
	return z;
}

double KinectPosture::addPitch(double val)
{
	pitch += val;
	return pitch;
}

double KinectPosture::addYaw(double val)
{
	yaw += val;
	return yaw;
}

double KinectPosture::addRoll(double val)
{
	roll += val;
	return roll;
}
