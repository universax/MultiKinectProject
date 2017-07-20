#pragma once
#include "Singleton.h"

class KinectPosture: public Singleton<KinectPosture>
{
public:
	bool loadKinectPostureData();
	bool saveKinectPostureData();

	inline double getX() { return x; }
	inline double getY() { return y; }
	inline double getZ() { return z; }
	inline double getPitch() { return pitch; }
	inline double getYaw() { return yaw; }
	inline double getRoll() { return roll; }

	double addX(double val);
	double addY(double val);
	double addZ(double val);
	double addPitch(double val);
	double addYaw(double val);
	double addRoll(double val);

private:
	friend class Singleton<KinectPosture>;
	KinectPosture();
	~KinectPosture(){}

	//Posture
	double x, y, z, pitch, yaw, roll;
};

