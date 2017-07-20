#pragma once
class App
{
public:
	App(){}
	~App(){}

	HRESULT init();
	
	void run();

private:
	//UDP SEND
	std::vector<unsigned char> convertPolygonData(pcl::PolygonMesh &mesh);
	vector<unsigned char> convertPoint(pcl::PointCloud<PointType>::Ptr cloud);
	vector<unsigned char> convertPoint(Eigen::Vector4f p);

	//Network Settings
	bool loadNeworkSettings();
	int depthSendtPort = 0;
	int positionSendPort = 0;
	string sendIpaddress = "";

	//Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//Util
	void workBegin();
	void workEnd();

	//Event
	bool _running;
	int lastDataSize[NUM_KINECT];
	int deathCounter[NUM_KINECT];

	//FPS‘ª’è
	boost::posix_time::ptime workBeginTime;
	boost::posix_time::time_duration workDuration;
};