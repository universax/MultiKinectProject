#include "stdafx.h"
#include "KinectManager.h"

HRESULT KinectManager::init() {
	//-----------------------------------------------
	//Kinect											 
	//-----------------------------------------------
	HRESULT hr = S_FALSE;
	hr = GetDefaultKinectSensor(&kinect);
	if (FAILED(hr)) {
		cerr << "ERROR: GetDefaultKinectSensor" << endl;
		return hr;
	}
	hr = kinect->Open();
	if (FAILED(hr))
	{
		cerr << "ERROR: IKinectSensor::Open()" << endl;
		return hr;
	}

	//-----------------------------------------------
	//Coordinate mapper											 
	//-----------------------------------------------
	hr = kinect->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hr))
	{
		cerr << "ERROR: ICoordinateMapper::get_CoordinateMapper()" << endl;
		return hr;
	}


	//-----------------------------------------------
	//Color											 
	//-----------------------------------------------
	CComPtr<IColorFrameSource> colorFrameSource;
	hr = kinect->get_ColorFrameSource(&colorFrameSource);
	if (FAILED(hr))
	{
		cerr << "ERROR: IColorFrameSource::get_ColorFrameSource()" << endl;
		return hr;
	}
	colorFrameSource->OpenReader(&colorFrameReader);

	CComPtr<IFrameDescription> colorFrameDescription;
	hr = colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription);
	if (FAILED(hr))
	{
		cerr << "ERROR: IFrameDescription::CreateFrameDescription()" << endl;
		return hr;
	}
	colorFrameDescription->get_Width(&colorWidth);
	colorFrameDescription->get_Height(&colorHeight);
	colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);

	colorBuffer.resize(colorWidth*colorHeight*colorBytesPerPixel);


	//-----------------------------------------------
	//Depth											 
	//-----------------------------------------------
	depthBuffer.resize(bufferSize);
	CComPtr<IDepthFrameSource> pDepthSource;
	hr = kinect->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hr))
	{
		cerr << "ERROR: IDepthFrameSource::get_DepthFrameSource()" << endl;
		return hr;
	}
	hr = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hr))
	{
		cerr << "ERROR: IDepthFrameReader::OpenReader()" << endl;
		return hr;
	}
	isFrameNew = false;

	//-----------------------------------------------
	//BodyIndex											 
	//-----------------------------------------------
	CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
	hr = kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource);
	if (FAILED(hr))
	{
		cerr << "ERROR: IBodyIndexFrameSource::get_BodyIndexFrameSource()" << endl;
		return hr;
	}
	hr = bodyIndexFrameSource->OpenReader(&pBodyIndexFrameReader);
	if (FAILED(hr))
	{
		cerr << "ERROR: IBodyIndexFrameReader::OpenReader()" << endl;
		return hr;
	}
	bodyIndexBuffer.resize(bufferSize);

	return hr;
}


void KinectManager::update() {
	isFrameNew = updateDepthFrame();
	if (isFrameNew)
	{
		//Color‰æ‘œ‚É‘Î‰ž‚µ‚½CameraPoint”z—ñ¶¬
		//mapColorSpaceToCameraSpace(camSpaceBuf);
	}
	
}

//--------------------------------------------------------------------------------------------
void KinectManager::getDepthBuf(vector<UINT16> &output)
{
	output.clear();
	copy(depthBuffer.begin(), depthBuffer.end(), back_inserter(output));
}


//--------------------------------------------------------------------------------------------
bool KinectManager::updateDepthFrame() {
	HRESULT hr = S_FALSE;
	CComPtr<IDepthFrame> pDepthFrame;
	hr = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		pDepthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
		return true;
	}
	return false;
}

bool KinectManager::updateColorFrame() {
	HRESULT hr = S_FALSE;
	CComPtr<IColorFrame> colorFrame;
	hr = colorFrameReader->AcquireLatestFrame(&colorFrame);
	if (SUCCEEDED(hr))
	{
		colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);
		return true;
	}
	return false;
}

bool KinectManager::updateBodyIndexFrame() {
	HRESULT hr = S_FALSE;
	CComPtr<IBodyIndexFrame> pBodyIndexFrame;
	hr = pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	if (SUCCEEDED(hr))
	{
		pBodyIndexFrame->CopyFrameDataToArray(bodyIndexBuffer.size(), &bodyIndexBuffer[0]);
		return true;
	}
	return false;
}