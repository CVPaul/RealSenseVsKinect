#ifndef KINECTFORWINDOWS_H
#define KINECTFORWINDOWS_H

#define MM_PER_M 1000

#include <iostream>
#include <vector>
#include <string>

#include <stdio.h>
#include <stdlib.h>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include "pxcbase.h"

#include <Kinect.h>

#include "PointCloud.h"

using namespace cv;
using std::vector;
using std::string;
using std::cout;
using std::cin;


extern int gFraps;
extern double preTime;
extern double curTime;


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease);
/*brief: remains the depth pixels that fall in [MIN_LINIT MAX_LIMIT]*/ 
void simpleBackGroudCut(cv::Mat& DepMap, int MIN_LIMIT, int MAX_LIMIT);
/*brief: count valid depth pixels that fall in [MIN_LINIT MAX_LIMIT]*/ 
int count_valid(cv::Mat depth,int MIN_LIMIT,int MAX_LIMIT);
void save_depth(int index,std::string prefix, cv::Mat depth_save,cv::Mat mask);
void load_depth(int index, std::string prefix, cv::Mat& depth_load,cv::Mat& mask);
/* highlight depth pixels that fall in [low_range high_range]*/
cv::Mat properShowformat(UINT16* depBuf, int height,
	int width, int low_range, int high_range, bool cut);

//------------------------------------------------
void getFraps();

class capKinect
{
public:
	static const int cDepthWidth = 512; // 320
	static const int cDepthHeight = 424; // 240

	static const int cColorWidth = 0;// 1920; // 640
	static const int cColorHieght = 0;// 1080; // height

	capKinect();
	~capKinect();
	HRESULT initKinect(); // init Kinect Capture
	cv::Mat update(cv::Mat& depth_show);// update
	cv::Mat capture(UINT16* pBuffer, int nWidth, int nHeight,cv::Mat& depth_show,
		USHORT nMinDepth, USHORT nMaxDepth);// capture the depth data of Kinect
	int getPointCloud(cv::Mat DepMap, PointCloud& world);// convert to PointCloud format for conveniet usage
	int saveDepthAsPointCloud(int index, std::string prefix);// save to PointCloud format file

	void put_isPaused(BOOLEAN isPuased){m_pDepthReader->put_IsPaused(isPuased);}
private:
	IKinectSensor*		m_pKinect; // current Kinect Device
	ICoordinateMapper*	m_pMapper; // coordinate mapper
	IDepthFrameReader*	m_pDepthReader; // Depth Reader
	IColorFrameReader*	m_pColorReader; // Color Reader
	uchar*				m_pColorRGBX;
	uchar*				m_pDepthRGBX;
	//RGBQUAD*			m_pColorRGBX;
	//RGBQUAD*			m_pDepthRGBX;
};
#endif /*KinectForWindows.h*/
