#include "KinectForWindows.h"
extern int gFraps = 0;
extern double preTime=0.0;
extern double curTime=0.0;
//------------------------------------------------
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
void getFraps()
{
	curTime = cv::getTickCount();
	double time = (curTime - preTime) / cv::getTickFrequency();
	if (time < 1.0)
	{
		gFraps++;
	}
	else
	{
		preTime = curTime;
		printf("FAPS:%d\n", gFraps);
		gFraps = 0;
	}
}
capKinect::capKinect()
{
	m_pKinect = NULL;
	m_pDepthReader = NULL;
	m_pColorReader=NULL;

	m_pDepthRGBX = new uchar[cDepthWidth*cDepthHeight];
	m_pColorRGBX = new uchar[cColorWidth*cColorHieght];
	//m_pDepthRGBX = new RGBQUAD[cDepthWidth*cDepthHeight];
	//m_pColorRGBX = new RGBQUAD[cColorWidth*cColorHieght];
}

capKinect::~capKinect()
{
	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}
	if(m_pDepthReader)SafeRelease(m_pDepthReader);
	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}
	if(m_pColorReader)SafeRelease(m_pColorReader);
}
HRESULT capKinect::initKinect()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinect);

	if (FAILED(hr)) return hr;
	if (m_pKinect)
	{
		// initialize successed
		IDepthFrameSource *pDepthFrameSource = NULL;
		hr = m_pKinect->Open();
		if (SUCCEEDED(hr))
		{
			hr = m_pKinect->get_DepthFrameSource(&pDepthFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthReader);
		}
		if(pDepthFrameSource)SafeRelease(pDepthFrameSource);
	}
	if (!m_pKinect || FAILED(hr))
	{
		printf("No ready Kinect found!\n");
		return E_FAIL;
	}
	return hr;
}
cv::Mat capKinect::update(cv::Mat& depth_show)
{
	if (!m_pDepthReader) return cv::Mat();
	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthReader->AcquireLatestFrame(&pDepthFrame);
	cv::Mat re;

	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}
		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}
		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)  
			// we are setting nDepthMaxDistance to the extreme potential depth threshold  
			nDepthMaxDistance = USHRT_MAX; //here we set maxDepth as 1000 mm (1 m) to simply cut the back background

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.  
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);  
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if (SUCCEEDED(hr))
		{
			re=capture(pBuffer, nWidth, nHeight, depth_show, nDepthMinReliableDistance, nDepthMaxDistance);
		}

		if(pFrameDescription)SafeRelease(pFrameDescription);
	}

	if(pDepthFrame)SafeRelease(pDepthFrame);
	return re;
}

cv::Mat capKinect::capture(UINT16* pBuffer, int nWidth, int nHeight,cv::Mat& depth_show,
	USHORT nMinDepth, USHORT nMaxDepth)
{
	// Make sure we've received valid data  
	cv::Mat re;

	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		uchar* pRGBX = m_pDepthRGBX;
		//RGBQUAD* pRGBX = m_pDepthRGBX;
		cv::Mat temp = cv::Mat(nHeight,nWidth,CV_16UC1,pBuffer);
		temp.copyTo(re);
		//// end pixel is start + width*height - 1  
		//const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		//while (pBuffer < pBufferEnd)
		//{
		//	USHORT depth = *pBuffer;

		//	// To convert to a byte, we're discarding the most-significant  
		//	// rather than least-significant bits.  
		//	// We're preserving detail, although the intensity will "wrap."  
		//	// Values outside the reliable depth range are mapped to 0 (black).  

		//	// Note: Using conditionals in this loop could degrade performance.  
		//	// Consider using a lookup table instead when writing production code.  
		//	BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);
		//	if(intensity>0)
		//		*pRGBX=255;
		//	else
		//		*pRGBX=0;

		//	++pRGBX;
		//	++pBuffer;
		//}

		//// Draw the data with OpenCV  
		//Mat DepthImage(nHeight, nWidth, CV_8UC1, m_pDepthRGBX);
		//DepthImage.copyTo(depth_show);
		int MIN_LIMIT=600;
		int MAX_LIMIT=1000;
		depth_show=properShowformat(pBuffer,nHeight,nWidth,MIN_LIMIT,MAX_LIMIT,false);
	}
	return re;
}
int capKinect::saveDepthAsPointCloud(int index, std::string prefix)
{
	cv::Mat depth,mask;
	load_depth(index,prefix,depth,mask);
	int pcs=0;
	if(!depth.empty())
	{
		std::string paths[] = { "firstViewDepth", "secondViewDepth", "thirdViewDepth",
		"forthViewDepth", "fifthViewDepth", "sixthViewDepth", "seventhViewDepth" };
		std::string curpath=prefix;
		curpath.append(paths[index]);
		curpath.append(".txt");
		pcs = getPointCloud(depth,world);
		world.save(curpath);
		printf("PointCloud saved in %s!\n",curpath.c_str());
	}
	return pcs;	
}
void simpleBackGroudCut(cv::Mat& DepMap,int MIN_LIMIT, int MAX_LIMIT)
{
	UINT16 *pDep;
	for (int i = 0; i < DepMap.rows; i++)
	{
		pDep = DepMap.ptr<UINT16>(i);
		for (int j = 0; j < DepMap.cols; j++)
		{
			if (pDep[j]<MIN_LIMIT || pDep[j]>MAX_LIMIT)
				pDep[j] = 0;
		}
	}
}
int count_valid(cv::Mat depth,int MIN_LIMIT,int MAX_LIMIT)
{
	UINT16 *pOUT;
	int count = 0;
	for (int i = 0; i < depth.rows; i++)
	{
		pOUT = depth.ptr<UINT16>(i);
		for (int j = 0; j < depth.cols; j++)
		{
			if (pOUT[j]>MIN_LIMIT&&pOUT[j] < MAX_LIMIT)
			{
				count++;
			}
		}
	}
	return count;
}
void save_depth(int index,std::string prefix, cv::Mat depth_save,cv::Mat mask)
{
	std::string paths[] = { "firstViewDepth.xml", "secondViewDepth.xml", "thirdViewDepth.xml",
		"forthViewDepth.xml", "fifthViewDepth.xml", "sixthViewDepth.xml", "seventhViewDepth.xml" };
	std::string curpath;
	if (index >= 7||index<0)
	{
		printf("Error in save_depth,index must smaller than 7,while bigger than -1!\n");
		return;
	}
	curpath = prefix;
	curpath.append(paths[index]);
	cv::FileStorage ofs(curpath, cv::FileStorage::WRITE);
	ofs << "depth" << depth_save;
	ofs.release();
	printf("%s file save finished!\n", curpath.c_str());

	std::string image_path = curpath;
	int pos=image_path.find_last_of('.');
	image_path = image_path.substr(0, pos);

	std::string mask_path = image_path;
	mask_path.append("mask.jpg");

	cv::imwrite(mask_path, mask);
	printf("%s mask file save finished!\n", mask_path.c_str());
}
void load_depth(int index, std::string prefix, cv::Mat& depth_load,cv::Mat& mask)
{
	std::string paths[] = { "firstViewDepth.xml", "secondViewDepth.xml", "thirdViewDepth.xml",
		"forthViewDepth.xml", "fifthViewDepth.xml", "sixthViewDepth.xml", "seventhViewDepth.xml" };
	std::string curpath;
	if (index >= 7 || index<0)
	{
		printf("Error in save_depth,index must sammler than 7,while bigger than -1!\n");
		return;
	}
	curpath = prefix;
	curpath.append(paths[index]);
	cv::FileStorage ifs(curpath, cv::FileStorage::READ);
	ifs["depth"] >> depth_load;
	//printf("%s file load finished!\n", curpath.c_str());
	
	std::string image_path = curpath;
	int pos = image_path.find_last_of('.');
	image_path = image_path.substr(0, pos);

	std::string mask_path = image_path;
	mask_path.append("mask.jpg");

	mask=cv::imread(mask_path);
	//printf("%s mask file load finished!\n", mask_path.c_str());
}
int capKinect::getPointCloud(cv::Mat DepMap, PointCloud& world)
{
	HRESULT hr = m_pKinect->get_CoordinateMapper(&m_pMapper);
	if (FAILED(hr))
	{
		printf("Error!get_CoordinateMapper()\n");
		return -1;
	}
	UINT16 *pDep;
	int count = DepMap.rows*DepMap.cols;
	world.create(count);
	if (world.size !=count)
	{
		printf("PointCloud world.create() failed!\n");
		return -1;
	}
	count = 0; // restar
	for (int y = 0; y < DepMap.rows; y++)
	{
		pDep = DepMap.ptr<UINT16>(y);
		for (int x = 0; x < DepMap.cols; x++)
		{
			DepthSpacePoint dep_xy={ static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = pDep[x];

			if(depth<=0)
			{
				world.pc[count].x = 0.0;
				world.pc[count].y = 0.0;
				world.pc[count].z = 0.0;
				count++;
				continue;
			}

			CameraSpacePoint camera_xyz = { 0.0, 0.0 ,0.0};
			m_pMapper->MapDepthPointToCameraSpace(dep_xy, depth, &camera_xyz);
			world.pc[count].x = camera_xyz.X*MM_PER_M;
			world.pc[count].y = camera_xyz.Y*MM_PER_M;
			world.pc[count].z = camera_xyz.Z*MM_PER_M;

			count++;
		}
	}
	return count;
}
cv::Mat properShowformat(UINT16* depBuf, int height,
	int width, int low_range, int high_range, bool cut)
{
	cv::Mat re;
	re = cv::Mat::zeros(height, width, CV_8UC1);
	uchar *pOUT;
	if (cut)
	{
		int index = 0;
		float range = (high_range - low_range) / 255.0;

		for (int i = 0; i < re.rows; i++)
		{
			pOUT = re.ptr<uchar>(i);
			for (int j = 0; j < re.cols; j++)
			{
				int depth = (int)depBuf[index];
				if (depth>low_range&&depth<high_range)
					pOUT[j] = depth%256;
				index++;
			}
		}
	}
	else
	{
		int index = 0;
		float range = (high_range - low_range) / 255.0;

		for (int i = 0; i < re.rows; i++)
		{
			pOUT = re.ptr<uchar>(i);
			for (int j = 0; j < re.cols; j++)
			{
				float depth = (float)depBuf[index];
				depth = (depth - low_range) / range;

				if (depth>255) depth = 0;
				if (depth < 0) depth = 0;
				pOUT[j] = depth;
				index++;
			}
		}
	}
	return re;
}
