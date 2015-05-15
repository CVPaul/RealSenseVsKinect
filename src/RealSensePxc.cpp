#include "RealSensePxc.h"
#include "gl_animation.h"
#include "gl_operation.h"

PXCSenseManager *sm;
PXC3DSeg *pSeg;
PXCFaceData *faceOutput;
PXCCaptureManager* capm;
PXCCapture::Device *device;
PXCProjection *projection;

PXCFaceData::LandmarkPoint point_ref[LANDMARKS];
PXCFaceData::PoseEulerAngles outAngle;
PXCFaceData::HeadPosition hpose;
pxcF64 hp_RotateMat[9];
PXCRectI32 faceRect;

int globalr_d;
int globalc_d;
PXCImage::ImageInfo cinfo;
PXCImage::ImageInfo dinfo;
PXCImage::ImageInfo segInfo;
PXCImage* Depth;
PXCImage* Image;

int nLandmarks = 0;

int init_device(pxcCHAR* Source, bool read_or_write)
{
	pxcStatus status;
	sm = PXCSenseManager::CreateInstance();
	if (!sm)
	{
		wprintf_s(L"Error! can not create the SenseManager!\n");
		return -2;
	}
	sm->EnableFace();
	PXCFaceModule* faceModule = sm->QueryFace();
	PXCFaceConfiguration* config = faceModule->CreateActiveConfiguration();
	faceOutput = faceModule->CreateOutput();

	config->detection.isEnabled = true;// true;
	config->landmarks.isEnabled = true;// true;
	config->pose.isEnabled = true;
	config->ApplyChanges();
	capm = sm->QueryCaptureManager();
	if (Source != NULL)
	{
		capm->SetFileName(Source, read_or_write);
	}
	sm->EnableStream(PXCCapture::StreamType::STREAM_TYPE_COLOR, C_COLS, C_ROWS, FRAPS);
	sm->EnableStream(PXCCapture::StreamType::STREAM_TYPE_DEPTH, D_COLS, D_ROWS, FRAPS);
	status = sm->Init();
	if (status < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error!SenseManager initial failed!\n");
		return -3;
	}
	device = capm->QueryDevice();
	projection = device->CreateProjection();
	/*
	PXCPointF32 fov = device->QueryColorFieldOfView();
	device->SetMirrorMode(PXCCapture::Device::MirrorMode::MIRROR_MODE_HORIZONTAL);
	*/
	if (!projection)
	{
		wprintf_s(L"Error!Create Projection failed!\n");
		return -4;
	}
	return 0;
}
int init_segm_device(pxcCHAR* Source, bool statu)
{
	pxcStatus status;
	//-----------------------------------------------------------------------------------------------
	sm = PXCSenseManager::CreateInstance();
	if (!sm)
	{
		wprintf_s(L"Error! can not create the SenseManager!\n");
		return -2;
	}
	//-----------------------------------------------------------------------------------------------
	pxcStatus result = sm->Enable3DSeg();
	if (result < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error: Enable3DSeg failed (%d)\n", result);
		return -3;
	}
	pSeg = sm->Query3DSeg();
	if (!pSeg) return -4;
	//-----------------------------------------------------------------------------------------------
	result = sm->EnableFace();
	if (result < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error: EnableFace failed (%d)\n", result);
		return -5;
	}
	PXCFaceModule* faceModule = sm->QueryFace();
	if (!faceModule) return -6;
	//------------------------------------------------------------------------------------------------
	PXCFaceConfiguration* config = faceModule->CreateActiveConfiguration();
	faceOutput = faceModule->CreateOutput();

	config->detection.isEnabled = true;// true;
	config->landmarks.isEnabled = true;// true;
	config->pose.isEnabled = true;

	config->ApplyChanges();
	capm = sm->QueryCaptureManager();
	if (Source != NULL)
	{
		capm->SetFileName(Source, statu);
	}
	sm->EnableStream(PXCCapture::StreamType::STREAM_TYPE_COLOR, C_COLS, C_ROWS, FRAPS);
	sm->EnableStream(PXCCapture::StreamType::STREAM_TYPE_DEPTH, D_COLS, D_ROWS, FRAPS);
	status = sm->Init();
	if (status < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error!SenseManager initial failed!\n");
		return -7;
	}
	device = capm->QueryDevice();
	projection = device->CreateProjection();
	
	if (!projection)
	{
		wprintf_s(L"Error!Create Projection failed!\n");
		return -8;
	}
	return 0;
}
int grabFaceMarks(PXCFaceData* faceOutput)
{
	int ndata = 0;
	if (faceOutput == NULL)
		return 0;
	for (int i = 0; i < faceOutput->QueryNumberOfDetectedFaces(); i++)
	{
		PXCFaceData::Face* trackedFace = faceOutput->QueryFaceByIndex(i);
		if (trackedFace->QueryDetection() == NULL)
			continue;
		const PXCFaceData::LandmarksData* landmarkdata = trackedFace->QueryLandmarks();
		const PXCFaceData::DetectionData* detectface = trackedFace->QueryDetection();
		const PXCFaceData::PoseData* pose = trackedFace->QueryPose();

		if (landmarkdata)
		{
			landmarkdata->QueryPoints(point_ref);
			ndata = landmarkdata->QueryNumPoints();
		}
		if (pose)
		{
			pose->QueryPoseAngles(&outAngle);
			pose->QueryHeadPosition(&hpose);
			pose->QueryRotationMatrix(hp_RotateMat);
		}
		if (detectface)detectface->QueryBoundingRect(&faceRect);
		
		break;
	}
	return ndata;
}
bool getSegData(PXCImage*& seg_image, cv::Mat& out_seg)
{
	segInfo = seg_image->QueryInfo();
	PXCImage::ImageData seg_data;
	pxcStatus seg_stat = seg_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &seg_data);
	if (seg_stat < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error! in getSegData() cannot get the data!\n");
		return false;
	}
	uchar* pixel = (uchar*)seg_data.planes[0];
	cv::Mat(segInfo.height, segInfo.width, CV_8UC4, pixel).copyTo(out_seg);
	//put the draw ellipse at the beginning(before Array2Mat)---------------------->>>>>
	if (&seg_data) seg_image->ReleaseAccess(&seg_data);
	return true;
}
bool getDepthData(PXCImage*& depth_image, cv::Mat& out_depth)
{
	dinfo = depth_image->QueryInfo();
	PXCImage::ImageData depth_data;
	pxcStatus depth_stat = depth_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depth_data);
	if (depth_stat < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error! in getSegData() cannot get the data!\n");
		return false;
	}
	unsigned short* pixel = (unsigned short*)depth_data.planes[0];
	cv::Mat(dinfo.height, dinfo.width, CV_16UC1, pixel).copyTo(out_depth);
	//put the draw ellipse at the beginning(before Array2Mat)---------------------->>>>>
	if (&depth_data) depth_image->ReleaseAccess(&depth_data);
	return true;
}
bool getColorData(PXCImage*& color_image, cv::Mat& out_color)
{
	cinfo = color_image->QueryInfo();
	PXCImage::ImageData color_data;
	pxcStatus color_stat = color_image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &color_data);
	if (color_stat < PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Error! in getSegData() cannot get the data!\n");
		return false;
	}
	uchar* pixel = (uchar*)color_data.planes[0];
	cv::Mat(cinfo.height, cinfo.width, CV_8UC3, pixel).copyTo(out_color);
	//put the draw ellipse at the beginning(before Array2Mat)---------------------->>>>>
	if (&color_data) color_image->ReleaseAccess(&color_data);
	return true;
}
bool getData(PXCImage*& color, PXCImage*& depth, cv::Mat& out_color, cv::Mat& out_depth)
{
	bool get_color = getColorData(color, out_color);
	bool get_depth = getDepthData(depth, out_depth);
	return get_color&&get_depth;
}
void rsFrameLoop()
{
	cv::Mat out_color, out_depth;
	cv::Mat mask;
	while (1)
	{
		if (sm->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;
		//wprintf_s(L"frames:%d,%f\n", frames++);
		faceOutput->Update();

		PXCCapture::Sample *sample = sm->QuerySample();
		Image = sample->color;
		Depth = sample->depth;

		bool data_suc = getData(Image, Depth, out_color, out_depth);
		nLandmarks = grabFaceMarks(faceOutput);

		if (data_suc&&nLandmarks >= LANDMARKS)
		{
			if (C_ROWS<500)
				fill_and_align(mask, out_color.size(), out_depth, 0);
			if (C_ROWS >= 500)
				fill_and_align(mask, out_color.size(), out_depth, 0);
			// TODO: add some code to deal each frame(out_color/out_depth) here!
			test_wink(out_color);
			cv::imshow("reFrameLoop_test", out_color);
			cv::waitKey(1);
		}
		sm->ReleaseFrame();
	}
}
int fill_and_align(cv::Mat& mask, const cv::Size ImgSz, cv::Mat& depth, int fill_dense)
{
	uchar *pMask;
	uint16_t *pDep_org;
	unsigned short* pDep;

	PXCPoint3DF32 data_3d;
	PXCPointF32 data_align;

	int cx, cy;
	mask = cv::Mat::zeros(ImgSz, CV_8UC1);
	cv::Mat depth_map = cv::Mat::zeros(ImgSz, CV_16UC1);

	for (int i = 0; i < depth.rows; i++)
	{
		pMask = mask.ptr<uchar>(i);
		pDep_org = depth.ptr<uint16_t>(i);
		for (int j = 0; j < depth.cols; j++)
		{
			if ((pDep_org[j])>100 && (pDep_org[j]) < 1200)
			{
				data_3d.x = j; // projection part
				data_3d.y = i;
				data_3d.z = pDep_org[j];
				projection->MapDepthToColor(1, &data_3d, &data_align);

				cx = (int)(data_align.x + 0.5); // alignment
				cy = (int)(data_align.y + 0.5);
				if (cx<0 || cx >= ImgSz.width || cy<0 || cy >= ImgSz.height)
					continue;
				pMask = mask.ptr<uchar>(cy)+cx;
				pDep = depth_map.ptr<unsigned short>(cy)+cx;

				*pMask = 255;
				*pDep = pDep_org[j];
			}
		}
	}
	int i = 0;
	for (i = 0; i < fill_dense; i++)
		neighbor_fill(mask, depth_map);
	depth_map.copyTo(depth);
	return i;
}
void neighbor_fill(cv::Mat& mask, cv::Mat& depth)
{
	uchar *pMsk, *pMsk_up, *pMsk_down;
	uchar *pMsk_p, *pMsk_up_p, *pMsk_down_p;
	unsigned short *pDep, *pDep_up, *pDep_down;

	int height = mask.rows;
	int width = mask.cols;

	cv::Mat pre_mask = mask.clone();
	for (int i = 1; i < height - 1; i++) // fill
	{
		pMsk_p = pre_mask.ptr<uchar>(i);
		pMsk_up_p = pre_mask.ptr<uchar>(i - 1);
		pMsk_down_p = pre_mask.ptr<uchar>(i + 1);

		pMsk = mask.ptr<uchar>(i);
		pMsk_up = mask.ptr<uchar>(i - 1);
		pMsk_down = mask.ptr<uchar>(i + 1);

		pDep = depth.ptr<unsigned short >(i);
		pDep_up = depth.ptr<unsigned short >(i - 1);
		pDep_down = depth.ptr<unsigned short >(i + 1);

		for (int j = 1; j < width - 1; j++)
		{
			if (pMsk_p[j])
			{
				if (!pMsk_up_p[j - 1])
				{
					pDep_up[j - 1] = pDep[j];
					pMsk_up[j - 1] = 255;
				}
				if (!pMsk_up_p[j])
				{
					pDep_up[j] = pDep[j];
					pMsk_up[j] = 255;
					//wprintf_s(L"%d\n", pDep[j]);
				}
				if (!pMsk_up_p[j + 1])
				{
					pDep_up[j + 1] = pDep[j];
					pMsk_up[j + 1] = 255;
				}
				if (!pMsk_p[j - 1])
				{
					pDep[j - 1] = pDep[j];
					pMsk[j - 1] = 255;
				}
				if (!pMsk_p[j + 1])
				{
					pDep[j + 1] = pDep[j];
					pMsk[j + 1] = 255;
				}
				if (!pMsk_down_p[j - 1])
				{
					pDep_down[j - 1] = pDep[j];
					pMsk_down[j - 1] = 255;
				}
				if (!pMsk_down_p[j])
				{
					pDep_down[j] = pDep[j];
					pMsk_down[j] = 255;
				}
				if (!pMsk_down_p[j + 1])
				{
					pDep_down[j + 1] = pDep[j];
					pMsk_down[j + 1] = 255;
				}
			}
		}
	}
}
void drawLandmarks(cv::Mat& canvas, PXCFaceData::LandmarkPoint* landmarks, int count/* = LANDMARKS*/)
{
	// first check it:
	if (nLandmarks < LANDMARKS) return;

	for (int k = 0; k < count; k++)
	{
		if (landmarks[k].confidenceImage)
		{
			cv::circle(canvas, cv::Point2f(landmarks[k].image.x, landmarks[k].image.y),
				2, cv::Scalar(0, 0, 255), 1);// red circles
			
		}
		else
		{
			cv::circle(canvas, cv::Point2f(landmarks[k].image.x, landmarks[k].image.y),
				2, cv::Scalar::all(255), 1);// white circles
		}
	}
}
void drawPose(cv::Mat &mat)
{
	if (mat.empty())
	{
		wprintf(L"Error in drawPose!\n the data is empty!notiong done and returned!\n");
		return;
	}
	char m_yaw[64];
	char m_pitch[64];
	char m_roll[64];

	sprintf_s(m_yaw, "Yaw:%.0f", outAngle.yaw);
	sprintf_s(m_pitch, "Pitch:%.0f", outAngle.pitch);
	sprintf_s(m_roll, "Roll:%.0f", outAngle.roll);

	const int THeight = 10;
	int xstart = 5 * THeight;
	int span_x = 100;

	cv::putText(mat, m_yaw, cv::Point(mat.cols - span_x, xstart + -1 * THeight), 1, 1, cv::Scalar(255, 0, 0), 1);
	cv::putText(mat, m_pitch, cv::Point(mat.cols - span_x, xstart + 1 * THeight), 1, 1, cv::Scalar(255, 0, 0), 1);
	cv::putText(mat, m_roll, cv::Point(mat.cols - span_x, xstart + 3 * THeight), 1, 1, cv::Scalar(255, 0, 0), 1);
}
void drawDetect(cv::Mat& canvas,int PositionCode)
{
	if (faceRect.x<0 || faceRect.y<0 || faceRect.w==0||faceRect.h==0||
		(faceRect.x + faceRect.w)>=C_COLS || (faceRect.y + faceRect.h)>=C_ROWS)
		return; // this detect is out of range or detect losed
	cv::Rect frect(faceRect.x, faceRect.y, faceRect.w, faceRect.h);
	cv::Mat face = canvas(frect);
	//frect.width = 170;
	//frect.height = float(faceRect.h) / float(faceRect.w)*frect.width;
	if (PositionCode == 0)// right-top
	{
		int span = C_COLS - frect.width;
		frect.x = span-1;
		frect.y = 0;
		cv::resize(face, canvas(frect), frect.size());
	}
	else if (PositionCode == 1)// left_top
	{
		frect.x = 0;
		frect.y = 0;
		cv::resize(face, canvas(frect), frect.size());
	}
	else
		printf("Error! not implement this mode=%d\n", PositionCode);
}
void test(cv::Mat& canvas) // test the convertion between wolrd and head coords
{
	PXCPoint3DF32 worlds[LANDMARKS], heads[LANDMARKS];
	PXCPointF32 image_pt[LANDMARKS], draw_pts[LANDMARKS];
	
	convertLansmarks(worlds, image_pt);
	world2head(LANDMARKS, worlds, heads);
	printf("test 29 index,value=%f,%f,%f\n", hpose.headCenter.x, hpose.headCenter.y, hpose.headCenter.z);
	//head2world(LANDMARKS, heads, heads);
	Transform trans;
	float TransV[3] = {
		/*hpose.headCenter.x,
		hpose.headCenter.y,
		hpose.headCenter.z*/
		0,0,500
	};
	//head2world(LANDMARKS, heads, heads);

	float RotateV[3] = { 0, 0, 0 };
	float MoveV[3] = { 0, 0, 0 };
	trans.TranformInOnce(LANDMARKS, TransV, RotateV, MoveV, heads, heads);

	for (int k = 0; k < LANDMARKS; k++) // check it!!!!
	{
		heads[k].x =- heads[k].x; // reflect by x-axis
	}

	projection->ProjectCameraToColor(LANDMARKS, heads, draw_pts);
	draw2image(LANDMARKS, draw_pts, canvas,cv::Scalar(255,0,0));
}
void rsMainFrame(pxcCHAR* Source, bool statu)
{
	int stat = init_device(Source, statu);// init RealSense Camera!
	if (stat < 0)
	{
		printf("init camera failed(in RS2D_FAP_main)!\n");
		return;
	}
	rsFrameLoop();
}
void test_snake(cv::Mat& canvas)
{
	float alpha = 1.0;
	float beta = 0.5;
	float gamma = 1.0;

	CvPoint* pts = new CvPoint[8];
	for (int k = 0; k < 8; k++) // left
	{
		pts[k].x = point_ref[10 + k].image.x;
		pts[k].y = point_ref[10 + k].image.y;
		cv::circle(canvas, pts[k], 1, cv::Scalar(255, 0, 0));
	}
	cv::Mat temp;
	cv::cvtColor(canvas, temp, CV_BGR2GRAY);
	IplImage imag = temp;
	CvTermCriteria criteria;

	criteria.epsilon = 0.1;
	criteria.max_iter = 50;
	criteria.type = CV_TERMCRIT_ITER;

	std::vector<cv::Point> left;
	std::vector<cv::Point> right;

	cvSnakeImage(&imag, pts, 8, &alpha, &beta, &gamma, CV_VALUE,cvSize(3,3),criteria);

	for (int k = 0; k < 8; k++) // right
	{
		cv::circle(canvas, pts[k], 1, cv::Scalar(0, 0, 255)); // after snake

		pts[k].x = point_ref[18 + k].image.x;
		pts[k].y = point_ref[18 + k].image.y;
		cv::circle(canvas, pts[k], 1, cv::Scalar(255, 0, 0));
	}

	cvSnakeImage(&imag, pts, 8, &alpha, &beta, &gamma, CV_VALUE, cvSize(3, 3), criteria);

	for (int k = 0; k < 8; k++) // right
	{
		cv::circle(canvas, pts[k], 1, cv::Scalar(0, 0, 255)); // after snake
	}
	delete pts;
}
void test_wink(cv::Mat& canvas)
{
	std::vector<cv::Point> lpts(8);
	std::vector<cv::Point> rpts(8);

	cv::Point2f lcenter(0, 0), rcenter(0, 0);

	for (int k = 0; k < 8; k++) 
	{
		lpts[k].x = point_ref[10 + k].image.x;// left
		lpts[k].y = point_ref[10 + k].image.y;

		rpts[k].x = point_ref[18 + k].image.x;// right
		rpts[k].y = point_ref[18 + k].image.y;

		lcenter.x += point_ref[10 + k].image.x;
		lcenter.y += point_ref[10 + k].image.y;

		rcenter.x += point_ref[18 + k].image.x;
		rcenter.y += point_ref[18 + k].image.y;
	}
	lcenter.x /= 8;
	lcenter.y /= 8;

	rcenter.x /= 8;
	rcenter.y /= 8;

	std::vector< std::vector<cv::Point> > fill_poly;

	fill_poly.push_back(lpts);
	fill_poly.push_back(rpts);

	cv::Mat mask = cv::Mat::zeros(canvas.size(),CV_8UC1);

	cv::polylines(mask, fill_poly[0], true, cv::Scalar::all(255), 1);
	cv::polylines(mask, fill_poly[1], true, cv::Scalar::all(255), 1);

	int lcount = cv::floodFill(mask, lcenter, cv::Scalar::all(255), &cv::Rect(), cv::Scalar::all(2), cv::Scalar::all(2));
	int rcount = cv::floodFill(mask, rcenter, cv::Scalar::all(255), &cv::Rect(), cv::Scalar::all(2), cv::Scalar::all(2));
	printf("lcount=%d,rcount=%d\n", lcount, rcount);

	cv::imshow("mask", mask);
	cv::waitKey(1);
	//cv::polylines(canvas, fill_poly[1], true, cv::Scalar::all(255), 1);

	/*cv::fillConvexPoly(canvas, fill_poly, cv::Scalar::all(255));
	cv::fillConvexPoly(canvas, fill_poly[1], cv::Scalar::all(255));*/
}