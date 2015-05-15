// some property of the pxc-camera is defined as macro
#ifndef REALSENSEPXC_H
#define REALSENSEPXC_H

#define MM_PER_M 1000

#define LANDMARKS 78

#define FRAPS 30

#define C_ROWS 480//1080//540//720//480//720
#define C_COLS 640//1920//960//1280//640//1280

#define D_ROWS 480 
#define D_COLS 640 

//#define SELECT_FILE_LIMIT 3
//#define NOMINMAX
//#include <Windows.h>
#include <opencv2\core\core.hpp>
#include <opencv2\legacy\legacy.hpp>

#include <pxcprojection.h>
#include <pxcsession.h>
#include <pxcfacemodule.h>
#include <pxccapture.h>
#include <pxcfaceconfiguration.h>
#include <pxcbase.h>
#include <pxcsensemanager.h>

extern PXCSenseManager *sm;
extern PXC3DSeg *pSeg;
extern PXCFaceData *faceOutput;
extern PXCCaptureManager* capm;
extern PXCCapture::Device *device;
extern PXCProjection *projection;

extern PXCFaceData::LandmarkPoint point_ref[LANDMARKS];
extern PXCFaceData::PoseEulerAngles outAngle;
extern PXCFaceData::HeadPosition hpose;
extern pxcF64 hp_RotateMat[9];
extern PXCRectI32 faceRect;

extern int nLandmarks;
extern int globalr_d;
extern int globalc_d;
extern PXCImage::ImageInfo cinfo;
extern PXCImage::ImageInfo dinfo;
extern PXCImage::ImageInfo segInfo;

extern PXCImage* Depth;
extern PXCImage* Image;

/*
@ brief: init the real sense device
@ param[in] Source:
@			Source==NULL: start the Camera
@			Source!=NULL: process a rssdk file
@ param[in] read_or_write (when Source!=NULL)
@				ture: write
@				false:read
*/
int init_device(pxcCHAR* Source, bool read_or_write);
/*
@ brief: init the real sense device with 3d-segmentation and faceModule
@ param[in] Source:
@			Source==NULL: start the Camera
@			Source!=NULL: process a rssdk file
@ param[in] read_or_write (when Source!=NULL)
@				ture: write
@				false:read
*/
int init_segm_device(pxcCHAR* Source, bool statu);
/*
@ brief: get color and depth map from the stream
@ param[in] PXCImage-color:
@ param[in] PXCImage-depth:
@ param[out]out_color: color stream(BGR-24bit) capture by the camera
@ param[out]out_depth: depth stream(UINT16) capture by the camera
*/

bool getData(PXCImage*& color, PXCImage*& depth, cv::Mat& out_color, cv::Mat& out_depth);
bool getSegData(PXCImage*& seg_image, cv::Mat& out_seg);
bool getDepthData(PXCImage*& depth_image, cv::Mat& out_depth);
bool getColorData(PXCImage*& color_image, cv::Mat& out_color);

/**
@ brief: grab face features (eg. landmarks, pose-angles,...),return number of landmarks that had obtained
@ param[in] faceOutput: a face relevant module defined in the sdk
@ param[in] cinfo: color image information (height and width are used)(replaced by C_COLS and C_ROWS)
*/
int grabFaceMarks(PXCFaceData* faceOutput);

/*
@ brief£º fill the neighbors whitch are  no sampled or invalid
@ param[in&out] mask: 255 denote valid 0 means invalid or not sampled
@ param[in] Img:BGR_iput image
@ param[in&out] depth:depth mat get from the camera
@ param[in] fill_dense: define the times of fill circulation
*/
int fill_and_align(cv::Mat& mask, const cv::Size ImgSz, cv::Mat& depth, int fill_dense);

/*
@ brief£ºfill the 8-neighbor whith center point depth
@ param[in&out] mask:the valid point of the last time,output the new filled;
@ param[in&out] depth:the depth_mat of the last time, outout the new filled;
*/
void neighbor_fill(cv::Mat& mask, cv::Mat& depth);

/* 
@ brief:draw the landmarks
*/
void drawLandmarks(cv::Mat& canvas, PXCFaceData::LandmarkPoint* landmarks, int count = LANDMARKS);
/*
@ brief:draw the head pose on the mat
*/
void drawPose(cv::Mat &mat);
/*
@ brief: the main loop of real sense camera
*/
void rsFrameLoop();
/*
@ brief: test frame work embeding in rsFrameLoop())
*/
void test(cv::Mat& canvas);
/*
@ brief: test opencv snake algorithm in rsFrameLoop())
*/
void test_snake(cv::Mat& canvas);

void test_wink(cv::Mat& canvas);
/*
@ brief: real sense main runing Framework
*/
void rsMainFrame(pxcCHAR* Source, bool statu);
/*
@ brief: draw face detect rectangle
*/
void drawDetect(cv::Mat& canvas, int PositionCode);
#endif /*RealSensePxc.h*/
