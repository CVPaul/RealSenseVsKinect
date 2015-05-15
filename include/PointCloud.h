#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <string>
#include "pxcbase.h"// this include the defination of struct PXCPoint3DF32 which is also 
//defined in RealSense's sdk so if Realsense is used please remove the pxcbase.h in this file else please in clude is for Kinect's sdk 

class PointCloud
{
public:
	PointCloud() :size(0), pc(NULL)
	{
		center.x = 0.0;
		center.y = 0.0;
		center.z = 0.0;
	};
	~PointCloud();
	void load(std::string file);
	void save(std::string file);
	void create(int sz);
	void move(PXCPoint3DF32 trans);
public:
	PXCPoint3DF32* pc;
	PXCPoint3DF32 center;
	int size;
};

extern PointCloud world;

#endif /*PointCloud.h*/
