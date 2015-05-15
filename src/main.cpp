#include "KinectForWindows.h"

// main function 

int main()
{
	capKinect kinect;
	kinect.initKinect();
	std::string path="KinectPointCloud.txt";
	std:;string dep_path="KinectDepth_sxp.avi";
	cv::Mat depth,depth_show;
	/*cv::VideoWriter wrt(dep_path,CV_FOURCC('M','J','P','G'),30,cv::Size(512,424),false);
	if(!wrt.isOpened())
	{
		printf("ERROR! can not open the file to write (cv::VideoWriter())\n");
		return -1;
	}*/
	int code=0;
	int start=0;
	while (1)
	{
		cv::Mat depth = kinect.update(depth_show); // this is save in UINT16 format
		if(!depth_show.empty())
		{
			cv::imshow("depth_show",depth_show);

			//if(start)wrt<<depth_show;

			code=cv::waitKey(20);
			if (code=='k')// press 'k' to save data;
			{
				//kinect.getPointCloud(depth, world);
				//world.save(path);
				//cv::imwrite("WholeView.jpg",depth_show);
				int save_index=0;
				save_depth(save_index,"KinectOBJ80cm",depth,depth_show);
				printf("PointCloud of Kinect saved in %s\n",path.c_str());
				break;
				if(start==0)
				{
					start=1;
					printf("start record!\n");
				}
				else
					break;
			}
			getFraps();
		}
	}
	//kinect.saveDepthAsPointCloud(1,"KinectOBJ_FACE");
	/*for(int k=0;k<7;k++)
	{
		kinect.saveDepthAsPointCloud(k,"Kinect");
	}
	for(int k=0;k<7;k++)
	{
		kinect.saveDepthAsPointCloud(k,"KinectOBJ");
	}*/
	//printf("Depth Image have been written to %s\n",dep_path.c_str());
	//wrt.release();
	system("PAUSE");
	return 0;
}