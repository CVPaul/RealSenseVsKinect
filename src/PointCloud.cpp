#include "PointCloud.h"

PointCloud world;

PointCloud::~PointCloud()
{
	if (pc) delete[] pc;
	size = 0;
}
void PointCloud::load(std::string file)
{
	FILE* fp;
	fp = fopen(file.c_str(), "r");
	if (!fp)
	{
		printf("Error! in loadPointCloud():file open failed!\n");
		return;
	}
	int nsz;
	fscanf(fp, "%d\n", &nsz);
	if (nsz != size)
	{
		delete[] pc;
		size = nsz;
		pc = new PXCPoint3DF32[size];
	}
	if (!pc)
	{
		printf("Error! in load(): out if memory!\n");
		return;
	}
	int count = 0, valid_sz = 0;
	center.x = center.y = center.z = 0.0;
	while (fscanf(fp, "%f %f %f\n", &pc[count].x,
		&pc[count].y, &pc[count].z) != EOF)
	{
		if (pc[count].z > 100 && pc[count].z < 2000)
		{
			valid_sz++;
			center.x += pc[count].x;
			center.y += pc[count].y;
			center.z += pc[count].z;
		}
		count++;
	}
	if (valid_sz>0)
	{
		center.x /= valid_sz;
		center.y /= valid_sz;
		center.z /= valid_sz;
	}
	if (size != count)
	{
		printf("Error! in loadPointCloud():sizes are not equal:size=%d,count=%d!\n", size, count);
		delete[] pc; pc = NULL;
		size = 0; count = 0;
		fclose(fp);
		return;
	}
	fclose(fp);
}
void PointCloud::save(std::string file)
{
	FILE *fp;
	fp = fopen(file.c_str(), "w");
	if (!fp)
	{
		printf("Error! in savePointCloud():file open failed!\n");
		return;
	}
	fprintf(fp, "%d\n", size);
	for (int k = 0; k < size; k++)
	{
		fprintf(fp, "%f %f %f\n", pc[k].x, pc[k].y, pc[k].z);
	}
	fclose(fp);
}
void PointCloud::create(int sz)
{
	if (size != sz)
	{
		delete[] pc;
		size = sz;
		pc = new PXCPoint3DF32[size];
	}
	if (!pc)
	{
		size = 0;
		pc = NULL;
	}
}
void PointCloud::move(PXCPoint3DF32 trans)
{
	for (int k = 0; k < size; k++)
	{
		pc[k].x -= trans.x;
		pc[k].y -= trans.y;
		pc[k].z -= trans.z;
	}
}