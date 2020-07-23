//实现Z最低点查找地面点 
// (pcd、las) 文件名 float 栅格化尺度 保存的文件名
// 适当结合坡度
#include <iostream>
#include <vector>
#include <string>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <liblas/liblas.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct point_grid
{
	int index_x, index_y;

	std::vector<int> grid_point_index;

	bool is_ocuupied;
};

//cloudIn要处理的点数据 grid_resolution：栅格化大小 point_grid_：记录栅格点
void computeGrid(PointCloudT::Ptr cloudIn, float grid_resolution, std::vector<std::vector<point_grid>> &point_grid_);

//groundPoint 提取的地面点
void computeGround(PointCloudT::Ptr cloudIn, PointCloudT::Ptr groundPoint, std::vector<std::vector<point_grid>> point_grid_);

void filterSor(PointCloudT::Ptr cloudIn, PointCloudT::Ptr cloudOut, int neark, float devThresh);

void readData(std::string file_name, PointCloudT::Ptr cloud_in);
int main(int argc, char **argv)
{
	PointCloudT::Ptr cloud_in(new PointCloudT), groundCloud(new PointCloudT);

	readData(argv[1], cloud_in);

	std::cout << "数据量为：" << cloud_in->size() << std::endl;

	//pcl::io::loadPCDFile(argv[1], *cloud_in);

	float grid_resolution = std::strtof(argv[2], 0);
	std::cout << "grid_resolution:" << grid_resolution << std::endl;
	//存储栅格信息
	std::vector<std::vector<point_grid>> point_grid_;

	clock_t clock_start, clock_end;
	clock_start = clock();

	//栅格计算
	computeGrid(cloud_in, grid_resolution, point_grid_);
	std::cout << "point_grid_.size():" << point_grid_.size() << " point_grid_[0].size():" << point_grid_[0].size() << std::endl;
	//计算最低点
	computeGround(cloud_in, groundCloud, point_grid_);

	if (groundCloud->size() > 0)
	{
		//体素滤波
		//filterSor(groundCloud, groundCloud, 50, 0.8);
		std::cout << "************************* 地面点的数量为：" << groundCloud->size() << " *************************" << std::endl;
		//pcl::io::savePCDFileBinary(argv[3] + std::to_string(static_cast<int>(grid_resolution * 100)) + ".pcd", *groundCloud);
		pcl::io::savePCDFileBinary(argv[3], *groundCloud);
	}
	else
	{
		std::cout << "************************* 地面点为空 *************************" << std::endl;
	}

	clock_end = clock();
	std::cout << "地面估计用时：" << (clock_end - clock_start) / CLOCKS_PER_SEC << " s" << std::endl;
	
	std::cout << "******************************************* 地面提取结束 *******************************************" << std::endl;
	//system("pause");
	return 0;
}

void computeGrid(PointCloudT::Ptr cloudIn, float grid_resolution, std::vector<std::vector<point_grid>> &point_grid_)
{
	PointT minXYZ, maxXYZ;
	pcl::getMinMax3D(*cloudIn, minXYZ, maxXYZ);

	//XY维度尺寸的大小
	int grid_x_size, grid_y_size;
	grid_x_size = static_cast<int>(floor((maxXYZ.x - minXYZ.x) / grid_resolution));
	grid_y_size = static_cast<int>(floor((maxXYZ.y - minXYZ.y) / grid_resolution));

	std::cout << "grid_x_size:" << grid_x_size << " grid_y_size:" << grid_y_size << std::endl;
	//初始化栅格空间
	point_grid_.resize(grid_x_size + 1);
	for (size_t i = 0; i < point_grid_.size(); i++)
	{
		point_grid_[i].resize(grid_y_size + 1);
	}

	//将点分割到栅格中
	for (size_t i = 0; i < cloudIn->size(); i++)
	{
		int index_x = static_cast<int>(floor((cloudIn->points[i].x - minXYZ.x) / grid_resolution));
		int index_y = static_cast<int>(floor((cloudIn->points[i].y - minXYZ.y) / grid_resolution));

		point_grid_[index_x][index_y].is_ocuupied = true;
		point_grid_[index_x][index_y].grid_point_index.push_back(i);
	}
}

void computeGround(PointCloudT::Ptr cloudIn, PointCloudT::Ptr groundPoint, std::vector<std::vector<point_grid>> point_grid_)
{
	PointCloudT::Ptr single_grid_point(new PointCloudT);

	for (size_t index_x = 0; index_x < point_grid_.size(); index_x++)
	{
		for (size_t index_y = 0; index_y < point_grid_[0].size(); index_y++)
		{
			//该栅格有点
			if (point_grid_[index_x][index_y].is_ocuupied)
			{
				//点数大于一定数量
				if (point_grid_[index_x][index_y].grid_point_index.size()>2)
				{
					pcl::copyPointCloud(*cloudIn, point_grid_[index_x][index_y].grid_point_index, *single_grid_point);

					float min_z = single_grid_point->points[0].z;
					int min_z_index = 0;
					//查找最小索引
					for (size_t i = 1; i < single_grid_point->size(); i++)
					{
						if (single_grid_point->points[i].z < min_z)
						{
							min_z = single_grid_point->points[i].z;
							min_z_index = i;
						}
					}
					//查找结束 并确定了min_z_index
					groundPoint->push_back(single_grid_point->points[min_z_index]);
					single_grid_point->clear();
				}
			}
		}
	}
}

//统计滤波
void filterSor(PointCloudT::Ptr cloudIn, PointCloudT::Ptr cloudOut, int neark, float devThresh)
{
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloudIn);
	sor.setMeanK(neark);
	sor.setStddevMulThresh(devThresh);
	sor.filter(*cloudOut);
}

void readData(std::string file_name, PointCloudT::Ptr cloud_in)
{
	//判断数据类型
	//为pcd数据
	if (file_name.compare(file_name.size() - 3, 3, "pcd") == 0)
	{
		pcl::io::loadPCDFile(file_name, *cloud_in);
	}
	//为las数据
	if (file_name.compare(file_name.size() - 3, 3, "las") == 0)
	{
		std::ifstream iFile;
		iFile.open(file_name, std::ios::in | std::ios::binary);
		if (!iFile)
		{
			std::cout << "找不到指定文件:" << file_name << std::endl;
		}
		liblas::ReaderFactory f_read;
		//std::cout << "***********************************" << std::endl;
		liblas::Reader las_reader = f_read.CreateWithStream(iFile);
		//std::cout << "***********************************" << std::endl;

		PointT single_point_xyz;
		while (las_reader.ReadNextPoint())
		{
			
			const liblas::Point & las_point = las_reader.GetPoint();
			//std::cout << las_point.GetX() << " " << las_point.GetY() << " " << las_point.GetZ() << std::endl;
			single_point_xyz.x = las_point.GetX();
			single_point_xyz.y = las_point.GetY();
			single_point_xyz.z = las_point.GetZ();
			cloud_in->push_back(single_point_xyz);
		}
		//std::cout << "数据读取结束" << std::endl;
		iFile.close();
	}
}