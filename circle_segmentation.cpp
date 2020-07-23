//参数说明 .pcd int（控制胸径内点数量） int（控制剩余点数量） float(模型偏离阈值)
//DBH_P.pcd 50 100 0.016
//DBH_C.pcd 10 100 0.016
//DBH_NE.pcd 30 100 0.018
//DBH_clip.pcd 50 100 0.016
//stem.pcd 50 100 0.016
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>
#include <vector>
#include <ctime>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>), stem(new  pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyr(new  pcl::PointCloud<pcl::PointXYZ>);//圆心与半径
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t(new  pcl::PointCloud<pcl::PointXYZ>),
		cloud_f(new  pcl::PointCloud<pcl::PointXYZ>),
		cloud_DBH(new  pcl::PointCloud<pcl::PointXYZ>);//内外点 一个一个找

	pcl::PointXYZ XYR; //记录XYR
	//pcl::io::loadPCDFile("DBH_P.pcd", *cloud);
	pcl::io::loadPCDFile(argv[1], *stem);


	pcl::PassThrough<pcl::PointXYZ> passThrough;
	passThrough.setInputCloud(stem);
	passThrough.setFilterFieldName("z");
	passThrough.setFilterLimits(1.275, 1.325);
	passThrough.filter(*cloud);

	//pcl::io::loadPCDFile(argv[1], *cloud);
	std::cerr << "the number of points are :" << cloud->points.size() << std::endl;

	time_t start, end;
	start = clock();
	//创建Statistical滤波器
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	//sor.setMeanK(15);
	sor.setMeanK(10);
	sor.setStddevMulThresh(2); //判断离群点的标准差倍数 越小越严格
	sor.filter(*cloud);  //滤波后的点
	//输出剩余点
	//std::cerr << "the number of left are :" << cloud->points.size() << std::endl;
	//pcl::io::savePCDFile("statis_left_point.pcd",*cloud);


	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	//pcl::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::KdTree<pcl::PointXYZ>::Ptr());
	//创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::SACSegmentation<pcl::PointXYZ>::SearchPtr search(new pcl::search::KdTree<pcl::PointXYZ >());
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr	search(new pcl::search::KdTree<pcl::PointXYZ>);
	//可选设置
	seg.setOptimizeCoefficients(true);
	//必须设置
	seg.setModelType(pcl::SACMODEL_CIRCLE2D);
	//seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMethodType(pcl::SAC_PROSAC);
	seg.setMaxIterations(100);
	//偏离模型距离  这个参数很重要
	seg.setDistanceThreshold(strtof(argv[4], 0));
	// seg.setDistanceThreshold(0.015);
	//模型半径设置
	seg.setRadiusLimits(0.04, 0.12);
	//至少一个局内点的概率
	seg.setProbability(1);

	// 创建滤波器对象
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	int i = 0;
	while (true)
	{
		if (cloud->size() > 0)
		{
			// 从余下的点云中分割最大平面组成部分
			//seg.setSamplesMaxDist(0.3, search);
			seg.setSamplesMaxDist(0.5, search);
			search->setInputCloud(cloud);
			seg.setInputCloud(cloud);
			seg.segment(*inliers, *coefficients);
			//控制内点数量的参数 
			if (inliers->indices.size() <strtol(argv[2], 0, 10))
			{
				//没有搜索到点数据小于argv[2]且剩余点数量小于argv[3]剔除当前点继续while循环 否则跳出循环
				if (cloud->size()>strtol(argv[3], 0, 10))
				{
					/*
				  //去除以一个索引值 累
				  std::vector<int> temp;
				  for (int j = 1; j < cloud->size(); ++j)
				  {
				  temp.push_back(j);
				  }
				  //std::cout << "point sizes of cloud are :" << cloud->size() << std::endl;
				  pcl::copyPointCloud(*cloud,temp, *cloud);
				  //std::cout << "point sizes of cloud are :" << cloud->size() << std::endl;
				  std::vector<int>().swap(temp);
				  */
					cloud->erase(cloud->begin());
					continue;
				}
				else
				{
					std::cerr << "Could not estimate a circle model for the given dataset." << std::endl;
					std::cout << "the total number of circles are : " << i << std::endl;
					break;
				}

			}
			XYR.x = coefficients->values[0];
			XYR.y = coefficients->values[1];
			XYR.z = coefficients->values[2];
			xyr->push_back(XYR);

			// 分离内层
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_t);
			*cloud_DBH = *cloud_DBH + *cloud_t;

			std::cerr << "PointCloud representing the circle component: " << cloud_t->width * cloud_t->height << " data points." << std::endl;

			// 创建滤波器对象
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud.swap(cloud_f);//把外点转化给cloud
			i++;
		}
		else
		{
			break;
		}
	}
	end = clock();
	std::cout << "程序用时: " << (end - start) / CLOCKS_PER_SEC << " s" << std::endl;
	if (cloud->points.size() > 0)
		std::cout << "the number left are : " << cloud->size() << std::endl;
	//   pcl::io::savePCDFile("outlier_Point.pcd",*cloud);  //输出局外点
	pcl::io::savePCDFile("DBH_circle_XYR.pcd", *xyr);
	pcl::io::savePCDFile("cloudDBH_point.pcd", *cloud_DBH);
	return (0);
}
