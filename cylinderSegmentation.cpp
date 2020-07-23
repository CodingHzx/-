//.pcd
//tree.pcd
//elevation_normalized.pcd
#include <iostream>
#include <ctime> //计时用的头文件
#include <pcl/octree/octree.h>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>

int
main (int argc, char** argv)
{				
	time_t start, end, time; /*注意计时所用的变量名称*/
	/*程序开始执行，开始计时*/
	start = clock();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>), cloud_cotree(new  pcl::PointCloud<pcl::PointXYZ>), cloud_cotree_true(new  pcl::PointCloud<pcl::PointXYZ>), cloud_f(new  pcl::PointCloud<pcl::PointXYZ>), cloud_out(new  pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr D_XYZR (new  pcl::PointCloud<pcl::PointNormal>);//圆柱方向 圆心与半径
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>), normal_in(new pcl::PointCloud<pcl::Normal>), normal_out(new pcl::PointCloud<pcl::Normal>);

	pcl::PointNormal direction_xyzr;
  //pcl::io::loadPCDFile("L5_3_02.pcd", *cloud);
	pcl::io::loadPCDFile(argv[1], *cloud);
  std::cerr << "the number of data is :" << cloud->points.size() << std::endl;
  std::cout << "数据加载完成" << std::endl;
 
  //创建八叉树 resolution八叉树大小
  typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;
  AlignedPointTVector voxel_center_list_arg;
  
  //octree体素大小
  float resolution = 0.8f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(cloud);    //将点云赋给octree
  octree.addPointsFromInputCloud();  //建立octree
  //提取octree中心
  voxel_center_list_arg.clear();
  octree.getOccupiedVoxelCenters(voxel_center_list_arg);
  pcl::PointXYZ center_point;
  
  std::cout << "the number of octree is :" << octree.getLeafCount() << std::endl;
  std::cout << "the number of octree is :" << voxel_center_list_arg.size() << std::endl;
  //创建提取对象
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  std::vector<int> pointIdxVec;
  
  int n = 0;
  
  for (int i = 0; i < voxel_center_list_arg.size(); i++)
  {
	  //确定octree中心点坐标
	  center_point.x = voxel_center_list_arg[i].x;
	  center_point.y = voxel_center_list_arg[i].y;
	  center_point.z = voxel_center_list_arg[i].z;

	  if (octree.voxelSearch(center_point, pointIdxVec))
	  {
		  //连续性
		  if (pointIdxVec.size()>40)
		  {
			  //std::cout << "duandian1"<< std::endl;
			  //对reset()函数不了解
			  pcl::copyPointCloud(*cloud, pointIdxVec, *cloud_cotree);
			  //清除内存
			  std::vector<int>().swap(pointIdxVec);
			  //创建法线估计对象
			  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
			  normal_est.setNumberOfThreads(8);
			  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
			  normal_est.setInputCloud(cloud_cotree);
			  normal_est.setSearchMethod(tree);
			  normal_est.setRadiusSearch(0.1);
			  normal_est.compute(*cloud_normal);
			 // std::vector<int>().swap(pointIdxVec);
			  // std::cout << "duandian1" << std::endl;
			   //std::cout << "估计法线 完成" << std::endl;
			  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			  //创建分割对象
			  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
			   pcl::SACSegmentation<pcl::PointXYZ>::SearchPtr search(new pcl::search::KdTree<pcl::PointXYZ >());
			  //可选设置
			  seg.setOptimizeCoefficients(true);
			  //必须设置
			  seg.setModelType(pcl::SACMODEL_CYLINDER);
			//  seg.setMethodType(pcl::SAC_RANSAC);
			  seg.setMethodType(pcl::SAC_PROSAC);
			  //偏离模型距离 这个参数很重要
			  seg.setDistanceThreshold(0.015);
			  //模型半径设置
			  ////seg.setRadiusLimits(0.03, 0.2); 
			  seg.setRadiusLimits(0.05, 0.4);

			  //局部搜索半径
			  seg.setSamplesMaxDist(0.35, search);  ////设置随机采样时样本之间允许的最大距离为radius ， search 为采样时使用的搜索对象
			  search->setInputCloud(cloud_cotree);
			  seg.setInputNormals(cloud_normal);
			  seg.setInputCloud(cloud_cotree);
			  seg.setMaxIterations(10);
			  seg.segment(*inliers, *coefficients);
			  //80、40、20、10分层 去噪
			  int m = inliers->indices.size();
			  while (m> 10)
			  {
				  //  std::cerr << "Could not estimate a circle model for the given dataset." << std::endl;
				  direction_xyzr.x = coefficients->values[0];
				  direction_xyzr.y = coefficients->values[1];
				  direction_xyzr.z = coefficients->values[2];
				  direction_xyzr.normal_x = coefficients->values[3];
				  direction_xyzr.normal_y = coefficients->values[4];
				  direction_xyzr.normal_z = coefficients->values[5];
				  direction_xyzr.curvature = coefficients->values[6];
				  D_XYZR->push_back(direction_xyzr);
				  // 分离树干点
				  extract.setInputCloud(cloud_cotree);
				  extract.setIndices(inliers);
				  extract.setNegative(false);
				  extract.filter(*cloud_cotree_true);
				  //连接两个点集
				  *cloud_out = *cloud_out + *cloud_cotree_true;
				  n++;

				  // 获取外层点
				  extract.setInputCloud(cloud_cotree);
				  extract.setIndices(inliers);
				  extract.setNegative(true);
				  extract.filter(*cloud_cotree);
				  //获取外层法线
				  extract_normals.setInputCloud(cloud_normal);
				  extract_normals.setIndices(inliers);
				  extract_normals.setNegative(true);
				  extract_normals.filter(*cloud_normal);
				 
				  //提取剩余的点
				  if (cloud_cotree->size() > 0)
				  {
				  
				  seg.setSamplesMaxDist(0.35, search);
				  search->setInputCloud(cloud_cotree);
				  seg.setInputNormals(cloud_normal);
				  seg.setInputCloud(cloud_cotree);
				  seg.setMaxIterations(10);
				  seg.segment(*inliers, *coefficients);
				  //控制while循环跳出条件
				  m = inliers->indices.size();
				  }
				  else{ m = 0; }
				  
			  }
		  }
	  }
  }
  pcl::io::savePCDFile<pcl::PointXYZ>("stem.pcd", *cloud_out);
 // pcl::io::savePCDFile("cylinder_Direction_xyz_R.pcd",*D_XYZR);
 // pcl::io::savePCDFile("source.pcd", *cloud);
  std::cout << "the number of selected octreeleaf is: " << n <<std::endl;
  end = clock();
  time = (end - start) / CLOCKS_PER_SEC;//这里的时间是计算机内部时间
  std::cout << "时间：" << time << "s" << std::endl;
return (0);
}
