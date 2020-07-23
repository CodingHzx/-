//.pcd
//tree.pcd
//elevation_normalized.pcd
#include <iostream>
#include <ctime> //��ʱ�õ�ͷ�ļ�
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
	time_t start, end, time; /*ע���ʱ���õı�������*/
	/*����ʼִ�У���ʼ��ʱ*/
	start = clock();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>), cloud_cotree(new  pcl::PointCloud<pcl::PointXYZ>), cloud_cotree_true(new  pcl::PointCloud<pcl::PointXYZ>), cloud_f(new  pcl::PointCloud<pcl::PointXYZ>), cloud_out(new  pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr D_XYZR (new  pcl::PointCloud<pcl::PointNormal>);//Բ������ Բ����뾶
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>), normal_in(new pcl::PointCloud<pcl::Normal>), normal_out(new pcl::PointCloud<pcl::Normal>);

	pcl::PointNormal direction_xyzr;
  //pcl::io::loadPCDFile("L5_3_02.pcd", *cloud);
	pcl::io::loadPCDFile(argv[1], *cloud);
  std::cerr << "the number of data is :" << cloud->points.size() << std::endl;
  std::cout << "���ݼ������" << std::endl;
 
  //�����˲��� resolution�˲�����С
  typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;
  AlignedPointTVector voxel_center_list_arg;
  
  //octree���ش�С
  float resolution = 0.8f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  octree.setInputCloud(cloud);    //�����Ƹ���octree
  octree.addPointsFromInputCloud();  //����octree
  //��ȡoctree����
  voxel_center_list_arg.clear();
  octree.getOccupiedVoxelCenters(voxel_center_list_arg);
  pcl::PointXYZ center_point;
  
  std::cout << "the number of octree is :" << octree.getLeafCount() << std::endl;
  std::cout << "the number of octree is :" << voxel_center_list_arg.size() << std::endl;
  //������ȡ����
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  std::vector<int> pointIdxVec;
  
  int n = 0;
  
  for (int i = 0; i < voxel_center_list_arg.size(); i++)
  {
	  //ȷ��octree���ĵ�����
	  center_point.x = voxel_center_list_arg[i].x;
	  center_point.y = voxel_center_list_arg[i].y;
	  center_point.z = voxel_center_list_arg[i].z;

	  if (octree.voxelSearch(center_point, pointIdxVec))
	  {
		  //������
		  if (pointIdxVec.size()>40)
		  {
			  //std::cout << "duandian1"<< std::endl;
			  //��reset()�������˽�
			  pcl::copyPointCloud(*cloud, pointIdxVec, *cloud_cotree);
			  //����ڴ�
			  std::vector<int>().swap(pointIdxVec);
			  //�������߹��ƶ���
			  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
			  normal_est.setNumberOfThreads(8);
			  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
			  normal_est.setInputCloud(cloud_cotree);
			  normal_est.setSearchMethod(tree);
			  normal_est.setRadiusSearch(0.1);
			  normal_est.compute(*cloud_normal);
			 // std::vector<int>().swap(pointIdxVec);
			  // std::cout << "duandian1" << std::endl;
			   //std::cout << "���Ʒ��� ���" << std::endl;
			  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			  //�����ָ����
			  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
			   pcl::SACSegmentation<pcl::PointXYZ>::SearchPtr search(new pcl::search::KdTree<pcl::PointXYZ >());
			  //��ѡ����
			  seg.setOptimizeCoefficients(true);
			  //��������
			  seg.setModelType(pcl::SACMODEL_CYLINDER);
			//  seg.setMethodType(pcl::SAC_RANSAC);
			  seg.setMethodType(pcl::SAC_PROSAC);
			  //ƫ��ģ�;��� �����������Ҫ
			  seg.setDistanceThreshold(0.015);
			  //ģ�Ͱ뾶����
			  ////seg.setRadiusLimits(0.03, 0.2); 
			  seg.setRadiusLimits(0.05, 0.4);

			  //�ֲ������뾶
			  seg.setSamplesMaxDist(0.35, search);  ////�����������ʱ����֮�������������Ϊradius �� search Ϊ����ʱʹ�õ���������
			  search->setInputCloud(cloud_cotree);
			  seg.setInputNormals(cloud_normal);
			  seg.setInputCloud(cloud_cotree);
			  seg.setMaxIterations(10);
			  seg.segment(*inliers, *coefficients);
			  //80��40��20��10�ֲ� ȥ��
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
				  // �������ɵ�
				  extract.setInputCloud(cloud_cotree);
				  extract.setIndices(inliers);
				  extract.setNegative(false);
				  extract.filter(*cloud_cotree_true);
				  //���������㼯
				  *cloud_out = *cloud_out + *cloud_cotree_true;
				  n++;

				  // ��ȡ����
				  extract.setInputCloud(cloud_cotree);
				  extract.setIndices(inliers);
				  extract.setNegative(true);
				  extract.filter(*cloud_cotree);
				  //��ȡ��㷨��
				  extract_normals.setInputCloud(cloud_normal);
				  extract_normals.setIndices(inliers);
				  extract_normals.setNegative(true);
				  extract_normals.filter(*cloud_normal);
				 
				  //��ȡʣ��ĵ�
				  if (cloud_cotree->size() > 0)
				  {
				  
				  seg.setSamplesMaxDist(0.35, search);
				  search->setInputCloud(cloud_cotree);
				  seg.setInputNormals(cloud_normal);
				  seg.setInputCloud(cloud_cotree);
				  seg.setMaxIterations(10);
				  seg.segment(*inliers, *coefficients);
				  //����whileѭ����������
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
  time = (end - start) / CLOCKS_PER_SEC;//�����ʱ���Ǽ�����ڲ�ʱ��
  std::cout << "ʱ�䣺" << time << "s" << std::endl;
return (0);
}
