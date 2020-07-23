//plane.pcd source_point.pcd nearSearch(int)
//plane_out.pcd tree.pcd 15
//����XY��������

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
	//Eigen::MatrixXf a;
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>), source_point(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXY>::Ptr plane_XY(new pcl::PointCloud<pcl::PointXY>), source_point_XY(new pcl::PointCloud<pcl::PointXY>);

	//��ǰ������
	pcl::PointXY search_point;
	pcl::io::loadPCDFile(argv[1],*plane);
	pcl::io::loadPCDFile(argv[2],*source_point);
	std::cout << "���ƶ�ȡ������" << std::endl;
	
	clock_t start = clock();
	//plane_XY->width = plane->width; plane_XY->height = plane->height; plane_XY->resize(plane_XY->width*plane_XY->height);
	//source_point_XY->width = source_point->width; source_point_XY->height = source_point->height; source_point_XY->resize(source_point_XY->width*source_point_XY->height);
	
	//��XYZ��XYcopy����Ӧ������
	pcl::copyPointCloud(*plane,*plane_XY);
	pcl::copyPointCloud(*source_point,*source_point_XY);
	
	std::vector<int> nearKindex; std::vector<float> nearKsquareDistance;
	//�ڽ�������Kֵ
	int nearSearchNum = strtol(argv[3], 0, 10);
	pcl::KdTreeFLANN<pcl::PointXY> KDtree;
	KDtree.setInputCloud(plane_XY);

	//�˴���mean_z��Ϊxy��Ӧ�ĵ���zֵ����
	float mean_z,sum_z=0;
	for (size_t i = 0; i < source_point_XY->size(); i++)
	{
		if (KDtree.nearestKSearch(source_point_XY->points[i], nearSearchNum, nearKindex, nearKsquareDistance)){
			for (size_t j = 0; j < nearKindex.size(); j++)
				sum_z = sum_z + plane->points[nearKindex[j]].z;
		}
		mean_z = sum_z / nearKindex.size();
		//zֵ���˹�һ������
		//std::cout << "mean_z :" << mean_z << std::endl;
		source_point->points[i].z = source_point->points[i].z - mean_z;
		
		//Ϊ��һ�ֵ�����׼��
		sum_z = 0; std::vector<int>().swap(nearKindex); std::vector<float>().swap(nearKsquareDistance);
		
	}
	
	clock_t end = clock();
	std::cout << "������ʱ��" << (end - start) / CLOCKS_PER_SEC <<" s"<< std::endl;

	pcl::io::savePCDFile("elevation_normalized.pcd",*source_point);
	system("pause");
	return 0;
}