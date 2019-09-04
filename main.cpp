#include<iostream>
#include<pcl/io/ply_io.h>
#include<liblas/reader.hpp>
#include<liblas/liblas.hpp>
#include<fstream>
#include"Readlas.h"
#include<string>
#include<pcl/point_traits.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include<pcl/features/normal_3d.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<map>
#include<pcl/segmentation/region_growing.h>
#include"Point.h"
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/segmentation/progressive_morphological_filter.h>

std::vector<std::vector<double> > computeNorbNormdiff(const std::vector<ExtBuild::RadiusPoint> radiuspoint)
{
	std::cout << "" << radiuspoint.size() << std::endl;
	int k = radiuspoint.size();
	//计算搜索点与邻域点法向量之间的夹角
	std::vector<std::vector<double> >norbNormdiff;
	for (int i = 0; i < k; i++)
	{
		//norbNormdiff[i]
		double norA = sqrt(pow(radiuspoint.at(i).point.normal_x, 2) + pow(radiuspoint.at(i).point.normal_y, 2) + pow(radiuspoint.at(i).point.normal_z, 2));
		int M = radiuspoint.at(i).knnPoints.size();
		std::vector<double> temp;
		if (M == 0) continue;
		std::cout << i << "个点" <<std::endl;
		for (int j = 0; j < M; j++)
		{
			if(abs(radiuspoint.at(i).knnPoints[j].normal_x)<=0.00001|| abs(radiuspoint.at(i).knnPoints[j].normal_y) <= 0.00001 || abs(radiuspoint.at(i).knnPoints[j].normal_z) <= 0.00001)
			{ 
				double norB = sqrt(pow(radiuspoint.at(i).knnPoints[j].normal_x, 2) + pow(radiuspoint.at(i).knnPoints[j].normal_y, 2) + pow(radiuspoint.at(i).knnPoints[j].normal_z, 2));
				double nCross = radiuspoint.at(i).point.normal_x*radiuspoint.at(i).knnPoints[j].normal_x + radiuspoint.at(i).point.normal_y*radiuspoint.at(i).knnPoints[j].normal_y + radiuspoint.at(i).point.normal_z * radiuspoint.at(i).knnPoints[j].normal_z;
				double corner = (double)(nCross / (norA*norB));
				
				std::cout << "    " << corner<< std::endl;
				if (abs(corner)<=0.00001 ) continue;
				temp.push_back(corner);
			}
		}
		norbNormdiff.push_back(temp);
		
	}
	std::cout << "目前点的个数为;" << norbNormdiff.size() << std::endl;
	return norbNormdiff;

}
int main()
{
	std::cout << "####读取激光点云文件.Las文件####" << std::endl;
	std::cout << "---------------------------------------" << std::endl;
	DataIO dataIO;
	std::string FilePath ="D:/test2.las";
	std::cout << "####点云数据开始读取..." << std::endl;
	dataIO.SetFilePath(FilePath);
	std::vector<pcl::PointXYZI> inputCloud;
	dataIO.ReadlasXYZI(inputCloud);
	std::cout << "####点云数据读取完成####"<<std::endl<<std::endl;
	std::cout << "----------------------------------------------" << std::endl;

	std::cout << "####根据点云的高程值Z和强度值Intensity进行过滤点云####" << std::endl;
	std::cout << "----------------------------------------------" << std::endl;
	/*点云数据根据点的Z高程值和强度值进行筛选*/
	/*filterPoints改变原先点的索引值，进行重新排序*/
	std::map<int, pcl::PointXYZI> filterPoints;
	std::vector<int> indexs;
	int count = 0;
	/*inputCloud2过滤点云不改变原先的点索引值*/
	std::vector<pcl::PointXYZI> inputCloud2;
	for (int i = 0; i < inputCloud.size(); i++)
	{
		if (inputCloud[i].z >20.0 && inputCloud[i].intensity<150.0)
		{
			indexs.push_back(i);
			inputCloud2.push_back(inputCloud[i]);
			
			filterPoints.insert(std::pair<int, pcl::PointXYZI>(count, inputCloud[i]));
			count++;
		}
	}
	std::cout << "####不改变原始点云索引的存储点云的数量大小为: " << inputCloud2.size() << std::endl << std::endl;
	std::cout << "####改变了原始点云索引的存储点云的数量大小为: " << filterPoints.size() << std::endl<<std::endl;

	/*将点云自定义存储结构转换为PCL中点云存储结构*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr  pcl_inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl_inputCloud->height = count;
	pcl_inputCloud->width = 1;
	pcl_inputCloud->resize(pcl_inputCloud->height*pcl_inputCloud->width);
	std::cout << "####将点云自定义存储结构转换为PCL中点云存储结构####"<<std::endl;
	std::cout << "----------------------------------------------" << std::endl;
	for (int i = 0; i < pcl_inputCloud->size(); i++)
	{
		pcl_inputCloud->points[i] = inputCloud2[i];
		//pcl_inputCloud->at(i).x = inputCloud2[i].x;
		//pcl_inputCloud->at(i).y = inputCloud2[i].y;
		//pcl_inputCloud->at(i).z = inputCloud2[i].z;
		//pcl_inputCloud->at(i).intensity = inputCloud2[i].intensity;
	}
	std::cout<<std::endl;
	if (pcl_inputCloud->size() == 0 || pcl_inputCloud->size() != inputCloud2.size())
	{
		std::cout << "存储结构转换失败!!!" << std::endl;
		std::cout << "##############################################" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "存储结构转换成功"    << std::endl;
		std::cout << "####转换存储格式后点云的大小："<<pcl_inputCloud->size() << std::endl;
		std::cout << "##############################################" << std::endl;
	}
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
	pmf.setInputCloud(pcl_inputCloud);
	pmf.setMaxWindowSize(20);
	pmf.setSlope(1.0f);
	pmf.setInitialDistance(0.5f);
	pmf.setMaxDistance(3.0f);
	pmf.extract(ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(pcl_inputCloud);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI>("samp11-utm_ground.pcd", *cloud_filtered, false);

	// Extract non-ground returns
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	writer.write<pcl::PointXYZI>("samp11-utm_object.pcd", *cloud_filtered, false);
	
	std::cout << "####计算点云法向量####" << std::endl;
	std::cout << "----------------------------------------------" << std::endl;
	std::cout << "####建立法向量估计对象……" << std::endl;
	//compute point Normals
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	std::cout << "####建立点云搜索结构KdTree……" << std::endl;
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	//为kdtree添加点运数据
	tree->setInputCloud(cloud_filtered);
	n.setInputCloud(cloud_filtered);
	n.setSearchMethod(tree);
	std::cout << "####点云法向量开始计算……" << std::endl;
	//点云法向计算时，需要所搜的近邻点大小
	n.setRadiusSearch(2.0);
	//n.setKSearch(15);
	//开始进行法向计算
	n.compute(*normals);
	std::cout << "####点云法向量计算完成####" << std::endl;
	std::cout << "----------------------------------------------" << std::endl;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointcloudN(new pcl::PointCloud<pcl::PointXYZINormal>);
	std::cout << "####合并点云坐标信息和法向量####" << std::endl;
	std::cout << "----------------------------------------------" << std::endl;
	pointcloudN->resize(cloud_filtered->size());
	for (int i = 0; i < cloud_filtered->size(); i++)
	{
		pointcloudN->at(i).x = cloud_filtered->at(i).x;
		pointcloudN->at(i).y = cloud_filtered->at(i).y;
		pointcloudN->at(i).z = cloud_filtered->at(i).z;
		pointcloudN->at(i).intensity = pcl_inputCloud->at(i).intensity;
		pointcloudN->at(i).normal_x = normals->at(i).normal_x;
		pointcloudN->at(i).normal_y = normals->at(i).normal_y;
		pointcloudN->at(i).normal_z = normals->at(i).normal_z;
		//pointcloudN->at(i).normal = normals->at(i).normal;
	}
	std::cout << "####点云坐标和对应法向量信息合并完成####" << std::endl;
	std::cout << "##############################################" << std::endl;
	
	std::cout << "####计算每个点的邻域值####" << std::endl;
	std::vector<ExtBuild::RadiusPoint> radiuspoint;
	//radiuspoint=
	pcl::KdTreeFLANN<pcl::PointXYZINormal> rkdtree;
	rkdtree.setInputCloud(pointcloudN);
	double Radius = 2.0;							
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusquaredDistance;
	for (int i = 0; i < pointcloudN->size(); i++)
	{
		ExtBuild::RadiusPoint tempRadiuspoint;
		if (rkdtree.radiusSearch(pointcloudN->points[i], Radius, pointIdxRadiusSearch, pointRadiusquaredDistance) > 0)
		{
			//搜索点的ID
			tempRadiuspoint.pointID = i;
			//搜索距离
			tempRadiuspoint.throedDist = Radius;
			{
				tempRadiuspoint.point.x = pointcloudN->points[i].x;
				tempRadiuspoint.point.y = pointcloudN->points[i].x;
				tempRadiuspoint.point.z = pointcloudN->points[i].x;
																   
				tempRadiuspoint.point.normal_x = pointcloudN->points[i].normal_x;
				tempRadiuspoint.point.normal_y = pointcloudN->points[i].normal_y;
				tempRadiuspoint.point.normal_z = pointcloudN->points[i].normal_z;
																				 
			}
			tempRadiuspoint.pointIdxRadiusSearch = pointIdxRadiusSearch;
			tempRadiuspoint.pointRadiusquaredDistance = pointRadiusquaredDistance;
			for (int j = 1; j < pointIdxRadiusSearch.size(); j++)
			{
				tempRadiuspoint.knnPoints.push_back(pointcloudN->at(pointIdxRadiusSearch[j]));
			}
		}
		radiuspoint.push_back(tempRadiuspoint);
	}
	std::cout << "搜索点的个数"<<radiuspoint.size() << std::endl;
	for (int i = 0; i < radiuspoint.size(); i++)
	{
		std::cout << "第" << i << "个搜索点在" << radiuspoint.at(i).throedDist << "范围内对应的邻域点个数是：" << radiuspoint.at(i).knnPoints.size() << std::endl;
	}
	
	std::vector<std::vector<double> >cornerNorms;

	cornerNorms=computeNorbNormdiff(radiuspoint);
	//for (int i = 0; i < radiuspoint.size(); i++)
	//{
	//	for (int j = 0; j < radiuspoint.at(i).pointRadiusquaredDistance.size(); j++)
	//	{
	//		std::cout << radiuspoint.at(i).pointRadiusquaredDistance[j] << std::endl;
	//	}
	//}



	//std::cout << "####VoxelGrid降采样点云数据####" << std::endl;
	////Creat the filtering object:downsample the dataset using a leaf size of
	////创建降采样对象，通过设置叶结点来降采样输入数据集
	//pcl::VoxelGrid<pcl::PointXYZI> vg;
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);
	//vg.setInputCloud(pcl_inputCloud);
	//vg.setLeafSize(1.0f, 1.0f, 1.0f);
	//vg.filter(*cloud_filter);
	//std::cout << "点云降采样后点云数量为: " << cloud_filter->points.size() << " data points." << std::endl;
	//
	//std::cout << "####计算点云法向量####" <<std::endl;
	//std::cout << "----------------------------------------------" << std::endl;
	//std::cout << "####建立法向量估计对象……" <<std::endl;
	////compute point Normals
	//pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//std::cout << "####建立点云搜索结构KdTree……" << std::endl;
	////建立kdtree来进行近邻点集搜索
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	////为kdtree添加点运数据
	//tree->setInputCloud(cloud_filter);
	//n.setInputCloud(cloud_filter);
	//n.setSearchMethod(tree);
	//std::cout << "####点云法向量开始计算……" << std::endl;
	////点云法向计算时，需要所搜的近邻点大小
	////n.setRadiusSearch(0.5);
	//n.setKSearch(10);
	////开始进行法向计算
	//n.compute(*normals);
	//std::cout << "####点云法向量计算完成####"<<std::endl;
	//std::cout << "----------------------------------------------" << std::endl;
	//pcl::PointCloud<pcl::PointXYZINormal>::Ptr pointcloudN(new pcl::PointCloud<pcl::PointXYZINormal>);
	//std::cout << "####合并点云坐标信息和法向量####" << std::endl;
	//std::cout << "----------------------------------------------" << std::endl;
	//pointcloudN->resize(cloud_filter->size());
	//for (int i = 0; i < cloud_filter->size(); i++)
	//{
	//	pointcloudN->at(i).x = pcl_inputCloud->at(i).x;
	//	pointcloudN->at(i).y = pcl_inputCloud->at(i).y;
	//	pointcloudN->at(i).z = pcl_inputCloud->at(i).z;
	//	pointcloudN->at(i).intensity = pcl_inputCloud->at(i).intensity;
	//	pointcloudN->at(i).normal_x = normals->at(i).normal_x;
	//	pointcloudN->at(i).normal_y = normals->at(i).normal_y;
	//	pointcloudN->at(i).normal_z = normals->at(i).normal_z;
	//    //pointcloudN->at(i).normal = normals->at(i).normal;
	//}
	//std::cout << "####合并信息完成####" << std::endl;
	//std::cout << "##############################################" << std::endl;
	////std::map<std::map<int, pcl::PointXYZINormal>, std::vector<pcl::PointXYZINormal>> KNNPoint;
	////std::pair<std::map<int, pcl::PointXYZINormal>, std::vector<pcl::PointXYZINormal>> value(std::pair<int, pcl::PointXYZINormal>value2(int(i), pcl::PointXYZINormal(pointcloudN->points[i])), {});
	////KNNPoint.insert(value);

	///*KNN搜索*/
	////pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
	////kdtree.setInputCloud(pointcloudN);
	////int K = 10;
	////std::vector<int> pointIdxKNNSearch(K);
	////std::vector<float> pointNKNSquaredDistance(K);
	////for (int i = 0; i < pointcloudN->size(); i++)
	////{
	////	std::cout << "K nearest neighbor search at (" << pointcloudN->points[i].x
	////												<< " " << pointcloudN->points[i].y
	////												<< " " << pointcloudN->points[i].z
	////												<< ") with K=" << K << std::endl;
	////	if (kdtree.nearestKSearch(pointcloudN->points[i], K, pointIdxKNNSearch, pointNKNSquaredDistance) > 0)
	////	{
	////		for (size_t i = 0; i < pointIdxKNNSearch.size(); ++i)
	////			std::cout << "    " << pointcloudN->points[pointIdxKNNSearch[i]].x
	////						<< " " << pointcloudN->points[pointIdxKNNSearch[i]].y
	////						<< " " << pointcloudN->points[pointIdxKNNSearch[i]].z
	////						<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	////
	////	}
	////}
	///*Radius搜索*/
	//std::cout << "####计算每个点的邻域值####" << std::endl;
	//std::vector<ExtBuild::RadiusPoint> radiuspoint;
	//pcl::KdTreeFLANN<pcl::PointXYZINormal> rkdtree;
	//rkdtree.setInputCloud(pointcloudN);
	//double Radius =10.0;
	//std::vector<int> pointIdxRadiusSearch;
	//std::vector<float> pointRadiusquaredDistance;
	//for (int i = 0; i < pointcloudN->size(); i++)
	//{
	//	//std::cout << "Radius search at (" << pointcloudN->points[i].x
	//	//	<< " " << pointcloudN->points[i].y
	//	//	<< " " << pointcloudN->points[i].z
	//	//	<< ") with Radius=" << Radius << std::endl;
	//	if (rkdtree.radiusSearch(pointcloudN->points[i], Radius, pointIdxRadiusSearch, pointRadiusquaredDistance) > 0)
	//	{
	//		//std::cout << "搜索半径" << Radius << "内的点数为：" << pointIdxRadiusSearch.size() - 1 << std::endl;
	//		//for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	//		//	std::cout << "    " << pointcloudN->points[pointIdxRadiusSearch[i]].x
	//		//	<< " " << pointcloudN->points[pointIdxRadiusSearch[i]].y
	//		//	<< " " << pointcloudN->points[pointIdxRadiusSearch[i]].z
	//		//	<< " (squared distance: " << pointRadiusquaredDistance[i] << ")" << std::endl;
	//		ExtBuild::RadiusPoint tempRadiuspoint;
	//		tempRadiuspoint.pointID = i;
	//		tempRadiuspoint.throedDist = Radius;
	//		{
	//			tempRadiuspoint.point.x = pointcloudN->points[i].x;
	//			tempRadiuspoint.point.y = pointcloudN->points[i].x;
	//			tempRadiuspoint.point.z = pointcloudN->points[i].x;
	//			//tempRadiuspoint.point.intensity = pointcloudN->points[i].intensity;
	//			tempRadiuspoint.point.normal_x = pointcloudN->points[i].normal_x;
	//			tempRadiuspoint.point.normal_y = pointcloudN->points[i].normal_y;
	//			tempRadiuspoint.point.normal_z = pointcloudN->points[i].normal_z;
	//			//std::cout << "tempRadiuspoint.point.normal_z: "<< tempRadiuspoint.point.normal_x <<std::endl;
	//		}
	//		for (int j = 1; j < pointIdxRadiusSearch.size(); j++)
	//		{
	//			if (pointRadiusquaredDistance[j] < Radius)
	//			{
	//				tempRadiuspoint.pointIdxRadiusSearch.push_back(pointIdxRadiusSearch[j]);
	//				tempRadiuspoint.pointRadiusquaredDistance.push_back(pointRadiusquaredDistance[j]);
	//			}
	//		}
	//		radiuspoint.push_back(tempRadiuspoint);
	//	}
	//}
	////std::cout << "RadiusPoints size is " << radiuspoint.at(0).pointIdxRadiusSearch.size() << std::endl;
	//std::cout << "####邻域值计算完成####" << std::endl;
	//std::cout << "----------------------------------------------" << std::endl;

	//

	////Creat the segmentation object for planner model and set all the param
	//pcl::SACSegmentation<pcl::PointXYZI> seg;
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_palne(new pcl::PointCloud<pcl::PointXYZI>);
	//seg.setOptimizeCoefficients(true);
	//seg.setModelType(pcl::SACMODEL_PLANE);
	//seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(300);
	//seg.setDistanceThreshold(0.01);
	//pcl::PCDWriter writer;
	////区域生长分割 
	//pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
	//reg.setMinClusterSize(50);
	//reg.setMaxClusterSize(10000);
	//reg.setSearchMethod(tree);
	//reg.setNumberOfNeighbours(30);
	//reg.setInputCloud(cloud_filter);
	////reg.setIndices (indices);
	//reg.setInputNormals(normals);
	//reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	//reg.setCurvatureThreshold(1.0);

	//std::vector <pcl::PointIndices> clusters;
	//reg.extract(clusters);
	//std::cout << "####聚类的数量为： " << clusters.size() << std::endl;
	//std::cout << "####第一类有" << clusters[0].indices.size() << " 个点." << std::endl;
	//std::cout << "These are the indices of the points of the initial" <<
	//	std::endl << "cloud that belong to the first cluster:" << std::endl;
	//int counter = 0;
	//while (counter < clusters[0].indices.size())
	//{
	//	std::cout << clusters[0].indices[counter] << ", ";
	//	counter++;
	//	if (counter % 10 == 0)
	//		std::cout << std::endl;
	//}
	//std::cout << std::endl;

	//int i = 0, nr_points = (int)cloud_filter->points.size();
	//std::stringstream ss;
	//const char* filepath = "D:/output";
	////mkdir(filepath);

	//
	//while (cloud_filter->points.size() > 0)
	//{
	//	// Segment the largest planar component from the remaining cloud
	//	seg.setInputCloud(cloud_filter);
	//	seg.segment(*inliers, *coefficients);
	//	if (inliers->indices.size() == 0)
	//	{
	//		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	//		break;
	//	}
	//	//std::cerr << "Model coefficients: " << coefficients->values[0] << " "
	//	//									<< coefficients->values[1] << " "
	//	//									<< coefficients->values[2] << " "
	//	//									<< coefficients->values[3] << std::endl;
	//	//
	//	//std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
	//	// Extract the planar inliers from the input cloud
	//	pcl::ExtractIndices<pcl::PointXYZI> extract;
	//	std::string s1 = filepath;
	//	char szFileName[30] = { 0 };
	//	std::string filepath2;
	//	sprintf(szFileName, "OUTPUT_%06d.ply", i);
	//	filepath2 = s1 + "\\" + szFileName;

	//	extract.setInputCloud(cloud_filter);
	//	extract.setIndices(inliers);
	//	extract.setNegative(false);


	//	// Get the points associated with the planar surface
	//	extract.filter(*cloud_palne);

	//	if (cloud_palne->points.size() > 0)
	//	{
	//		std::cout << "PointCloud representing the planar component: " << cloud_palne->points.size() << " data points." << std::endl;

	//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
	//		colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	//		std::vector<unsigned int> colors;
	//		colors.push_back(static_cast<unsigned int> (rand() % 256));
	//		colors.push_back(static_cast<unsigned int> (rand() % 256));
	//		colors.push_back(static_cast<unsigned int> (rand() % 256));

	//		colored_cloud->width = cloud_palne->width;
	//		colored_cloud->height = cloud_palne->height;

	//		colored_cloud->is_dense = cloud_palne->is_dense;
	//		for (size_t i_point = 0; i_point < cloud_palne->points.size(); i_point++)
	//		{
	//			pcl::PointXYZRGB point;
	//			point.x = *(cloud_palne->points[i_point].data);
	//			point.y = *(cloud_palne->points[i_point].data + 1);
	//			point.z = *(cloud_palne->points[i_point].data + 2);
	//			point.r = colors[0];
	//			point.g = colors[1];
	//			point.b = colors[2];
	//			colored_cloud->points.push_back(point);
	//		}
	//		pcl::io::savePLYFileASCII(filepath2, *colored_cloud);
	//		++i;
	//		// Remove the planar inliers, extract the rest
	//		extract.setNegative(true);
	//		extract.filter(*cloud_f);
	//		*cloud_filter = *cloud_f;
	//	}
	//	
	//	
	//}
	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZI>());
	//tree->setInputCloud(cloud_filter);

	//std::vector<pcl::PointIndices>cluster_indices;
	//pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	//ec.setClusterTolerance(0.02);
	//ec.setMinClusterSize(100);
	//ec.setMaxClusterSize(10000);
	//ec.setSearchMethod(tree2);
	//ec.setInputCloud(cloud_filter);
	//ec.extract(cluster_indices);

	//int j = 0;
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
	//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	//		cloud_cluster->points.push_back(cloud_filter->points[*pit]); //*
	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	//	std::stringstream ss;
	//	ss << "cloud_cluster_" << j << ".pcd";
	//	writer.write<pcl::PointXYZI>(ss.str(), *cloud_cluster, false); //*
	//	j++;
	//}
	//pcl::PLYWriter writer;

	system("pause");
	return 0;
}