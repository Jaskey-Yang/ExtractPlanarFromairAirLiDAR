#pragma once
#ifndef __EXTRACT_BUILD_H__
#define __EXTRACT_BUILD_H__

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include "Building.h"
#include "Point.h"
#include <vector>
#include<pcl/kdtree/kdtree_flann.h>
//template<class Type>
class Build
{
public:
	Build();
	virtual ~Build() {};
	struct Matrix
	{
		int i;
		pcl::PointXYZ point;
		union
		{
			double matrix[9];
			double M11, M12, M13, M21, M22, M23, M31, M32, M33;
		};
		Matrix() :M11(0.0), M12(0.0), M13(0.0), M21(0.0), M22(0.0), M23(0.0), M31(0.0), M32(0.0), M33(0.0) {}

	};
public:
	void setInputCloud(std::vector<pcl::PointXYZ> &inCloud)
	{
		this->m_Cloud = inCloud;
	}
	/*
	 @ 提取建筑物点云
	 @ param   ->(out)outCloud 
	 @ Return				
	 @ 输出为提取的建筑物点云
	*/
	void extractBuild(std::vector<pcl::PointXYZ> &outCloud);
	void computeNeiborPoints(std::vector<pcl::PointXYZ> inCloud, std::vector<std::vector<pcl::PointXYZ> > &nKNNPoint);
	void computeMeanPoint(std::vector<std::vector<pcl::PointXYZ> > nKNNPoint, std::vector<pcl::PointXYZ> &meanKNNPoint);
	void Build::computeCorvianceMatrix( std::vector<pcl::PointXYZ> inCloud, 
										std::vector<pcl::PointXYZ> meanPoint,
										std::vector<Build::Matrix> & pointCloudMatrix);
	/**
	* @brief 求实对称矩阵的特征值及特征向量的雅克比法
	* 利用雅格比(Jacobi)方法求实对称矩阵的所有特征值及特征向量
	* @param pMatrix				长度为n*n的数组。存放实对称矩阵
	* @param nDim					矩阵的阶数
	* @param pdblVects				长度为n*n的数组，返回特征向量(按列存储)
	* @param dbEps					精度要求
	* @param nJt					整型变量。控制最大迭代次数
	* @param pdbEigenValues			特征值数组
	* @return
	*/
	bool JacbiCor(double * pMatrix, int nDim, double *pdblVects, double *pdbEigenValues, double dbEps, int nJt);

	void Build::computeEigenVectorAndValue(const Eigen::Matrix3d matrix, Eigen::Vector3d &matrixVal, Eigen::Matrix3d  &matrixVec);


	void sortEigenVal(const Eigen::Matrix3d eigenvalue, 
					Eigen::Matrix3d::Index &eValMax,
					Eigen::Matrix3d::Index&eValMin);

	void extEigenMaxMinVec(const Eigen::Matrix3d matrixVec, 
							Eigen::Vector3d &maxVec, 
							Eigen::Vector3d &minVec, 
							const Eigen::Matrix3d::Index eValMax,
							const Eigen::Matrix3d::Index eValMin);

	/*
	 *@brief 计算每一个搜索点的邻域点
	 *@param (in)inputCloudPoint 输入待计算的点云数据
	 *@param (in)thredValue      输入的搜索阈值半径
	 *@return 返回存储每一个搜索点搜索的附近点
	*/
	std::vector<ExtBuild::RadiusPoint> computeNeiborPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inputCloudPoint, double thredValue);

	/*
	 *@
	 *@
	*/

private:
	std::vector<pcl::PointXYZ> &m_Cloud;



};



#endif //!__EXTRACT_BUILD_H__