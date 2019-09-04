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
	 @ ��ȡ���������
	 @ param   ->(out)outCloud 
	 @ Return				
	 @ ���Ϊ��ȡ�Ľ��������
	*/
	void extractBuild(std::vector<pcl::PointXYZ> &outCloud);
	void computeNeiborPoints(std::vector<pcl::PointXYZ> inCloud, std::vector<std::vector<pcl::PointXYZ> > &nKNNPoint);
	void computeMeanPoint(std::vector<std::vector<pcl::PointXYZ> > nKNNPoint, std::vector<pcl::PointXYZ> &meanKNNPoint);
	void Build::computeCorvianceMatrix( std::vector<pcl::PointXYZ> inCloud, 
										std::vector<pcl::PointXYZ> meanPoint,
										std::vector<Build::Matrix> & pointCloudMatrix);
	/**
	* @brief ��ʵ�Գƾ��������ֵ�������������ſ˱ȷ�
	* �����Ÿ��(Jacobi)������ʵ�Գƾ������������ֵ����������
	* @param pMatrix				����Ϊn*n�����顣���ʵ�Գƾ���
	* @param nDim					����Ľ���
	* @param pdblVects				����Ϊn*n�����飬������������(���д洢)
	* @param dbEps					����Ҫ��
	* @param nJt					���ͱ�������������������
	* @param pdbEigenValues			����ֵ����
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
	 *@brief ����ÿһ��������������
	 *@param (in)inputCloudPoint ���������ĵ�������
	 *@param (in)thredValue      �����������ֵ�뾶
	 *@return ���ش洢ÿһ�������������ĸ�����
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