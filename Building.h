#pragma once
#include<pcl/point_types.h>
#include<pcl/point_traits.h>
#include<pcl/io/ply_io.h>
#include<vector>
#include"Vec3d.h"
#include"Point.h"
using namespace std;
typedef pcl::PointXYZ pointT;

class Building
{
public:
	Building();
	~Building();
	/*
	*@计算每一个建筑屋顶的顶点坐标
	*@paramType  ->std::vector<pcl::PointXYZ> pointCloud
	*@paramType  ->std::vector<ExtBuild::Point>& vec4
	*@param      ->(in)pointCloud
	*@param      ->(out)vec4
	*@return     返回建筑物的边界顶点坐标
	*/
	void ComputeBoxVertexs(std::vector<pcl::PointXYZ> pointCloud, std::vector<ExtBuild::Point> &vec4);

	ExtBuild::Point get_LeftUpCorner() const
	{
		return LeftUpCorner;
	}
	/** \brief A point structure representing Euclidean xyz coordinates. (SSE friendly)
	* @param in group ,,,,
	* @param in group ,,,,
	* @param in group ,,,,
	* return 0，success;-1,false
	*/
	ExtBuild::Point get_RightUpCorner() const
	{
		return RightUpCorner;
	}

	ExtBuild::Point get_LeftDownCorner() const
	{
		return LeftDownCorner;
	}

	ExtBuild::Point get_RightDownCorner() const
	{
		return RightDownCorner;
	}

private:

	std::vector<pcl::PointXYZ> pointCloud;
	std::vector<ExtBuild::Point> Vertex4;
	ExtBuild::Point LeftUpCorner;
	ExtBuild::Point RightUpCorner;
	ExtBuild::Point LeftDownCorner;
	ExtBuild::Point RightDownCorner;
};

//class Solution {
//public: 
//	int integerBreak(int n)
//	{
//		if (n < 4)
//		{
//			return n - 1;
//		}
//		int count3 = 0;
//		if (n % 3 == 0) {
//			return (int)pow(3, n / 3);
//		}
//		else if (n % 3 == 1) {
//			return (int)pow(3, (n / 3 - 1)) * 4;
//		}
//		else {
//			return (int)pow(3, n / 3) * 2;
//		}
//	}
//	void setZeroes(std::vector<std::vector<int> > &matrix) {
//		int rows = matrix.size();
//		int cols = 0;
//		if (rows != 0) {
//			cols = matrix[0].size();
//		}
//		if (rows == 0 || cols == 0)
//			return;
//		for (int i = 0; i < rows; i++) {
//			for (int j = 0; j < cols; j++) {
//				if (matrix[i][j] == 0) {
//					for (int k = 0; k < cols; k++) {
//						if (matrix[i][k] != 0)
//						{   //跳过该行中其他值为0 的元素
//												   //if(k!=j){
//							matrix[i][k] = '*';
//						}
//					}
//					for (int k = 0; k < rows; k++) {
//						if (matrix[k][j] != 0) {
//							matrix[k][j] = '*';
//						}
//					}
//
//				}
//			}
//		}
//		for (int i = 0; i < rows; i++) {
//			for (int j = 0; j < cols; j++) {
//				if (matrix[i][j] == '*') {
//					matrix[i][j] = 0;
//				}
//			}
//		}
//	}
//
//};


