#pragma once
#ifndef __POINT__H_
#define __POINT__H_

#include<vector>
#include<pcl/point_types.h>
#include<pcl/point_traits.h>
#include<cmath>
namespace ExtBuild
{
	/*
  *@ 以搜索点为中心的邻域点信息
  *@ { searchPoint				（x y z Intensity Nx Ny Nz ）
  *@   searchPoint				 ID
  *@   搜索阈值					 throedDist
  *@   存储搜索半径内的点信息    {P1(x y z Intensity Nx Ny Nz),P1_Id;P2(x y z Intensity Nx Ny Nz),P2_Id...}
  *@   存储半径内与搜索点的距离  {P1_dis_searchPoint(Dis1),P2_dis_searchPoint(Dis2)....}
  *@  }
 */
	struct RadiusPoint
	{
		//搜索点的ID号
		int pointID;
		//搜索点的坐标信息、强度值和法向量
		pcl::PointXYZINormal point;
		//搜索的半径
		float throedDist;
		//存储搜索半径内点的ID值
		std::vector<int> pointIdxRadiusSearch;
		//存储搜搜半径内每个点距离搜索点的距离
		std::vector<float> pointRadiusquaredDistance;	
		//存储搜索点的附近邻域点的坐标信息、强度值和法向量
		std::vector<pcl::PointXYZINormal> knnPoints;
	};
	/*
	@定义点类型结构体
	@联合体分别存储x,y,z和u[3]
	*/
	struct Point
	{
		union 
		{
			struct {
				double x;
				double y; 
				double z;
			};
			double u[3];
		};
		Point():x(0), y(0), z(0) {}
	};
	/*
	*继承vector<Point>模板类
	*/
	class PointCloud :public std::vector<Point>
	{
	public:
		/*
		*继承vector<Point>模板类
		*求解点云包围盒顶点（max_X,max_Y,max_Z和min_X,min_Y,min_Z）
		*param bbMin,bbMax (out)
		*返回max_X,max_Y,max_Z和min_X,min_Y,min_Z
		*/
		void computeBuildingBox(Point& bbMin, Point& bbMax);
		bool computeNorOfTwoVector(const pcl::PointXYZINormal &pointA, const pcl::PointXYZINormal& pointB);
	};
}

#endif // !__Point__H_
