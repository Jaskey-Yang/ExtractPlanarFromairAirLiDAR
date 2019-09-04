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
  *@ ��������Ϊ���ĵ��������Ϣ
  *@ { searchPoint				��x y z Intensity Nx Ny Nz ��
  *@   searchPoint				 ID
  *@   ������ֵ					 throedDist
  *@   �洢�����뾶�ڵĵ���Ϣ    {P1(x y z Intensity Nx Ny Nz),P1_Id;P2(x y z Intensity Nx Ny Nz),P2_Id...}
  *@   �洢�뾶����������ľ���  {P1_dis_searchPoint(Dis1),P2_dis_searchPoint(Dis2)....}
  *@  }
 */
	struct RadiusPoint
	{
		//�������ID��
		int pointID;
		//�������������Ϣ��ǿ��ֵ�ͷ�����
		pcl::PointXYZINormal point;
		//�����İ뾶
		float throedDist;
		//�洢�����뾶�ڵ��IDֵ
		std::vector<int> pointIdxRadiusSearch;
		//�洢���Ѱ뾶��ÿ�������������ľ���
		std::vector<float> pointRadiusquaredDistance;	
		//�洢������ĸ���������������Ϣ��ǿ��ֵ�ͷ�����
		std::vector<pcl::PointXYZINormal> knnPoints;
	};
	/*
	@��������ͽṹ��
	@������ֱ�洢x,y,z��u[3]
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
	*�̳�vector<Point>ģ����
	*/
	class PointCloud :public std::vector<Point>
	{
	public:
		/*
		*�̳�vector<Point>ģ����
		*�����ư�Χ�ж��㣨max_X,max_Y,max_Z��min_X,min_Y,min_Z��
		*param bbMin,bbMax (out)
		*����max_X,max_Y,max_Z��min_X,min_Y,min_Z
		*/
		void computeBuildingBox(Point& bbMin, Point& bbMax);
		bool computeNorOfTwoVector(const pcl::PointXYZINormal &pointA, const pcl::PointXYZINormal& pointB);
	};
}

#endif // !__Point__H_
