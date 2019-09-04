#pragma once
#ifndef  __POINTFILTER_H__
#define  __POINTFILTER_H__

#include<iostream>
#include<pcl/point_types.h>
#include<pcl/point_traits.h>
#include<vector>
class PointFilter
{
public:
	PointFilter();
	PointFilter(std::vector<pcl::PointXYZ> pointCloud)
	{
		this->m_PointCloud = pointCloud;
	}
	~PointFilter();
	/*
	��ʼ�������ĺ���
	*/
	void init()
	{
		this->m_PointCloud = {};
	}
	/*
	�˲������
	*/
	void groundFilter();
	/*
	�˲��ǵ����
	*/
	void non_groundFilter();

private:
	std::vector<pcl::PointXYZ> m_PointCloud;

};

#endif// __POINTFILTER_H__