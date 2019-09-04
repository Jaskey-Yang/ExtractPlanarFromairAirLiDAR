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
	初始化参数的函数
	*/
	void init()
	{
		this->m_PointCloud = {};
	}
	/*
	滤波地面点
	*/
	void groundFilter();
	/*
	滤波非地面点
	*/
	void non_groundFilter();

private:
	std::vector<pcl::PointXYZ> m_PointCloud;

};

#endif// __POINTFILTER_H__