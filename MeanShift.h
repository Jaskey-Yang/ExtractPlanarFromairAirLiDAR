#pragma once 

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
/*
*@ brief ->  2D聚类结构体
*/
struct Cluster 
{
    std::vector<double> mode;
    std::vector<std::vector<double> > original_points;
    std::vector<std::vector<double> > shifted_points;
};
/*
*@ brief ->  3D聚类结构体
*/
struct Cluster3D
{
	pcl::PointXYZI mode;
	std::vector<pcl::PointXYZI> original_points;
	std::vector<pcl::PointXYZI> shifted_points;
};

class MeanShift {
public:
    typedef std::vector<double> Point;
	typedef pcl::PointXYZI Point3D;
    MeanShift() { set_kernel(NULL); }
    MeanShift(double (*_kernel_func)(double,double)) { set_kernel(kernel_func); }
	/*
	*@ brief   -> 二维坐标均值漂移
	*@ param   -> (in)std::vector<Point>& points ->   输入点云
	*@ param   -> (in)kerner_bandwidth           ->   核函数带宽 
	*@ param   -> (in)EPSILON                    ->   
	*@ return  -> std::vector<Point> (std::vector<std::vector<double> >)  ->   二维点坐标类型
	*/
    std::vector<Point> meanshift(const std::vector<Point> & points,
                                                double kernel_bandwidth,
                                                double EPSILON = 0.00001);

	/*
	*@ brief   ->  三维坐标均值漂移
	*@ param   ->  (in)std::vector<pcl::PointXYZI>& points ->   输入点云
	*@ param   ->  (in)kerner_bandwidth                    ->   核函数带宽
	*@ param   ->  (in)EPSILON                             ->
	*@ return  ->   std::vector<pcl::PointXYZI>            ->   三维点坐标类型
	*/
	std::vector<pcl::PointXYZI> meanshift3D(const std::vector<pcl::PointXYZI>&points,
		                                                  double kernel_bandwidth,
		                                                  double EPSILON = 0.00001);
    
	std::vector<Cluster3D> cluster3D(const std::vector<pcl::PointXYZI>&, double kernel_bandwidth);
	
	std::vector<Cluster> cluster(const std::vector<Point> &, double);


private:
    double (*kernel_func)(double,double);
    void set_kernel(double (*_kernel_func)(double,double));
    void shift_point(const Point&, const std::vector<Point> &, double, Point&);
	void shift_point(const Point3D&, const std::vector<Point3D> &, double, Point3D&);
    std::vector<Cluster> cluster(const std::vector<Point> &, const std::vector<Point> &);
	std::vector<Cluster3D> cluster3D(const std::vector<pcl::PointXYZI> &, const std::vector<pcl::PointXYZI> &);
};
