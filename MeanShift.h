#pragma once 

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
/*
*@ brief ->  2D����ṹ��
*/
struct Cluster 
{
    std::vector<double> mode;
    std::vector<std::vector<double> > original_points;
    std::vector<std::vector<double> > shifted_points;
};
/*
*@ brief ->  3D����ṹ��
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
	*@ brief   -> ��ά�����ֵƯ��
	*@ param   -> (in)std::vector<Point>& points ->   �������
	*@ param   -> (in)kerner_bandwidth           ->   �˺������� 
	*@ param   -> (in)EPSILON                    ->   
	*@ return  -> std::vector<Point> (std::vector<std::vector<double> >)  ->   ��ά����������
	*/
    std::vector<Point> meanshift(const std::vector<Point> & points,
                                                double kernel_bandwidth,
                                                double EPSILON = 0.00001);

	/*
	*@ brief   ->  ��ά�����ֵƯ��
	*@ param   ->  (in)std::vector<pcl::PointXYZI>& points ->   �������
	*@ param   ->  (in)kerner_bandwidth                    ->   �˺�������
	*@ param   ->  (in)EPSILON                             ->
	*@ return  ->   std::vector<pcl::PointXYZI>            ->   ��ά����������
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
