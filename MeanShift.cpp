#include <stdio.h>
#include <math.h>
#include "MeanShift.h"

using namespace std;

#define CLUSTER_EPSILON 0.5
///////////////////////////////////////////////////////////////////////////////////////
/*
*@ brief  ->计算三维坐标的欧氏距离(开方后坐标)
*@ param  ->(in) point_a  ->输入的点A（pcl::PointXYZI）
*@ param  ->(in) point_b  ->输入的点B（pcl::PointXYZI）
*@ return ->(double)      ->sqrt(pow((a.x - b.x),2)+pow((a.y - b.y),2)+pow((a.z - b.z),2))
*/
double euclidean_distance3D(const pcl::PointXYZI &point_a, const pcl::PointXYZI &point_b) {
	double total = 0;
	total=pow((point_a.x - point_b.x),2)+pow((point_a.y - point_b.y),2)+pow((point_a.z - point_b.z),2);
	return sqrt(total);
}
/*
*@ brief  ->计算三维坐标的欧氏距离
*@ param  ->(in) point_a  ->输入的点A（pcl::PointXYZI）
*@ param  ->(in) point_b  ->输入的点B（pcl::PointXYZI）
*@ return ->(double)      ->(pow((a.x - b.x),2)+pow((a.y - b.y),2)+pow((a.z - b.z),2))
*/
double euclidean_distance3D_sqr(const pcl::PointXYZI &point_a, const pcl::PointXYZI &point_b)
{
	double total = 0;
	total = pow((point_a.x - point_b.x), 2) + pow((point_a.y - point_b.y), 2) + pow((point_a.z - point_b.z), 2);
	return (total);
}
//////////////////////////////////////////////////////////////////////////////////////
/*
*@ brief  ->计算二维或者三维坐标的欧氏距离
*@ param  ->(in) point_a  ->输入的点A(std::vector<double>& )
*@ param  ->(in) point_b  ->输入的点B(std::vector<double>& )
*@ return ->(double)      ->sqrt(pow((a.x - b.x),2)+pow((a.y - b.y),2)+pow((a.z - b.z),2))
*/
double euclidean_distance(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return sqrt(total);
}

/*
*@ brief  ->计算二维或者三维坐标的欧氏距离
*@ param  ->(in) point_a  ->输入的点A(std::vector<double>& )
*@ param  ->(in) point_b  ->输入的点B(std::vector<double>& )
*@ return ->(double)      ->(pow((a.x - b.x),2)+pow((a.y - b.y),2)+pow((a.z - b.z),2))
*/
double euclidean_distance_sqr(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        const double temp = (point_a[i] - point_b[i]);
        total += temp*temp;
    }
    return (total);
}
/////////////////////////////////////////////////////////////
double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0/2.0 * (distance*distance) / (kernel_bandwidth*kernel_bandwidth));
    return temp;
}

void MeanShift::set_kernel( double (*_kernel_func)(double,double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;    
    }
}

void MeanShift::shift_point(const Point &point,
                            const std::vector<Point> &points,
                            double kernel_bandwidth,
                            Point &shifted_point) {
    shifted_point.resize( point.size() ) ;
    for(int dim = 0; dim<shifted_point.size(); dim++){
        shifted_point[dim] = 0;
    }
    double total_weight = 0;
#pragma omp parallel for;
    for(int i=0; i<points.size(); i++)
	{
        const Point& temp_point = points[i];
        double distance = euclidean_distance(point, temp_point);
        double weight = kernel_func(distance, kernel_bandwidth);
        for(int j=0; j<shifted_point.size(); j++)
		{
            shifted_point[j] += temp_point[j] * weight;
        }
        total_weight += weight;
    }

    const double total_weight_inv = 1.0/total_weight;
    for(int i=0; i<shifted_point.size(); i++){
        shifted_point[i] *= total_weight_inv;
    }
}

void MeanShift::shift_point(const MeanShift::Point3D& search_points, const std::vector<MeanShift::Point3D> & src_points, double kernal_bandwidth, MeanShift::Point3D& shift_points)
{
	{
		shift_points.x = 0.0;
		shift_points.y = 0.0;
		shift_points.z = 0.0;
	}
	double total_weight = 0.0;
	for (int i = 0; i < src_points.size(); i++)
	{
		const MeanShift::Point3D& temp_point = src_points[i];
		double distance = euclidean_distance3D(search_points, temp_point);
		double weight = kernel_func(distance, kernal_bandwidth);
		{
			shift_points.x += temp_point.x*weight;
			shift_points.y += temp_point.y*weight;
			shift_points.z += temp_point.z*weight;
		}
		total_weight += weight;
	}
	const double total_weight_inv = 1.0 / total_weight;
	{
		shift_points.x *= total_weight_inv;
		shift_points.y *= total_weight_inv;
		shift_points.z *= total_weight_inv;
	}
}


std::vector<MeanShift::Point> MeanShift::meanshift(const std::vector<Point> &points,
                                             double kernel_bandwidth,
                                             double EPSILON){
    const double EPSILON_SQR = EPSILON*EPSILON;
    vector<bool> stop_moving(points.size(), false);
    vector<Point> shifted_points = points;
    double max_shift_distance;
    Point point_new;
#pragma omp parallel for;
    do {
        max_shift_distance = 0;
        for(int i=0; i<points.size(); i++){
            if (!stop_moving[i]) 
			{
                shift_point(shifted_points[i], points, kernel_bandwidth, point_new);
                double shift_distance_sqr = euclidean_distance_sqr(point_new, shifted_points[i]);
                if(shift_distance_sqr > max_shift_distance)
				{
                    max_shift_distance = shift_distance_sqr;
                }
                if(shift_distance_sqr <= EPSILON_SQR)
				{
                    stop_moving[i] = true;
                }
                shifted_points[i] = point_new;
            }
        }
        printf("max_shift_distance: %f\n", sqrt(max_shift_distance));
    } while (max_shift_distance > EPSILON_SQR);
    return shifted_points;
}

std::vector<pcl::PointXYZI> MeanShift::meanshift3D( const std::vector<pcl::PointXYZI>&src_points,
													double kernel_bandwidth,
													double EPSILON )
{
	const double EPSILON_SQR = EPSILON*EPSILON;
	std::vector<bool> stop_moving(src_points.size(), false);
	std::vector<MeanShift::Point3D> shifted_points = src_points;
	double max_shift_distance;
	MeanShift::Point3D point_new;

	do {
		max_shift_distance = 0;
#pragma omp parallel for;
		for (int i = 0; i < src_points.size(); i++) {
			if (!stop_moving[i])
			{
				shift_point(shifted_points[i], src_points, kernel_bandwidth, point_new);
				double shift_distance_sqr = euclidean_distance3D_sqr(point_new, shifted_points[i]);
				if (shift_distance_sqr > max_shift_distance)
				{
					max_shift_distance = shift_distance_sqr;
				}
				if (shift_distance_sqr <= EPSILON_SQR)
				{
					stop_moving[i] = true;
				}
				shifted_points[i] = point_new;
			}
		}
		printf("max_shift_distance: %f\n", sqrt(max_shift_distance));
	} while (max_shift_distance > EPSILON_SQR);
	return shifted_points;
}

vector<Cluster> MeanShift::cluster(const std::vector<Point> &points,
    const std::vector<Point> &shifted_points)
{
    vector<Cluster> clusters;
#pragma omp parallel for;
    for (int i = 0; i < shifted_points.size(); i++) {

        int c = 0;
        for (; c < clusters.size(); c++) {
            if (euclidean_distance(shifted_points[i], clusters[c].mode) <= CLUSTER_EPSILON) {
                break;
            }
        }

        if (c == clusters.size()) {
            Cluster clus;
            clus.mode = shifted_points[i];
            clusters.push_back(clus);
        }

        clusters[c].original_points.push_back(points[i]);
        clusters[c].shifted_points.push_back(shifted_points[i]);
    }

    return clusters;
}

std::vector<Cluster3D> MeanShift::cluster3D(const std::vector<pcl::PointXYZI> &src_points, const std::vector<pcl::PointXYZI> &shifted_points)
{
	vector<Cluster3D> clusters;
#pragma omp parallel for;
	for (int i = 0; i < shifted_points.size(); i++) {

		int c = 0;
		for (; c < clusters.size(); c++) {
			if (euclidean_distance3D(shifted_points[i], clusters[c].mode) <= CLUSTER_EPSILON) {
				break;
			}
		}

		if (c == clusters.size()) {
			Cluster3D clus;
			clus.mode = shifted_points[i];
			clusters.push_back(clus);
		}

		clusters[c].original_points.push_back(src_points[i]);
		clusters[c].shifted_points.push_back(shifted_points[i]);
	}

	return clusters;
}


std::vector<Cluster3D> MeanShift::cluster3D(const std::vector<pcl::PointXYZI>&src_points, double kernel_bandwidth)
{
	vector<Point3D> shifted_points = meanshift3D(src_points, kernel_bandwidth);
	return cluster3D(src_points, shifted_points);
}


vector<Cluster> MeanShift::cluster(const std::vector<Point> &points, double kernel_bandwidth){
    vector<Point> shifted_points = meanshift(points, kernel_bandwidth);
    return cluster(points, shifted_points);
}
