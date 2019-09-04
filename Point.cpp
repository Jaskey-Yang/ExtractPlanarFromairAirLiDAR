#include"Point.h"
void ExtBuild::PointCloud::computeBuildingBox(Point& bbMin, Point& bbMax)
{
	if (empty())
	{
		bbMin = bbMax = Point();
		return;
	}
	bbMin = bbMax = at(0);
	for (std::size_t i = 1; i < size(); i++)
	{
		const ExtBuild::Point& p = at(i);
		for (int d = 0; d < 3; ++d)
		{
			if (p.u[d] < bbMin.u[d])
			{
				bbMin.u[d] = p.u[d];
			}
			else if(p.u[d] > bbMax.u[d])
			{
				bbMax.u[d] = p.u[d];
			}
		}
	}
}
bool ExtBuild::PointCloud::computeNorOfTwoVector(const pcl::PointXYZINormal &pointA, const pcl::PointXYZINormal& pointB)
{
	bool flag = true;
	double norA = sqrt(pow(pointA.normal_x, 2) + pow(pointA.normal_y, 2) + pow(pointA.normal_z, 2));
	double norB = sqrt(pow(pointB.normal_x, 2) + pow(pointB.normal_y, 2) + pow(pointB.normal_z, 2));
	double nCross = pointA.normal_x*pointB.normal_x + pointA.normal_y*pointB.normal_y + pointA.normal_z * pointB.normal_z;
	double corner = (double)(nCross / (norA*norB));


}
