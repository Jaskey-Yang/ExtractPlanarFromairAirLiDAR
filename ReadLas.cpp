#include "Readlas.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include<pcl/point_traits.h>
void DataIO::ReadlasXYZ(std::vector<pcl::PointXYZ> &Cloud)
{
	std::ifstream ifs;
	ifs.open(_FilePath, std::ios::in | std::ios::binary);
	if (!ifs.is_open())
	{
		std::cout << "File Error!" << std::endl;
		return;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);                                                //
	liblas::Header const& header = reader.GetHeader();
	std::cout << "Number of point records     : " << header.GetPointRecordsCount() << std::endl;          //记录的点数信息,下面的信息比较重要，都列出来了，具体意思也比较好理解
	std::cout << "File Signature (“LASF”)   : " << header.GetFileSignature() << std::endl;
	std::cout << std::setiosflags(std::ios::fixed);                                                   //设置显示小数点后9位小数。
	std::cout << std::setprecision(9) << "X scale factor              : " << header.GetScaleX() << std::endl;
	std::cout << "Y scale factor              : " << header.GetScaleY() << std::endl;
	std::cout << "Z scale factor              : " << header.GetScaleZ() << std::endl;
	std::cout << "X offset                    : " << header.GetOffsetX() << std::endl;
	std::cout << "Y offset                    : " << header.GetOffsetY() << std::endl;
	std::cout << "Z offset                    : " << header.GetOffsetZ() << std::endl;
	std::cout << "Max X                       : " << header.GetMaxX() << std::endl;
	std::cout << "Max Y                       : " << header.GetMaxY() << std::endl;
	std::cout << "Max Z                       : " << header.GetMaxZ() << std::endl;
	std::cout << "Min X                       : " << header.GetMinX() << std::endl;
	std::cout << "Min Y                       : " << header.GetMinY() << std::endl;
	std::cout << "Min Z                       : " << header.GetMinZ() << std::endl;
	//std::ofstream ofile;
	
	while (reader.ReadNextPoint())                                                                  //每个1000万对数据进行一次保存
	{
		pcl::PointXYZ temp;
		liblas::Point const& p = reader.GetPoint();
		//ofile << std::setiosflags(std::ios::fixed);
		//ofile << std::setprecision(8) << p.GetX() << " " << p.GetY() << " " << p.GetZ() << " " << p.GetIntensity() << " "
		//	<< p.GetReturnNumber() << " " << p.GetNumberOfReturns() << " " << p.GetClassification() << " "
		//	<< p.GetScanDirection() << " " << p.GetFlightLineEdge() << " " << p.GetScanAngleRank() << " "
		//	<< p.GetPointSourceID() << " " << p.GetTime() << std::endl;
		temp.x = p.GetX();
		temp.y = p.GetY();
		temp.z = p.GetZ();
		Cloud.push_back(temp);
	}
}

void DataIO::ReadlasXYZI(std::vector<pcl::PointXYZI>& Cloud)
{
	std::ifstream ifs;
	ifs.open(_FilePath, std::ios::in | std::ios::binary);
	if (!ifs.is_open())
	{
		std::cout << "读取点云文件错误！！！" << std::endl;
		return;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);                                                //
	liblas::Header const& header = reader.GetHeader();
	std::cout << "Number of point records     : " << header.GetPointRecordsCount() << std::endl;          //记录的点数信息,下面的信息比较重要，都列出来了，具体意思也比较好理解
	std::cout << "File Signature (“LASF”)   : " << header.GetFileSignature() << std::endl;
	std::cout << std::setiosflags(std::ios::fixed);                                                   //设置显示小数点后9位小数。
	std::cout << std::setprecision(9) << "X scale factor              : " << header.GetScaleX() << std::endl;
	std::cout << "Y scale factor              : " << header.GetScaleY() << std::endl;
	std::cout << "Z scale factor              : " << header.GetScaleZ() << std::endl;
	std::cout << "X offset                    : " << header.GetOffsetX() << std::endl;
	std::cout << "Y offset                    : " << header.GetOffsetY() << std::endl;
	std::cout << "Z offset                    : " << header.GetOffsetZ() << std::endl;
	std::cout << "Max X                       : " << header.GetMaxX() << std::endl;
	std::cout << "Max Y                       : " << header.GetMaxY() << std::endl;
	std::cout << "Max Z                       : " << header.GetMaxZ() << std::endl;
	std::cout << "Min X                       : " << header.GetMinX() << std::endl;
	std::cout << "Min Y                       : " << header.GetMinY() << std::endl;
	std::cout << "Min Z                       : " << header.GetMinZ() << std::endl;
	//std::ofstream ofile;

	while (reader.ReadNextPoint())                                                                  //每个1000万对数据进行一次保存
	{
		pcl::PointXYZI temp;
		liblas::Point const& p = reader.GetPoint();
		//ofile << std::setiosflags(std::ios::fixed);
		//ofile << std::setprecision(8) << p.GetX() << " " << p.GetY() << " " << p.GetZ() << " " << p.GetIntensity() << " "
		//	<< p.GetReturnNumber() << " " << p.GetNumberOfReturns() << " " << p.GetClassification() << " "
		//	<< p.GetScanDirection() << " " << p.GetFlightLineEdge() << " " << p.GetScanAngleRank() << " "
		//	<< p.GetPointSourceID() << " " << p.GetTime() << std::endl;
		temp.x = p.GetX();
		temp.y = p.GetY();
		temp.z = p.GetZ();
		temp.intensity = p.GetIntensity();
		Cloud.push_back(temp);
	}
	std::cout << "读取的点云的点云的数量为： "<<Cloud.size() << std::endl;
}
void DataIO::ReadLasPCLXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud)
{
	std::ifstream ifs;
	ifs.open(_FilePath, std::ios::in | std::ios::binary);
	if (!ifs.is_open())
	{
		std::cout << "File Error!" << std::endl;
		return;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);                                                //
	liblas::Header const& header = reader.GetHeader();
	std::cout << "Number of point records     : " << header.GetPointRecordsCount() << std::endl;          //记录的点数信息,下面的信息比较重要，都列出来了，具体意思也比较好理解
	std::cout << "File Signature (“LASF”)   : " << header.GetFileSignature() << std::endl;
	std::cout << std::setiosflags(std::ios::fixed);                                                   //设置显示小数点后9位小数。
	std::cout << std::setprecision(9) << "X scale factor              : " << header.GetScaleX() << std::endl;
	std::cout << "Y scale factor              : " << header.GetScaleY() << std::endl;
	std::cout << "Z scale factor              : " << header.GetScaleZ() << std::endl;
	std::cout << "X offset                    : " << header.GetOffsetX() << std::endl;
	std::cout << "Y offset                    : " << header.GetOffsetY() << std::endl;
	std::cout << "Z offset                    : " << header.GetOffsetZ() << std::endl;
	std::cout << "Max X                       : " << header.GetMaxX() << std::endl;
	std::cout << "Max Y                       : " << header.GetMaxY() << std::endl;
	std::cout << "Max Z                       : " << header.GetMaxZ() << std::endl;
	std::cout << "Min X                       : " << header.GetMinX() << std::endl;
	std::cout << "Min Y                       : " << header.GetMinY() << std::endl;
	std::cout << "Min Z                       : " << header.GetMinZ() << std::endl;

	Cloud->width = header.GetPointRecordsCount();
	Cloud->height = 1;
	Cloud->resize(Cloud->width*Cloud->height);
	//std::ofstream ofile;
	int i = 0;
	while (reader.ReadNextPoint())                                                                  //每个1000万对数据进行一次保存
	{
		//pcl::PointXYZ temp;
		liblas::Point const& p = reader.GetPoint();
		//ofile << std::setiosflags(std::ios::fixed);
		//ofile << std::setprecision(8) << p.GetX() << " " << p.GetY() << " " << p.GetZ() << " " << p.GetIntensity() << " "
		//	<< p.GetReturnNumber() << " " << p.GetNumberOfReturns() << " " << p.GetClassification() << " "
		//	<< p.GetScanDirection() << " " << p.GetFlightLineEdge() << " " << p.GetScanAngleRank() << " "
		//	<< p.GetPointSourceID() << " " << p.GetTime() << std::endl;
		Cloud->at(i).x = p.GetX();
		Cloud->at(i).y = p.GetY();
		Cloud->at(i).z = p.GetZ();
		i++;
		//temp.intensity = p.GetIntensity();
	}



}
	
/*点云写入las格式*/
void DataIO::WriteXYZItoLas(std::string outFile, const std::vector<pcl::PointXYZI> Cloud)
{


}

