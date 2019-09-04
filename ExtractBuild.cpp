#include"ExtractBuild.h"


void Build::extractBuild(std::vector<pcl::PointXYZ> &outCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	int count = this->m_Cloud.size();
	inputCloud->width = count;
	inputCloud->height = 1;
	inputCloud->resize(inputCloud->width*inputCloud->height);
	//std::vector<std::vector<pcl::PointXYZ> >nKNNPoints;
	//Build::computeNeiborPoints(outCloud, nKNNPoints);

	/*
	*@将类内部定义的点云转换为PCL共享指针类型
	*@ paramType    ->(std::vector<pcl::PointXYZ>  -> pcl::PointCloud<pcl::PointXYZ>::Ptr)
	*@ m_Cloud      ->inputCloud
	*/
	for (int i = 0; i < count; i++)
	{
		inputCloud->at(i).x = this->m_Cloud.at(i).x;
		inputCloud->at(i).y = this->m_Cloud.at(i).y;
		inputCloud->at(i).z = this->m_Cloud.at(i).z;
	}
	/*建立kdTree搜索对象*/
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(inputCloud);
	int K = 20;
	std::vector<std::vector<pcl::PointXYZ> > nKNNPoint;
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	for (int i = 0; i < inputCloud->size(); i++)
	{
		std::cout << "K nearest neighbor search at (" << inputCloud->points[i].x
			<< " " << inputCloud->points[i].y
			<< " " << inputCloud->points[i].z
			<< ") with K=" << K << std::endl;
		if (kdtree.nearestKSearch(inputCloud->points[i], K, pointIdxKNNSearch, pointNKNSquaredDistance) > 0)
		{
			for (size_t j = 0; j < pointIdxKNNSearch.size(); ++j)
			{
				std::cout << "    " << inputCloud->points[pointIdxKNNSearch[j]].x
							<< " " << inputCloud->points[pointIdxKNNSearch[j]].y
							<< " " << inputCloud->points[pointIdxKNNSearch[j]].z
							<< " (squared distance: " << pointNKNSquaredDistance[j] << ")" << std::endl;
				nKNNPoint[i][j] = inputCloud->points[pointIdxKNNSearch[j]];
			}
		}
	}

	std::vector<pcl::PointXYZ> meanKNNPoint;
	size_t row = nKNNPoint.size();
	size_t col = nKNNPoint[0].size();


	
	std::cout << "Computing points' mass-point..." << std::endl;
	double mean_X, mean_Y, mean_Z;
	//计算以每个搜索点为中心的K的邻域点的平均坐标（质心）
	for (size_t i = 0; i < nKNNPoint.size(); i++)
	{
		pcl::PointXYZ tempMeanPoint;
		double total_X, total_Y, total_Z;
		for (size_t j = 0; j <nKNNPoint[i].size(); j++)
		{
			total_X += nKNNPoint[i][j].x;
			total_Y += nKNNPoint[i][j].y;
			total_Z += nKNNPoint[i][j].z;
		}
		tempMeanPoint.x = (double)(total_X / nKNNPoint[i].size());
		tempMeanPoint.y = (double)(total_Y / nKNNPoint[i].size());
		tempMeanPoint.z = (double)(total_Z / nKNNPoint[i].size());
		meanKNNPoint.push_back(tempMeanPoint);
	}
	
	
	//计算每个搜索点的协方差矩阵
	std::cout << "Computing Covariance Matrix of Every Search point..." << std::endl;
	std::vector<std::vector<Eigen::Matrix3d> >nPoint_CovarianceMatrix;
	

	for (size_t i = 0; i < nKNNPoint.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> temp;//(new pcl::PointCloud<pcl::PointXYZ>);
		temp.resize(nKNNPoint[i].size() * 1);
		for (size_t j = 0; j < nKNNPoint[i].size(); j++)
		{
			temp.points[j].x = nKNNPoint[i][j].x;
			temp.points[j].y = nKNNPoint[i][j].y;
			temp.points[j].z = nKNNPoint[i][j].z;
		}
		// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
		Eigen::Vector4d xyz_centroid;
		xyz_centroid<<meanKNNPoint[i].x, meanKNNPoint[i].y, meanKNNPoint[i].z;
		/*xyz_centroid.y = meanKNNPoint[i].y;
		xyz_centroid.z = meanKNNPoint[i].z;*/
		// Placeholder for the 3x3 covariance matrix at each surface patch
		Eigen::Matrix3d covariance_matrix;
		// Compute the 3x3 covariance matrix
		std::cout << "Start to computing CovarianceMatrix..." << std::endl;
		unsigned int nvalid;
		nvalid=pcl::computeCovarianceMatrix(temp, xyz_centroid, covariance_matrix);
		if (nvalid != 0)
		{
			std::cout << "computeCovarianceMatrix return Valid is " << nvalid << std::endl;
			nPoint_CovarianceMatrix[i].push_back(covariance_matrix);
		}
	}
	std::cout << "Computing Covariance Matrix of Every Search point is finished..." << std::endl;
}

void Build::computeNeiborPoints(std::vector<pcl::PointXYZ> inCloud, std::vector<std::vector<pcl::PointXYZ> > &nKNNPoint)
{
	this->m_Cloud = inCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	int count = this->m_Cloud.size();
	inputCloud->width = count;
	inputCloud->height = 1;
	inputCloud->resize(inputCloud->width*inputCloud->height);
	/*
	*@将类内部定义的点云转换为PCL共享指针类型
	*@ paramType    ->(std::vector<pcl::PointXYZ>  -> pcl::PointCloud<pcl::PointXYZ>::Ptr)
	*@ m_Cloud      ->inputCloud
	*/
	for (int i = 0; i < count; i++)
	{
		inputCloud->at(i).x = this->m_Cloud.at(i).x;
		inputCloud->at(i).y = this->m_Cloud.at(i).y;
		inputCloud->at(i).z = this->m_Cloud.at(i).z;
	}
	/*建立kdTree搜索对象*/
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(inputCloud);
	int K = 20;
	//std::vector<std::vector<pcl::PointXYZ> > nKNNPoint;
	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	for (int i = 0; i < inputCloud->size(); i++)
	{
		std::cout << "K nearest neighbor search at (" << inputCloud->points[i].x
			<< " " << inputCloud->points[i].y
			<< " " << inputCloud->points[i].z
			<< ") with K=" << K << std::endl;
		if (kdtree.nearestKSearch(inputCloud->points[i], K, pointIdxKNNSearch, pointNKNSquaredDistance) > 0)
		{
			for (size_t j = 0; j < pointIdxKNNSearch.size(); ++j)
			{
				std::cout << "    " << inputCloud->points[pointIdxKNNSearch[j]].x
					<< " " << inputCloud->points[pointIdxKNNSearch[j]].y
					<< " " << inputCloud->points[pointIdxKNNSearch[j]].z
					<< " (squared distance: " << pointNKNSquaredDistance[j] << ")" << std::endl;
				nKNNPoint[i][j] = inputCloud->points[pointIdxKNNSearch[j]];
			}
		}
	}

}
void Build::computeMeanPoint(std::vector<std::vector<pcl::PointXYZ> > nKNNPoint,std::vector<pcl::PointXYZ> &meanKNNPoint)
{
	//std::vector<std::vector<pcl::PointXYZ> >meanKNNPoint;
	size_t row = nKNNPoint.size();
	size_t col = nKNNPoint[0].size();

	std::cout << "Computing points' mass-point..." << std::endl;
	double mean_X, mean_Y, mean_Z;
	//计算以每个搜索点为中心的K的邻域点的平均坐标（质心）
	for (size_t i = 0; i < nKNNPoint.size(); i++)
	{
		pcl::PointXYZ tempMeanPoint;
		double total_X, total_Y, total_Z;
		for (size_t j = 0; j < nKNNPoint[i].size(); j++)
		{
			total_X += nKNNPoint[i][j].x;
			total_Y += nKNNPoint[i][j].y;
			total_Z += nKNNPoint[i][j].z;
		}
		tempMeanPoint.x = (double)(total_X / nKNNPoint[i].size());
		tempMeanPoint.y = (double)(total_Y / nKNNPoint[i].size());
		tempMeanPoint.z = (double)(total_Z / nKNNPoint[i].size());
		meanKNNPoint.push_back(tempMeanPoint);
	}
}

void Build::computeCorvianceMatrix(std::vector<pcl::PointXYZ> inCloud,std::vector<pcl::PointXYZ> meanPoint, std::vector<Build::Matrix> & pointCloudMatrix)
{
	//计算每个搜索点的协方差矩阵
	std::cout << "Computing Covariance Matrix of Every Search point..." << std::endl;
	std::vector<std::vector<Eigen::Matrix3d> >nPoint_CovarianceMatrix;
	
	if (inCloud.size() == meanPoint.size())
	{
		pcl::PointCloud<pcl::PointXYZ> searchDiffToMean;
		searchDiffToMean.resize(inCloud.size() * 1);
		//求每一个搜索点与领域内所求的中心点的差值
		for (size_t i = 0; i < inCloud.size(); i++)
		{
			pcl::PointXYZ temp;
			temp.x = inCloud[i].x - meanPoint[i].x;
			temp.y = inCloud[i].y - meanPoint[i].y;
			temp.z = inCloud[i].z - meanPoint[i].z;
			searchDiffToMean.push_back(temp);
		}
		//计算协方差矩阵
		for (size_t i = 0; i < searchDiffToMean.size(); i++)
		{
			Build::Matrix tempMatrix;
			tempMatrix.i = i;
			tempMatrix.point = inCloud[i];
			tempMatrix.M11 = pow(searchDiffToMean[i].x, 2);
			tempMatrix.M12 = searchDiffToMean[i].x * searchDiffToMean[i].y;
			tempMatrix.M13 = searchDiffToMean[i].x * searchDiffToMean[i].z;
			tempMatrix.M21 = searchDiffToMean[i].y * searchDiffToMean[i].x;
			tempMatrix.M22 = pow(searchDiffToMean[i].y,2);
			tempMatrix.M23 = searchDiffToMean[i].y * searchDiffToMean[i].z;
			tempMatrix.M31 = searchDiffToMean[i].z * searchDiffToMean[i].x;
			tempMatrix.M32 = searchDiffToMean[i].z * searchDiffToMean[i].y;
			tempMatrix.M33 = pow(searchDiffToMean[i].z, 2);
			pointCloudMatrix.push_back(tempMatrix);
		}
	}
	else
	{
		std::cout << "error:The input point-clouds not equal to the size of meanPoint of search-point..." << std::endl;
		return;
	}
	
	//for (size_t i = 0; i < inCloud.size(); i++)
	//{
	//	
	//	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	//	Eigen::Vector4d xyz_centroid;
	//	xyz_centroid.x = meanPoint[i].x;
	//	xyz_centroid.y = meanPoint[i].y;
	//	xyz_centroid.z = meanPoint[i].z;
	//	// Placeholder for the 3x3 covariance matrix at each surface patch
	//	Eigen::Matrix3d covariance_matrix;
	//	// Compute the 3x3 covariance matrix
	//	std::cout << "Start to computing CovarianceMatrix..." << std::endl;
	//	unsigned int nvalid;
	//	nvalid = pcl::computeCovarianceMatrix(temp, xyz_centroid, covariance_matrix);
	//	if (nvalid != 0)
	//	{
	//		std::cout << "computeCovarianceMatrix return Valid is " << nvalid << std::endl;
	//		nPoint_CovarianceMatrix[i].push_back(covariance_matrix);
	//	}
	//}
	std::cout << "Computing Covariance Matrix of Every Search point is finished..." << std::endl;
}

void Build::computeEigenVectorAndValue(const Eigen::Matrix3d matrix,  Eigen::Vector3d &matrixVal, Eigen::Matrix3d &matrixVec)
{
	std::cout << "Here is a n*n Matrix...Matrix: " << std::endl;
	std::cout << matrix << std::endl;

	std::cout << "####计算该矩阵的特征值和特征向量####" <<std::endl;
	Eigen::EigenSolver<Eigen::Matrix3d> es(matrix);
	//es.compute(matrix,true);
	matrixVal      = es.eigenvalues().real();
	matrixVec      = es.eigenvectors().real();

	std::cout << "####提取最大和最小的特征值####" << std::endl;
	Eigen::Matrix3d::Index eValMax,eValMin;
	matrixVal.rowwise().sum().maxCoeff(&eValMax);
	matrixVal.rowwise().sum().minCoeff(&eValMin);

	Eigen::Vector3d maxVec,minVec;
	maxVec << matrixVec.real()(0, eValMax), matrixVec.real()(1, eValMax), matrixVec.real()(2, eValMax);
	minVec << matrixVec.real()(0, eValMin), matrixVec.real()(1, eValMin), matrixVec.real()(2, eValMin);
	std::cout << "特征值:" << std::endl << es.eigenvalues() << std::endl << std::endl;
	std::cout << "特征向量:" << std::endl << es.eigenvectors() << std::endl << std::endl;
}
/*
* @ brief 提取特征值最大和最小值的索引值
* @ 先计算出特征值的最大值和最小值的索引之后，在提取最大最小特征值对应的特征向量
* @ param  ->(in)eigenvalue  ->输入的特征值
* @ param  ->(out)eValMax    ->输出的最大特征值对应的索引值
* @ param  ->(out)eValMax    ->输出的最小特征值对应的索引值
* @ return
*/
void Build::sortEigenVal(const Eigen::Matrix3d eigenvalue, Eigen::Matrix3d::Index &eValMax, Eigen::Matrix3d::Index&eValMin)
{
	std::cout << "####提取最大和最小的特征值####" << std::endl;
	eigenvalue.rowwise().sum().maxCoeff(&eValMax);
	eigenvalue.rowwise().sum().minCoeff(&eValMin);
	
}
/*
 * @ brief 根据最大和最小的特征值索引提取对应的特征向量
 * @ 在提取最大最小特征值对应的特征向量之前应先计算出特征值的最大值和最小值
 * @ param  ->(in)matrixVec  ->输入的特征向量 
 * @ param  ->(out)maxVec    ->输出的最大特征值对应的特征向量
 * @ param  ->(out)minVec    ->输出的最小特征值对应的特征向量
 * @ param  ->(in)eValMax    ->输入的最大特征值对应的索引值
 * @ param  ->(in)eValMin    ->输入的最小特征值对应的索引值
 * @ return
*/
void Build::extEigenMaxMinVec(const Eigen::Matrix3d matrixVec,Eigen::Vector3d &maxVec, Eigen::Vector3d &minVec, const Eigen::Matrix3d::Index eValMax , const Eigen::Matrix3d::Index eValMin)
{
	std::cout << "####根据最大最小特征值提取对应的特征向量####" << std::endl;
	maxVec << matrixVec.real()(0, eValMax), matrixVec.real()(1, eValMax), matrixVec.real()(2, eValMax);
	minVec << matrixVec.real()(0, eValMin), matrixVec.real()(1, eValMin), matrixVec.real()(2, eValMin);
}
/**
* @brief 求实对称矩阵的特征值及特征向量的雅克比法
* 利用雅格比(Jacobi)方法求实对称矩阵的所有特征值及特征向量
* @param pMatrix				长度为n*n的数组。存放实对称矩阵
* @param nDim					矩阵的阶数
* @param pdblVects				长度为n*n的数组，返回特征向量(按列存储)
* @param dbEps					精度要求
* @param nJt					整型变量。控制最大迭代次数
* @param pdbEigenValues			特征值数组
* @return                       0 success
* @Reference https://www.cnblogs.com/tlnshuju/p/6726000.html
*/
bool Build::JacbiCor(double * pMatrix, int nDim, double *pdblVects, double *pdbEigenValues, double dbEps, int nJt)
{
	//初始化特征向量pdblVects矩阵 nDim * nDim
	//[1  0  0...0
	// 0  1  0...0
    // 0  0  1...0
	//   ...  ...
    // 0  0  0...1]
	for (int i = 0; i < nDim; i++)
	{
		pdblVects[i*nDim + i] = 1.0f;
		for (int j = 0; j < nDim; j++)
		{
			if (i != j)
				pdblVects[i*nDim + j] = 0.0f;
		}
	}

	int nCount = 0;		//迭代次数
	while (1)
	{
		//在pMatrix的非对角线上找到最大元素
		double dbMax = pMatrix[1];
		int nRow = 0;
		int nCol = 1;
		for (int i = 0; i < nDim; i++)			//行
		{
			for (int j = 0; j < nDim; j++)		//列
			{
				double d = fabs(pMatrix[i*nDim + j]);

				if ((i != j) && (d > dbMax))
				{
					dbMax = d;
					nRow = i;
					nCol = j;
				}
			}
		}

		if (dbMax < dbEps)     //精度符合要求 
			break;

		if (nCount > nJt)       //迭代次数超过限制
			break;

		nCount++;

		double dbApp = pMatrix[nRow*nDim + nRow];
		double dbApq = pMatrix[nRow*nDim + nCol];
		double dbAqq = pMatrix[nCol*nDim + nCol];

		//计算旋转角度
		double dbAngle = 0.5*atan2(-2 * dbApq, dbAqq - dbApp);
		double dbSinTheta = sin(dbAngle);
		double dbCosTheta = cos(dbAngle);
		double dbSin2Theta = sin(2 * dbAngle);
		double dbCos2Theta = cos(2 * dbAngle);

		pMatrix[nRow*nDim + nRow] = dbApp*dbCosTheta*dbCosTheta +
									dbAqq*dbSinTheta*dbSinTheta + 2 * dbApq*dbCosTheta*dbSinTheta;
		pMatrix[nCol*nDim + nCol] = dbApp*dbSinTheta*dbSinTheta +
									dbAqq*dbCosTheta*dbCosTheta - 2 * dbApq*dbCosTheta*dbSinTheta;
		pMatrix[nRow*nDim + nCol] = 0.5*(dbAqq - dbApp)*dbSin2Theta + dbApq*dbCos2Theta;
		pMatrix[nCol*nDim + nRow] = pMatrix[nRow*nDim + nCol];

		for (int i = 0; i < nDim; i++)
		{
			if ((i != nCol) && (i != nRow))
			{
				int u = i*nDim + nRow;	//p  
				int w = i*nDim + nCol;	//q
				dbMax = pMatrix[u];
				pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax*dbCosTheta;
				pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax*dbSinTheta;
			}
		}

		for (int j = 0; j < nDim; j++)
		{
			if ((j != nCol) && (j != nRow))
			{
				int u = nRow*nDim + j;	//p
				int w = nCol*nDim + j;	//q
				dbMax = pMatrix[u];
				pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax*dbCosTheta;
				pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax*dbSinTheta;
			}
		}

		//计算特征向量
		for (int i = 0; i < nDim; i++)
		{
			int u = i*nDim + nRow;		//p   
			int w = i*nDim + nCol;		//q
			dbMax = pdblVects[u];
			pdblVects[u] = pdblVects[w] * dbSinTheta + dbMax*dbCosTheta;
			pdblVects[w] = pdblVects[w] * dbCosTheta - dbMax*dbSinTheta;
		}

	}

	//对特征值进行排序以及又一次排列特征向量,特征值即pMatrix主对角线上的元素
	std::map<double, int> mapEigen;
	for (int i = 0; i < nDim; i++)
	{
		pdbEigenValues[i] = pMatrix[i*nDim + i];

		mapEigen.insert(std::pair<double,int>(pdbEigenValues[i], i));
	}

	double *pdbTmpVec = new double[nDim*nDim];
	std::map<double, int>::reverse_iterator iter = mapEigen.rbegin();
	for (int j = 0; iter != mapEigen.rend(), j < nDim; ++iter, ++j)
	{
		for (int i = 0; i < nDim; i++)
		{
			pdbTmpVec[i*nDim + j] = pdblVects[i*nDim + iter->second];
		}

		//特征值又一次排列
		pdbEigenValues[j] = iter->first;
	}

	//设定正负号
	for (int i = 0; i < nDim; i++)
	{
		double dSumVec = 0;
		for (int j = 0; j < nDim; j++)
			dSumVec += pdbTmpVec[j * nDim + i];
		if (dSumVec < 0)
		{
			for (int j = 0; j < nDim; j++)
				pdbTmpVec[j * nDim + i] *= -1;
		}
	}

	memcpy(pdblVects, pdbTmpVec, sizeof(double)*nDim*nDim);
	delete[]pdbTmpVec;

	return 0;
}

std::vector<ExtBuild::RadiusPoint> Build::computeNeiborPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr inputCloudPoint,  double thredValue)
{
	std::cout << "####计算每个点的邻域值####" << std::endl;
	std::vector<ExtBuild::RadiusPoint> radiuspoint;
	pcl::KdTreeFLANN<pcl::PointXYZINormal> rkdtree;
	rkdtree.setInputCloud(inputCloudPoint);
	double Radius = thredValue;							//*/
	std::vector<int> pointIdxRadiusSearch;////为提取点云时使用的搜素对象利用输入点云pcl_inputCloud创建Kd树对象tree。
	std::vector<float> pointRadiusquaredDistance;//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	for (int i = 0; i < inputCloudPoint->size(); i++)//tree->setInputCloud(pcl_inputCloud);//创建点云索引向量，用于存储实际的点云信息
	{
		ExtBuild::RadiusPoint tempRadiuspoint;
		if (rkdtree.radiusSearch(inputCloudPoint->points[i], Radius, pointIdxRadiusSearch, pointRadiusquaredDistance) > 0)//ec.setMaxClusterSize(25000); //设置一个聚类需要的最大点数目为25000
		{
		
			tempRadiuspoint.pointID = i;
			tempRadiuspoint.throedDist = Radius;
			{
				tempRadiuspoint.point.x = inputCloudPoint->points[i].x;
				tempRadiuspoint.point.y = inputCloudPoint->points[i].x;
				tempRadiuspoint.point.z = inputCloudPoint->points[i].x;

				tempRadiuspoint.point.normal_x = inputCloudPoint->points[i].normal_x;
				tempRadiuspoint.point.normal_y = inputCloudPoint->points[i].normal_y;
				tempRadiuspoint.point.normal_z = inputCloudPoint->points[i].normal_z;

			}
			tempRadiuspoint.pointIdxRadiusSearch = pointIdxRadiusSearch;
			tempRadiuspoint.pointRadiusquaredDistance = pointRadiusquaredDistance;
			for (int j = 1; j < pointIdxRadiusSearch.size(); j++)
			{
				tempRadiuspoint.knnPoints.push_back(inputCloudPoint->at(pointIdxRadiusSearch[j]));
			}
		}
		radiuspoint.push_back(tempRadiuspoint);

	}
	return radiuspoint;
}