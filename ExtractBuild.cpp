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
	*@�����ڲ�����ĵ���ת��ΪPCL����ָ������
	*@ paramType    ->(std::vector<pcl::PointXYZ>  -> pcl::PointCloud<pcl::PointXYZ>::Ptr)
	*@ m_Cloud      ->inputCloud
	*/
	for (int i = 0; i < count; i++)
	{
		inputCloud->at(i).x = this->m_Cloud.at(i).x;
		inputCloud->at(i).y = this->m_Cloud.at(i).y;
		inputCloud->at(i).z = this->m_Cloud.at(i).z;
	}
	/*����kdTree��������*/
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
	//������ÿ��������Ϊ���ĵ�K��������ƽ�����꣨���ģ�
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
	
	
	//����ÿ���������Э�������
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
	*@�����ڲ�����ĵ���ת��ΪPCL����ָ������
	*@ paramType    ->(std::vector<pcl::PointXYZ>  -> pcl::PointCloud<pcl::PointXYZ>::Ptr)
	*@ m_Cloud      ->inputCloud
	*/
	for (int i = 0; i < count; i++)
	{
		inputCloud->at(i).x = this->m_Cloud.at(i).x;
		inputCloud->at(i).y = this->m_Cloud.at(i).y;
		inputCloud->at(i).z = this->m_Cloud.at(i).z;
	}
	/*����kdTree��������*/
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
	//������ÿ��������Ϊ���ĵ�K��������ƽ�����꣨���ģ�
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
	//����ÿ���������Э�������
	std::cout << "Computing Covariance Matrix of Every Search point..." << std::endl;
	std::vector<std::vector<Eigen::Matrix3d> >nPoint_CovarianceMatrix;
	
	if (inCloud.size() == meanPoint.size())
	{
		pcl::PointCloud<pcl::PointXYZ> searchDiffToMean;
		searchDiffToMean.resize(inCloud.size() * 1);
		//��ÿһ������������������������ĵ�Ĳ�ֵ
		for (size_t i = 0; i < inCloud.size(); i++)
		{
			pcl::PointXYZ temp;
			temp.x = inCloud[i].x - meanPoint[i].x;
			temp.y = inCloud[i].y - meanPoint[i].y;
			temp.z = inCloud[i].z - meanPoint[i].z;
			searchDiffToMean.push_back(temp);
		}
		//����Э�������
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

	std::cout << "####����þ��������ֵ����������####" <<std::endl;
	Eigen::EigenSolver<Eigen::Matrix3d> es(matrix);
	//es.compute(matrix,true);
	matrixVal      = es.eigenvalues().real();
	matrixVec      = es.eigenvectors().real();

	std::cout << "####��ȡ������С������ֵ####" << std::endl;
	Eigen::Matrix3d::Index eValMax,eValMin;
	matrixVal.rowwise().sum().maxCoeff(&eValMax);
	matrixVal.rowwise().sum().minCoeff(&eValMin);

	Eigen::Vector3d maxVec,minVec;
	maxVec << matrixVec.real()(0, eValMax), matrixVec.real()(1, eValMax), matrixVec.real()(2, eValMax);
	minVec << matrixVec.real()(0, eValMin), matrixVec.real()(1, eValMin), matrixVec.real()(2, eValMin);
	std::cout << "����ֵ:" << std::endl << es.eigenvalues() << std::endl << std::endl;
	std::cout << "��������:" << std::endl << es.eigenvectors() << std::endl << std::endl;
}
/*
* @ brief ��ȡ����ֵ������Сֵ������ֵ
* @ �ȼ��������ֵ�����ֵ����Сֵ������֮������ȡ�����С����ֵ��Ӧ����������
* @ param  ->(in)eigenvalue  ->���������ֵ
* @ param  ->(out)eValMax    ->������������ֵ��Ӧ������ֵ
* @ param  ->(out)eValMax    ->�������С����ֵ��Ӧ������ֵ
* @ return
*/
void Build::sortEigenVal(const Eigen::Matrix3d eigenvalue, Eigen::Matrix3d::Index &eValMax, Eigen::Matrix3d::Index&eValMin)
{
	std::cout << "####��ȡ������С������ֵ####" << std::endl;
	eigenvalue.rowwise().sum().maxCoeff(&eValMax);
	eigenvalue.rowwise().sum().minCoeff(&eValMin);
	
}
/*
 * @ brief ����������С������ֵ������ȡ��Ӧ����������
 * @ ����ȡ�����С����ֵ��Ӧ����������֮ǰӦ�ȼ��������ֵ�����ֵ����Сֵ
 * @ param  ->(in)matrixVec  ->������������� 
 * @ param  ->(out)maxVec    ->������������ֵ��Ӧ����������
 * @ param  ->(out)minVec    ->�������С����ֵ��Ӧ����������
 * @ param  ->(in)eValMax    ->������������ֵ��Ӧ������ֵ
 * @ param  ->(in)eValMin    ->�������С����ֵ��Ӧ������ֵ
 * @ return
*/
void Build::extEigenMaxMinVec(const Eigen::Matrix3d matrixVec,Eigen::Vector3d &maxVec, Eigen::Vector3d &minVec, const Eigen::Matrix3d::Index eValMax , const Eigen::Matrix3d::Index eValMin)
{
	std::cout << "####���������С����ֵ��ȡ��Ӧ����������####" << std::endl;
	maxVec << matrixVec.real()(0, eValMax), matrixVec.real()(1, eValMax), matrixVec.real()(2, eValMax);
	minVec << matrixVec.real()(0, eValMin), matrixVec.real()(1, eValMin), matrixVec.real()(2, eValMin);
}
/**
* @brief ��ʵ�Գƾ��������ֵ�������������ſ˱ȷ�
* �����Ÿ��(Jacobi)������ʵ�Գƾ������������ֵ����������
* @param pMatrix				����Ϊn*n�����顣���ʵ�Գƾ���
* @param nDim					����Ľ���
* @param pdblVects				����Ϊn*n�����飬������������(���д洢)
* @param dbEps					����Ҫ��
* @param nJt					���ͱ�������������������
* @param pdbEigenValues			����ֵ����
* @return                       0 success
* @Reference https://www.cnblogs.com/tlnshuju/p/6726000.html
*/
bool Build::JacbiCor(double * pMatrix, int nDim, double *pdblVects, double *pdbEigenValues, double dbEps, int nJt)
{
	//��ʼ����������pdblVects���� nDim * nDim
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

	int nCount = 0;		//��������
	while (1)
	{
		//��pMatrix�ķǶԽ������ҵ����Ԫ��
		double dbMax = pMatrix[1];
		int nRow = 0;
		int nCol = 1;
		for (int i = 0; i < nDim; i++)			//��
		{
			for (int j = 0; j < nDim; j++)		//��
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

		if (dbMax < dbEps)     //���ȷ���Ҫ�� 
			break;

		if (nCount > nJt)       //����������������
			break;

		nCount++;

		double dbApp = pMatrix[nRow*nDim + nRow];
		double dbApq = pMatrix[nRow*nDim + nCol];
		double dbAqq = pMatrix[nCol*nDim + nCol];

		//������ת�Ƕ�
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

		//������������
		for (int i = 0; i < nDim; i++)
		{
			int u = i*nDim + nRow;		//p   
			int w = i*nDim + nCol;		//q
			dbMax = pdblVects[u];
			pdblVects[u] = pdblVects[w] * dbSinTheta + dbMax*dbCosTheta;
			pdblVects[w] = pdblVects[w] * dbCosTheta - dbMax*dbSinTheta;
		}

	}

	//������ֵ���������Լ���һ��������������,����ֵ��pMatrix���Խ����ϵ�Ԫ��
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

		//����ֵ��һ������
		pdbEigenValues[j] = iter->first;
	}

	//�趨������
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
	std::cout << "####����ÿ���������ֵ####" << std::endl;
	std::vector<ExtBuild::RadiusPoint> radiuspoint;
	pcl::KdTreeFLANN<pcl::PointXYZINormal> rkdtree;
	rkdtree.setInputCloud(inputCloudPoint);
	double Radius = thredValue;							//*/
	std::vector<int> pointIdxRadiusSearch;////Ϊ��ȡ����ʱʹ�õ����ض��������������pcl_inputCloud����Kd������tree��
	std::vector<float> pointRadiusquaredDistance;//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	for (int i = 0; i < inputCloudPoint->size(); i++)//tree->setInputCloud(pcl_inputCloud);//���������������������ڴ洢ʵ�ʵĵ�����Ϣ
	{
		ExtBuild::RadiusPoint tempRadiuspoint;
		if (rkdtree.radiusSearch(inputCloudPoint->points[i], Radius, pointIdxRadiusSearch, pointRadiusquaredDistance) > 0)//ec.setMaxClusterSize(25000); //����һ��������Ҫ��������ĿΪ25000
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