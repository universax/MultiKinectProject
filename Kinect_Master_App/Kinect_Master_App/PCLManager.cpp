#include "stdafx.h"
#include "BoostUdpServer.h"
#include "PCLManager.h"



//pcl::visualization::CloudViewer clusterViewer("Master: Clusters");
//pcl::visualization::CloudViewer cHullViewer("Master: Concave Hull");

PCLManager::PCLManager()
{
	pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
	margedCloud = p;
}

void PCLManager::update(std::vector<pcl::PointCloud<PointType>::Ptr> &outputPoints)
{
	////-----------------------------------
	////�e�L�l�N�g�f�[�^�̈ʒu���킹
	////-----------------------------------
	//int mainKinectID = 0;
	////Kinect 001
	//ICPTransformMatrix[0] <<
	//	1, 0, 0, 0,
	//	0, 1, 0, 0,
	//	0, 0, 1, 0,
	//	0, 0, 0, 1;
	////Kinect 002
	//ICPTransformMatrix[1] <<
	//	1, 0, 0, 0,
	//	0, 1, 0, 0,
	//	0, 0, 1, 0,
	//	0, 0, 0, 1;


	//boost::thread_group registrationThreads;
	//vector<pcl::PointCloud<PointType>::Ptr> registrationClouds;
	////for (int i = 0; i < outputPoints.size(); i++)
	////{
	////	statisticalOutlierFilter(outputPoints[i]);
	////}

	//for (int i = 0; i < outputPoints.size(); i++)
	//{
	//	pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
	//	registrationClouds.push_back(p);
	//	registrationThreads.create_thread(boost::bind(&PCLManager::registrationAll, this, outputPoints[i], registrationClouds[i], ICPTransformMatrix[i]));

	//	////�ʒu������Ȃ��Ȃ��ė�����A����ōēx���W�X�g���[�V��������	
	//	//transformationVector[i] = iterativeClosestPoint(outputPoints[mainKinectID], outputPoints[i]);
	//	//cout << transformationVector[i] << endl;
	//	//pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
	//	//registrationClouds.push_back(p);
	//	//registrationThreads.create_thread(boost::bind(&PCLManager::registrationAll, this, outputPoints[i], registrationClouds[i], transformationVector[i]));
	//}
	//registrationThreads.join_all();


	//-----------------------------------
	//�}�[�W
	//-----------------------------------
	pcl::PointCloud<PointType>::Ptr calcPoints(new pcl::PointCloud<PointType>());
	for (int i = 0; i < outputPoints.size(); i++)
	{
		//pcl::PointIndices::Ptr indices = getPlaneIndices(outputPoints[i]);
		//extractIndices(outputPoints[i], indices);

		*calcPoints += *outputPoints[i];
		//registrationClouds[i].reset();
	}
	//registrationClouds.clear();

	//Filter
	float voxelVal = 0.02f;
	voxelGridFilter(voxelVal, calcPoints);
	statisticalOutlierFilter(calcPoints);
	
	//Copy
	margedCloud->clear();
	*margedCloud = *calcPoints;

	//-----------------------------------
	//�N���X�^����
	//-----------------------------------
	//�|�C���g�N���E�h���N���X�^�ɕ���
	voxelGridFilter(0.06f, calcPoints);
	euclideanClusterExtraction(calcPoints, eachClouds);
	calcPoints.reset();

	//�d�S�v�Z
	centroids.clear();
	for (int i = 0; i < eachClouds.size(); i++)
	{
		//�N���X�^�̏d�S���v�Z
		Eigen::Vector4f center = centroid(eachClouds[i]);
		centroids.push_back(center);
	}

	////�������ꂽ�N���X�^�ɐF��t���ĕ\��
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	//vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> coloredClouds;
	//for (int i = 0; i < eachClouds.size(); i++)
	//{
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	pcl::copyPointCloud(*eachClouds[i], *cc);
	//	coloredClouds.push_back(cc);
	//	double red = 255 * (i % 2 * 3 / (double)eachClouds.size());
	//	double green = 255 - 255 * ((i % 3) * 3 / (double)eachClouds.size());
	//	double blue = 255 - 255 * ((i % 5) * 3 / (double)eachClouds.size());
	//	for (int j = 0; j < coloredClouds[i]->points.size(); j++)
	//	{
	//		coloredClouds[i]->points[j].r = red;
	//		coloredClouds[i]->points[j].g = green;
	//		coloredClouds[i]->points[j].b = blue;
	//	}
	//	*clusteredColorCloud += *coloredClouds[i];
	//}
	//cout << "Cluster: " << eachClouds.size() << endl;
	//clusterViewer.showCloud(clusteredColorCloud);

	std::cout << "----------------------------------\n" << std::endl;
}

void PCLManager::registrationAll(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputClouds, Eigen::Matrix4f &matrix) {
	if (inputCloud->empty())
	{
		return;
	}

	pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*inputCloud, *p, matrix);
	*inputCloud = *p;
	p.reset();
	*outputClouds += *inputCloud;
}

void PCLManager::createPolygon(vector<pcl::PointCloud<PointType>::Ptr> &clouds) {
	//�e�L�l�N�g�̃f�v�X�ɑ΂��ă|���S������
	polygonMeshs.clear();
	for (int i = 0; i < clouds.size(); i++)
	{
		voxelGridFilter(0.005f, clouds[i]);
		if (clouds[i]->size() > 100)
		{
			polygonMeshs.push_back(createMeshWithOFM(clouds[i]));;
		}
	}
	cout << "NUM Polygon Mesh: " << polygonMeshs.size() << endl;
}

void PCLManager::transform(pcl::PointCloud<PointType>::Ptr cloud, float x, float y, float z, float  pitch, float yaw, float roll)
{
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	float pitchRad = pitch / 180.0 * M_PI;
	float yawRad = yaw / 180.0 * M_PI;
	float rollRad = roll / 180.0 * M_PI;
	//��]
	transform.rotate(Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitZ()));

	//�ړ�
	transform.translation() << x, y, z;

	//���s
	pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*cloud, *p, transform);
	*cloud = *p;
	p.reset();
}

void PCLManager::edgeRmoveFilter(pcl::PointCloud<PointType>::Ptr cloud) {
	//�@��
	pcl::PointCloud<PointNormalType>::Ptr normal = createNormals(cloud);

	//�@���̌X�������ȏゾ������X���[
	vector<int> removeIndex;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (abs(normal->points[i].normal_x / normal->points[i].normal_z) > 0.1)
		{
			removeIndex.push_back(i);
		}
	}

	for (int i = 0; i < removeIndex.size(); i++)
	{
		cloud->erase(cloud->points.begin() + removeIndex[i] - i);
	}
}


void PCLManager::statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud)
{
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(10);				//�ߐډ��|�C���g���g����
	sor.setStddevMulThresh(1.0);	//���̕W���΍��ȏ���t�B���^�[���Đ؂�

	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
	sor.filter(*cloud_filtered);

	pcl::copyPointCloud(*cloud_filtered, *cloud);
	cloud_filtered.reset();
}

void PCLManager::voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud)
{
	//VoxelGrid
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize(leaf, leaf, leaf);		//�t�B���^�[�͈͐ݒ�
	grid.setInputCloud(cloud);
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
	grid.filter(*cloud_filtered);
	pcl::copyPointCloud(*cloud_filtered, *cloud);
	cloud_filtered.reset();
}

void PCLManager::extractIndices(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointIndices::Ptr inliners)
{
	pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
	pcl::copyPointCloud(*cloud, *tmp);

	pcl::ExtractIndices<PointType> extract;
	extract.setInputCloud(tmp);
	extract.setIndices(inliners);

	extract.setNegative(true);
	extract.filter(*cloud);
}

pcl::PointIndices::Ptr PCLManager::getPlaneIndices(pcl::PointCloud<PointType>::Ptr cloud)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliners(new pcl::PointIndices);

	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.03);

	seg.setInputCloud(cloud);
	seg.segment(*inliners, *coefficients);

	return inliners;
}

pcl::PointCloud<pcl::Normal>::Ptr PCLManager::createNormals(pcl::PointCloud<PointType>::Ptr cloud, int KSearh)
{
	// Normal estimation
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(KSearh);
	n.compute(*normals);

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	return normals;
}

pcl::PointCloud<PointNormalType>::Ptr PCLManager::createNormals(pcl::PointCloud<PointType>::Ptr cloud)
{
	//�@���̎擾
	//�@���i�[�p�̃X�}�[�g�|�C���^��p��
	pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalType>);
	//�X���[�W���O�p�̃��W���[����p��
	pcl::MovingLeastSquares<PointType, PointNormalType> mls;
	//kdTree���W���[��
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

	mls.setComputeNormals(true);	//�@���̌v�Z���s�����ǂ���
	mls.setInputCloud(cloud);		//���͂̃|�C���g�N���E�h
	mls.setPolynomialFit(true);		//�������t�B�b�e�B���O��邩�ǂ����B�I�t�ɂ���Ƒ����͂Ȃ�B
	mls.setSearchMethod(tree);		//�T�����\�b�h�̐ݒ�
	mls.setSearchRadius(0.06);		//�������̋�i0.03�ʂ��œ|�H0.06�ł����ԕ���ɂȂ�jhttp://www.cc.kyoto-su.ac.jp/~kano/pdf/study/student/2013YamamotoPresen.pdf
									//�v�Z

	mls.process(*cloud_with_normals);

	return cloud_with_normals;
}

pcl::PolygonMesh PCLManager::createMeshWithOFM(pcl::PointCloud<PointType>::Ptr cloud)
{
	//RangeImage�Ɉ�x�ϊ�����
	float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(120.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(60.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.0;
	float minRange = 0.0f;
	int borderSize = 0;

	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	//std::cout << rangeImage << std::endl;

	//RangeImage����ēxPointCloud��Organized Point Cloud�Ƃ��ĕϊ�
	pcl::PointCloud<PointType>::Ptr rangedCloud(new pcl::PointCloud<PointType>());
	rangedCloud->width = rangeImage.width;
	rangedCloud->height = rangeImage.height;
	rangedCloud->resize(rangeImage.width * rangeImage.height);
	for (int i = 0; i < rangeImage.size(); i++)
	{
		pcl::copyPoint(rangeImage.points[i], rangedCloud->points[i]);
	}

	//���b�V����
	pcl::OrganizedFastMesh<PointType> ofm;
	ofm.setTrianglePixelSize(2);

	ofm.setTriangulationType(pcl::OrganizedFastMesh<PointType>::TRIANGLE_ADAPTIVE_CUT);
	ofm.setInputCloud(rangedCloud);
	pcl::PolygonMesh mesh;
	ofm.reconstruct(mesh);

	return mesh;
}

pcl::PolygonMesh PCLManager::createMeshWithGP3(pcl::PointCloud<PointType>::Ptr cloud)
{
	////�@���̎擾
	////�@���i�[�p�̃X�}�[�g�|�C���^��p��
	//pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalType>);
	////�X���[�W���O�p�̃��W���[����p��
	//pcl::MovingLeastSquares<PointType, PointNormalType> mls;
	////kdTree���W���[��
	//pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

	//mls.setComputeNormals(true);	//�@���̌v�Z���s�����ǂ���
	//mls.setInputCloud(cloud);		//���͂̃|�C���g�N���E�h
	//mls.setPolynomialFit(true);		//�������t�B�b�e�B���O��邩�ǂ����B�I�t�ɂ���Ƒ����͂Ȃ�B
	//mls.setSearchMethod(tree);		//�T�����\�b�h�̐ݒ�
	//mls.setSearchRadius(0.04);		//�������̋�i0.03�ʂ��œ|�H0.06�ł����ԕ���ɂȂ�jhttp://www.cc.kyoto-su.ac.jp/~kano/pdf/study/student/2013YamamotoPresen.pdf
	//								//�v�Z
	//mls.process(*cloud_with_normals);


	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//���b�V����
	//Greedy Projection Triangulate�A���S���Y�����g�p
	pcl::GreedyProjectionTriangulation<PointNormalType> gp3;

	gp3.setSearchRadius(0.025);				//�ߖT�T���Ɏg���X�t�B�A�̃T�C�Y
	gp3.setMu(2.5);							//�ߖT�T�����s���̂Ɏg���搔
	gp3.setMaximumNearestNeighbors(100);	//�ߖT�ɂ��������邩�̍ő吔�ݒ�

	gp3.setMaximumSurfaceAngle(M_PI);	//�P�ʐ��K�΍�������ȏゾ�����疳��
	gp3.setMinimumAngle(M_PI / 18);			//�e�|���S���̎�������\�ʖ@���̌X��
	gp3.setMaximumAngle(2 * M_PI / 2);		//�e�|���S���̎�������\�ʖ@���̌X���ő�l
	gp3.setNormalConsistency(true);		//�i���v���T�[�`�j

											//���ʂ̎擾
	gp3.setInputCloud(cloud_with_normals);
	pcl::search::KdTree<PointNormalType>::Ptr tree3(new pcl::search::KdTree<PointNormalType>);
	gp3.setSearchMethod(tree3);
	pcl::PolygonMesh mesh;
	gp3.reconstruct(mesh);

	return mesh;
}

Eigen::Matrix4f PCLManager::iterativeClosestPoint(pcl::PointCloud<PointType>::Ptr target, pcl::PointCloud<PointType>::Ptr source)
{
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setInputTarget(target);
	icp.setInputSource(source);
	icp.align(*source);

	return icp.getFinalTransformation();
}

pcl::PolygonMesh PCLManager::concaveHull(pcl::PointCloud<PointType>::Ptr cloud) {
	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<PointType>::Ptr cloud_hull(new pcl::PointCloud<PointType>);
	pcl::ConcaveHull<PointType> chull;
	chull.setInputCloud(cloud);
	chull.setAlpha(0.03);
	pcl::PointCloud<PointType>::Ptr voronoi_centers(new pcl::PointCloud<PointType>);
	chull.setVoronoiCenters(voronoi_centers);
	chull.setKeepInformation(true);
	vector<pcl::Vertices> polygons;
	chull.reconstruct(*cloud_hull, polygons);

	pcl::PolygonMesh mesh;

	pcl::toROSMsg(*cloud_hull, mesh.cloud);
	mesh.polygons = polygons;

	*cloud = *cloud_hull;


	return mesh;
}

void PCLManager::createRangeImage(pcl::PointCloud<PointType>::Ptr cloud, pcl::RangeImage &rangeImage)
{
	//RangeImage�Ɉ�x�ϊ�����
	float sensorAngleWidht = 90.0f;
	float sensorAngleHeight = 60.0f;
	float angularResolution = (float)(0.15f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(sensorAngleWidht * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(sensorAngleHeight * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.0;
	float minRange = 0.0f;
	int borderSize = 0;

	rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	//std::cout << rangeImage << std::endl;
}

void PCLManager::createOrganizedCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	pcl::RangeImage rangeImage;
	createRangeImage(inputCloud, rangeImage);

	//RangeImage����ēxPointCloud��Organized Point Cloud�Ƃ��ĕϊ�
	pcl::PointCloud<PointType>::Ptr rangedCloud(new pcl::PointCloud<PointType>());
	rangedCloud->width = rangeImage.width;
	rangedCloud->height = rangeImage.height;
	rangedCloud->resize(rangeImage.width * rangeImage.height);
	pcl::copyPointCloud(rangeImage, *rangedCloud);
	*outputCloud = *rangedCloud;
}

void PCLManager::euclideanClusterExtraction(pcl::PointCloud<PointType>::Ptr cloud, vector<pcl::PointCloud<PointType>::Ptr> &outputCloud) {
	//Cluster�ɕ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.07); // 2cm
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	//�������ꂽ�C���f�b�N�X�����ɁA�N���E�h�𕪊����ĕԂ�
	//Reset
	for (int i = 0; i < outputCloud.size(); i++)
	{
		outputCloud[i].reset();
	}
	outputCloud.clear();

	//Thread
	boost::thread_group ths;
	for (int i = 0; i < cluster_indices.size(); i++)
	{
		pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
		outputCloud.push_back(p);
		ths.create_thread(boost::bind(&PCLManager::splitCloud, this, cloud, outputCloud[i], cluster_indices[i]));
	}
	ths.join_all();
}

void PCLManager::splitCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr ouputCloud, pcl::PointIndices &indices) {
	for (int j = 0; j < indices.indices.size(); j++)
	{
		int index = indices.indices[j];
		ouputCloud->push_back(inputCloud->points[index]);
	}
}

void PCLManager::createLineWithCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	//�T������ߖT�_�̐�
	int K = 10;

	//�����Ɋi�[���Ă�
	pcl::PointCloud<PointType>::Ptr c(new pcl::PointCloud<PointType>());

	outputCloud->clear();

	//kdTree
	pcl::KdTreeFLANN<PointType> kdTree;
	kdTree.setInputCloud(inputCloud);

	int count = 0;
	for (int i = 0; i < inputCloud->size(); i += 1)
	{
		//���̓_����ŋߖT��2�_���擾���Ċi�[���Ă���
		//�T���Ώۂ̓_
		PointType p = inputCloud->points[i];

		if (p.z < 2.0)
		{
			continue;
		}

		//���ʂ��i�[�����z��
		vector<int> indexSearch(K);
		vector<float> distanceSearch(K);

		//�T��
		if (kdTree.radiusSearch(p, 0.06, indexSearch, distanceSearch, 2))
		{
			for (int j = 1; j < indexSearch.size(); j += 1)
			{
				float distance = sqrtf(distanceSearch[j]);
				int index = indexSearch[j];
				//if (distance < 0.05)
				//{
				//	c->push_back(p);
				//	c->push_back(inputCloud->points[index]);
				//}
				c->push_back(p);
				c->push_back(inputCloud->points[index]);
			}
		}
	}


	/*cout << ";;;;;;;;;;;input: " << inputCloud->size() << ", " << c->size() << endl;*/
	//�o�͂Ƀ}�[�W
	*outputCloud += *c;
}


void PCLManager::createPolygonWithRangeImage(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	//RangeImage�Ɉ�x�ϊ�����
	float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(120.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(60.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.0;
	float minRange = 0.0f;
	int borderSize = 0;

	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*inputCloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	//std::cout << rangeImage << std::endl;

	//RangeImage����ēxPointCloud��Organized Point Cloud�Ƃ��ĕϊ�
	pcl::PointCloud<PointType>::Ptr rangedCloud(new pcl::PointCloud<PointType>());
	rangedCloud->width = rangeImage.width;
	rangedCloud->height = rangeImage.height;
	rangedCloud->resize(rangeImage.width * rangeImage.height);
	for (int i = 0; i < rangeImage.size(); i++)
	{
		pcl::copyPoint(rangeImage.points[i], rangedCloud->points[i]);
	}

	outputCloud->clear();
	pcl::PointCloud<PointType>::Ptr triangleCloud(new pcl::PointCloud<PointType>());
	for (int y = 1; y < rangedCloud->height - 1; y++)
	{
		for (int x = 1; x < rangedCloud->width - 1; x++)
		{
			int index = y * rangedCloud->width + x;
			PointType p = rangedCloud->points[index];


			//��
			int indexLeft = y * rangedCloud->width + (x - 1);
			PointType leftP = rangedCloud->points[indexLeft];
			//��
			int indexTop = (y - 1) * rangedCloud->width + x;
			PointType topP = rangedCloud->points[indexTop];
			//�E
			int indexRight = y * rangedCloud->width + (x + 1);
			PointType rightP = rangedCloud->points[indexRight];
			//��
			int indexBtm = (y + 1) * rangedCloud->width + x;
			PointType btmP = rangedCloud->points[indexBtm];

			//����|���S��
			if (p.z > 1 && leftP.z > 1 && topP.z > 1)
			{
				triangleCloud->points.push_back(p);
				triangleCloud->points.push_back(topP);
				triangleCloud->points.push_back(leftP);
			}

			////�E���|���S��
			//if (rightP.z > 1 && btmP.z > 1)
			//{
			//	triangleCloud->points.push_back(p);
			//	triangleCloud->points.push_back(btmP);
			//	triangleCloud->points.push_back(rightP);
			//}

			////�����|���S��
			//if (leftP.z > 1 && btmP.z > 1)
			//{
			//	triangleCloud->points.push_back(p);
			//	triangleCloud->points.push_back(leftP);
			//	triangleCloud->points.push_back(btmP);
			//}

			//�E��|���S��
			if (p.z > 1 && rightP.z > 1 && topP.z > 1)
			{
				triangleCloud->points.push_back(p);
				triangleCloud->points.push_back(rightP);
				triangleCloud->points.push_back(topP);
			}
		}
	}

	*outputCloud = *triangleCloud;
}


void PCLManager::createPolygonWithCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
{
	//�T������ߖT�_�̐�
	int K = 90;

	//kdTree
	pcl::KdTreeFLANN<PointType> kdTree;
	kdTree.setInputCloud(inputCloud);


	for (int i = 0; i < inputCloud->size(); i += 1)
	{
		//���̓_����ŋߖT��2�_���擾���Ċi�[���Ă���
		//�T���Ώۂ̓_
		PointType p = inputCloud->points[i];

		//���ʂ��i�[�����z��
		vector<int> indexSearch(K);
		vector<float> distanceSearch(K);

		//�T��
		if (kdTree.radiusSearch(p, 0.3, indexSearch, distanceSearch, K))
		{
			if (indexSearch.size() >= 3)
			{
				for (int j = 0; j < indexSearch.size(); j += 3)
				{
					if (sqrtf(distanceSearch[j]) + sqrtf(distanceSearch[j + 1]) + sqrtf(distanceSearch[j + 2]) < 0.36)
					{
						outputCloud->push_back(inputCloud->points[indexSearch[j]]);
						outputCloud->push_back(inputCloud->points[indexSearch[j + 1]]);
						outputCloud->push_back(inputCloud->points[indexSearch[j + 2]]);
					}
				}
			}
		}
	}
}

pcl::PointCloud<PointType>::Ptr PCLManager::projectionToZ(pcl::PointCloud<PointType>::Ptr cloud, float zValue) {
	//�^����ꂽZ�̒l�ɂ��镽�ʂɃN���E�h���v���W�F�N�V��������
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = zValue;

	// Create the filtering object
	pcl::PointCloud<PointType>::Ptr cloud_projected(new pcl::PointCloud<PointType>());
	pcl::ProjectInliers<PointType> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	return cloud_projected;
}

Eigen::Vector4f PCLManager::centroid(pcl::PointCloud<PointType>::Ptr cloud)
{
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);//�d�S���v�Z

	return xyz_centroid;
}