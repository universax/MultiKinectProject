#pragma once
class PCLManager
{
public:
	PCLManager();
	void update(std::vector<pcl::PointCloud<PointType>::Ptr> &outputPoints);
	void registrationAll(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputClouds, Eigen::Matrix4f &matrix);
	void createPolygon(vector<pcl::PointCloud<PointType>::Ptr>& clouds);
	
	inline pcl::PointCloud<PointType>::Ptr getMargedPoints() { return margedCloud; }
	inline vector<pcl::PointCloud<PointType>::Ptr> getClusteredPoints() {
		return eachClouds;
	}
	inline vector<pcl::PolygonMesh> getPlygonMeshs() { return polygonMeshs; }
	inline vector<Eigen::Vector4f> getCentroids() { return centroids; }

private:
	//�ړ��E��]
	void transform(pcl::PointCloud<PointType>::Ptr cloud, float x, float y, float z, float  pitch, float yaw, float roll);

	void edgeRmoveFilter(pcl::PointCloud<PointType>::Ptr cloud);

	//Filter
	void statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud);
	void voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud);
	void extractIndices(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointIndices::Ptr inliners);

	//Segmentation
	pcl::PointIndices::Ptr getPlaneIndices(pcl::PointCloud<PointType>::Ptr cloud);

	//Create Normal
	pcl::PointCloud<pcl::Normal>::Ptr createNormals(pcl::PointCloud<PointType>::Ptr cloud, int KSearh);
	pcl::PointCloud<PointNormalType>::Ptr createNormals(pcl::PointCloud<PointType>::Ptr cloud);

	//CreateMesh
	pcl::PolygonMesh createMeshWithOFM(pcl::PointCloud<PointType>::Ptr cloud);
	pcl::PolygonMesh createMeshWithGP3(pcl::PointCloud<PointType>::Ptr cloud);

	//ICP
	Eigen::Matrix4f iterativeClosestPoint(pcl::PointCloud<PointType>::Ptr target, pcl::PointCloud<PointType>::Ptr source);
	Eigen::Matrix4f transformationVector[NUM_KINECT];	//just for debug
	Eigen::Matrix4f ICPTransformMatrix[NUM_KINECT];

	//Concave Hull
	pcl::PolygonMesh concaveHull(pcl::PointCloud<PointType>::Ptr cloud);

	//RangeImage
	void createRangeImage(pcl::PointCloud<PointType>::Ptr cloud, pcl::RangeImage &rangeImage);
	void createOrganizedCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud);

	//EuclideanClusterExtraction�i�N���X�^���ɃN���E�h�𕪊��j
	//�}�[�W�����N���E�h
	pcl::PointCloud<PointType>::Ptr margedCloud;
	//�N���X�^���������N���E�h�Q
	vector<pcl::PointCloud<PointType>::Ptr> eachClouds;

	//�N���X�^�ɕ�������
	void euclideanClusterExtraction(pcl::PointCloud<PointType>::Ptr cloud, vector<pcl::PointCloud<PointType>::Ptr> &outputCloud);
	void splitCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr ouputCloud, pcl::PointIndices &indices);

	//�^����ꂽ�N���E�h����A�|���S���𐶐�������
	void createLineWithCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud);
	void createPolygonWithCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud);
	void createPolygonWithRangeImage(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud);

	//z���ʂɃv���W�F�N�V����������
	pcl::PointCloud<PointType>::Ptr projectionToZ(pcl::PointCloud<PointType>::Ptr cloud, float zValue);

	//�d�S���Z�o������
	Eigen::Vector4f centroid(pcl::PointCloud<PointType>::Ptr cloud);

	//Mesh
	vector<pcl::PolygonMesh> polygonMeshs;

	//Centroids
	vector<Eigen::Vector4f> centroids;

};
