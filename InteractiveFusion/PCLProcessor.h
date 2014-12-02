#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/keyboard_event.h>

class PCLProcessor
{
public:
	bool PlaneSegmentation();
	bool EuclideanSegmentation();
	bool RegionGrowingSegmentation();
	//bool DifferenceOfNormalsSegmentation();
	pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr coloredSegmentedCloud;
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr wallSegmentCloud;
	bool ConvertToCloud(std::vector<Vertex> startingVertices, std::vector<Triangle> startingIndices);
	bool ConvertToTriangleMesh(int clusterIndex, std::vector<Vertex> allVertices, std::vector<Vertex> &vertices, std::vector<Triangle> &indices, bool isPlane);
	//bool ConvertPlaneToTriangleMesh(int clusterIndex, std::vector<float> allVertices, std::vector<float> &vertices, std::vector<GLuint> &indices);
	void PoissonReconstruction();
	void PlaneIndexEstimation();
	void IndexEstimation();
	void ConfirmPlaneIndices(int index);
	//void MovingLeastSquares(std::vector<Vertex> vertices, std::vector<float> normals, std::vector<Vertex> &outputVertices, std::vector<GLuint> &outputIndices);
	bool coloredCloudReady = false;
	void ShowViewer();
	bool IsMainCloudInitialized();
	bool IsPlaneSegmented();
	std::vector<int> GetInlierIndices();
	std::vector<int> GetPlaneCloudIndices(int index);
	std::vector<int> GetColoredCloudIndices(int index);
	void PrepareForObjectSegmentation();
	int GetClusterCount();
	int GetRegionClusterCount();
	int GetPlaneClusterCount();
	int GetCloudSize();
	std::vector<pcl::ModelCoefficients::Ptr> planeCoefficients;

	void ClearAll();
private:
	
	int clusterCount = 0;
	bool isPlaneSegmented = false;
	int clusterIndexCount = 0;
	int procIndexCount = 0;
	bool closeViewer = false;
	bool isWall = false;
	std::vector<pcl::PointIndices> segmentedClusterIndices;
	std::vector<pcl::PointIndices::Ptr> planeCloudIndices;
	//bool EnforceCurvatureOrColor(const pcl::PointXYZRGB& point_a, const pcl::PointXYZRGB& point_b, float squared_distance);
	void KeyDown(const pcl::visualization::KeyboardEvent &keyEvent, void* viewer_void);
	std::vector<int> CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	bool CompareTwoPoints(pcl::PointXYZRGB &ptA, pcl::PointXYZRGB &ptB);
	
	
};

extern PCLProcessor pclProcessor;

void ViewerCallback(pcl::visualization::PCLVisualizer& viewer);
void ViewerInit(pcl::visualization::PCLVisualizer& viewer);