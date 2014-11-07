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
	bool ConvertToCloud(std::vector<float> startingVertices, std::vector<GLuint> startingIndices, std::vector<float> startingNormals);
	bool ConvertToTriangleMesh(int clusterIndex, std::vector<float> allVertices, std::vector<float> &vertices, std::vector<GLuint> &indices);
	//bool ConvertPlaneToTriangleMesh(int clusterIndex, std::vector<float> allVertices, std::vector<float> &vertices, std::vector<GLuint> &indices);
	void PoissonReconstruction();
	void PlaneIndexEstimation();
	void IndexEstimation();
	void MovingLeastSquares(std::vector<float> vertices, std::vector<float> normals, std::vector<float> &outputVertices, std::vector<GLuint> &outputIndices, std::vector<float> &outputNormals);
	bool coloredCloudReady = false;
	void ShowViewer();
	bool IsMainCloudInitialized();
	bool IsPlaneSegmented();
	int GetClusterCount();
	int GetRegionClusterCount();
	int GetPlaneClusterCount();
	int GetCloudSize();
	std::vector<pcl::ModelCoefficients::Ptr> planeCoefficients;
private:
	
	int clusterCount = 0;
	bool isPlaneSegmented = false;
	int clusterIndexCount = 0;
	int procIndexCount = 0;
	bool closeViewer = false;
	bool isWall = false;
	std::vector<pcl::PointIndices> segmentedClusterIndices;
	std::vector<pcl::PointIndices::Ptr> planeCloudIndices;
	
	void KeyDown(const pcl::visualization::KeyboardEvent &keyEvent, void* viewer_void);
	std::vector<int> CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	bool CompareTwoPoints(pcl::PointXYZRGB &ptA, pcl::PointXYZRGB &ptB);
	
	
};

void ViewerCallback(pcl::visualization::PCLVisualizer& viewer);
void ViewerInit(pcl::visualization::PCLVisualizer& viewer);