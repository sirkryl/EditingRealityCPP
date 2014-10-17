#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>

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
	void PoissonReconstruction();
	void IndexEstimation();
	bool coloredCloudReady = false;
	void ShowViewer();
	int GetClusterCount();
	int GetCloudSize();
private:
	int clusterCount = 0;
	int clusterIndexCount = 0;
	int procIndexCount = 0;
	std::vector<pcl::PointIndices> segmentedClusterIndices;
	
	void KeyDown(const pcl::visualization::KeyboardEvent &keyEvent, void* viewer_void);
	void CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	bool CompareTwoPoints(pcl::PointXYZRGB &ptA, pcl::PointXYZRGB &ptB);
	
	
};

void ViewerCallback(pcl::visualization::PCLVisualizer& viewer);
void ViewerInit(pcl::visualization::PCLVisualizer& viewer);