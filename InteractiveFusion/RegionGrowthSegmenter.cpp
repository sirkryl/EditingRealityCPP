#include "RegionGrowthSegmenter.h"
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "DebugUtility.h"
#include "StopWatch.h"
namespace InteractiveFusion {
	RegionGrowthSegmenter::RegionGrowthSegmenter() : 
		ObjectSegmenter()
	{
		
	}


	RegionGrowthSegmenter::~RegionGrowthSegmenter()
	{

	}

	void RegionGrowthSegmenter::SetSegmentationParameters(ObjectSegmentationParams& _segmentationParams)
	{
		DebugUtility::DbgOut(L"RegionGrowthSegmenter::SetSegmentationParameters");

		RegionGrowthSegmentationParams* temporaryParams = dynamic_cast<RegionGrowthSegmentationParams*>(&_segmentationParams);
		if (temporaryParams != nullptr)
			segmentationParameters = *temporaryParams;
		else
			segmentationParameters = RegionGrowthSegmentationParams();
		
	}

	bool RegionGrowthSegmenter::Segment()
	{
		temporarySegmentationClusterIndices.clear();

		pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> >(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

		StopWatch stopWatch;
		stopWatch.Start();

		
		pcl::PointCloud<pcl::Normal>::Ptr temporaryNormalCloud(new pcl::PointCloud<pcl::Normal>);

		temporaryNormalCloud->width = mainCloud->points.size();
		temporaryNormalCloud->height = 1;
		temporaryNormalCloud->is_dense = true;
		temporaryNormalCloud->points.resize(mainCloud->width * mainCloud->height);

		for (int i = 0; i < mainCloud->points.size(); i++)
		{
			temporaryNormalCloud->points[i].normal_x = mainCloud->points[i].normal_x;
			temporaryNormalCloud->points[i].normal_y = mainCloud->points[i].normal_y;
			temporaryNormalCloud->points[i].normal_z = mainCloud->points[i].normal_z;
		}
		pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
		//reg.setMinClusterSize(openGLWin.minClusterSize);
		reg.setMinClusterSize(segmentationParameters.minComponentSize);
		reg.setMaxClusterSize(1000000);

		reg.setSearchMethod(tree);
		reg.setResidualTestFlag(true);
		reg.setNumberOfNeighbours(segmentationParameters.numberOfNeighbors);
		reg.setInputCloud(mainCloud);
		/*if (cloudWithoutPlaneIndices.size() > 0)
			reg.setIndices(cloudWithoutPlaneIndices[0]);
		if (!cloudWithoutPlane->empty())
			DebugUtility::DbgOut(L"NOT EMPTY ", (int)cloudWithoutPlaneIndices.size());*/
		
		reg.setInputNormals(temporaryNormalCloud);


		reg.setSmoothnessThreshold((segmentationParameters.smoothnessThreshold / 10.0) / 180.0 * M_PI);
		reg.setCurvatureThreshold((segmentationParameters.curvatureThreshold / 10.0));

		reg.extract(temporarySegmentationClusterIndices);
		DebugUtility::DbgOut(L"Region based clustering in ", stopWatch.Stop());

		if (temporarySegmentationClusterIndices.size() == 0)
			return false;

		return true;
	}
}