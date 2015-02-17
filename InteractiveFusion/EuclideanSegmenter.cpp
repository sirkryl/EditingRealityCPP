#include "EuclideanSegmenter.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/io.h>
#include "DebugUtility.h"
#include "StopWatch.h"
namespace InteractiveFusion {
	EuclideanSegmenter::EuclideanSegmenter() :
		ObjectSegmenter()
	{
	}


	EuclideanSegmenter::~EuclideanSegmenter()
	{
	}

	void EuclideanSegmenter::SetSegmentationParameters(ObjectSegmentationParams* _segmentationParams)
	{
		DebugUtility::DbgOut(L"EuclideanSegmenter::SetSegmentationParameters");
		EuclideanSegmentationParams* temporaryParams = dynamic_cast<EuclideanSegmentationParams*>(_segmentationParams);
		if (temporaryParams != nullptr)
			segmentationParameters = *temporaryParams;
		else
			segmentationParameters = EuclideanSegmentationParams();
	}

	bool EuclideanSegmenter::Segment()
	{
		//pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

		StopWatch stopWatch;
		stopWatch.Start();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloudWithoutNormals(new pcl::PointCloud<pcl::PointXYZRGB>);

		copyPointCloud(*mainCloud, *mainCloudWithoutNormals);

		temporarySegmentationClusterIndices.clear();

		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;


		ec.setClusterTolerance(segmentationParameters.clusterTolerance);


		ec.setMinClusterSize(segmentationParameters.minComponentSize);

		ec.setMaxClusterSize(9000000);
		//ec.setSearchMethod(tree);
		ec.setInputCloud(mainCloudWithoutNormals);
		ec.extract(temporarySegmentationClusterIndices);

		DebugUtility::DbgOut(L"Euclidean clustering in ", stopWatch.Stop());

		if (temporarySegmentationClusterIndices.size() == 0)
			return false;
		return true;
	}
}