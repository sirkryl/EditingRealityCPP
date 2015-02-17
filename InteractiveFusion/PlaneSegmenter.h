#pragma once
#include "Segmenter.h"


namespace InteractiveFusion {
	class PlaneSegmenter : public Segmenter
	{
	public:
		PlaneSegmenter();
		~PlaneSegmenter();

		virtual void SetSegmentationParameters(PlaneSegmentationParams* _segmentationParams);

		void ConfirmLastSegment();
		void RejectLastSegment();
		void RemoveLastSegmentForNewSegmentation();

		virtual void FinishSegmentation(ModelData* _inputModelData, ModelData* _outputModelData);

		void CleanUp();
	protected:
		PlaneSegmentationParams segmentationParameters;

		Eigen::Vector3f axisUsedForSegmentation;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloudWithoutNormals;
		pcl::PointCloud<pcl::Normal>::Ptr mainCloudNormals;

		pcl::PointIndices::Ptr allPlaneSegmentIndices;
		pcl::PointIndices::Ptr indicesUsedForNextPlaneSegmentation;
		pcl::PointIndices::Ptr indicesFromLastPlaneSegmentation;

		std::vector<pcl::ModelCoefficients::Ptr> planeCoefficients;

		int axisChangedCount = 0;
		int indexOfGroundPlane = -1;
		PlaneParameters GetModelCoefficients(int _clusterIndex);

		virtual bool Segment();
		virtual bool ConvertToPointCloud(MeshContainer &_mesh);
		virtual void UpdateHighlights(ModelData* _modelData);
		void UpdateIndicesUsedForNextSegmentation();
		pcl::PointIndices GetIndicesThatWereNotSegmented();
	};
}

