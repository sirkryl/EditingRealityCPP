#pragma once


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include "SegmentationParams.h"
#include "VertexHasher.h"
#include "CommonStructs.h"
#include <unordered_map>
#include <vector>
#include "MeshContainer.h"

namespace InteractiveFusion {

	
	class ModelData;
	class GraphicsController;
	class Segmenter
	{
	public:
		Segmenter();
		virtual ~Segmenter();

		virtual bool UpdateSegmentation(GraphicsController& _glControl, ModelData& _modelData);
		
		virtual void FinishSegmentation(ModelData& _inputModelData, ModelData& _outputModelData);
		
		int GetClusterCount();

		void CleanUp();

	protected:
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mainCloud;
		std::unordered_map<int, std::vector<int>> indexMap;

		std::unordered_map<Vertex, int, VertexHasher> vertexMap;
		std::vector<pcl::PointIndices> temporarySegmentationClusterIndices;
		std::vector<pcl::PointIndices> rejectedIndices;
		std::vector<std::vector<int>> finalSegmentationClusterIndices;
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> finalSegmentationClusters;

		std::vector<Vertex> meshVertices;

		virtual bool InitializeSegmentation(ModelData& _modelData);

		virtual bool ConvertToPointCloud(MeshContainer &_mesh);

		virtual bool Segment();
		
		int CalculateVertexHash(Vertex _vertex);

		void EstimateIndices();

		std::vector<int> CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

		std::vector<int> GetClusterIndices(int _clusterIndex);

		MeshContainer ConvertToMesh(int _clusterIndex);

		virtual void UpdateHighlights(ModelData& _modelData);

		bool HasPointCloudData();
		bool IsValidClusterIndex(int _index);
	};
}
