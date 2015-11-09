#include "Segmenter.h"
#include "GraphicsController.h"
#include "ModelData.h"
#include "MeshContainer.h"
#include "StopWatch.h"
#include <pcl/common/io.h>
#include "DebugUtility.h"

using namespace std;

namespace InteractiveFusion {
	Segmenter::Segmenter()
	{
		mainCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	}


	Segmenter::~Segmenter()
	{
	}

	bool Segmenter::InitializeSegmentation(ModelData& _modelData)
	{
		std::shared_ptr<MeshContainer> remainingMesh = _modelData.GetFirstMeshThatIsNotPlane();
		if (remainingMesh == nullptr)
		{
			DebugUtility::DbgOut(L"GraphicsController::StartSegmentationThread::No Mesh found that is not a plane");
			return false;
		}
		if (!ConvertToPointCloud(*remainingMesh))
			return false;

		return true;
	}

	bool Segmenter::UpdateSegmentation(GraphicsController& _glControl, ModelData& _modelData)
	{
		if (!HasPointCloudData())
		{
			if (!InitializeSegmentation(_modelData))
				return false;
		}

		bool segmentationResult = Segment();
		EstimateIndices();
		if (!segmentationResult)
		{
			DebugUtility::DbgOut(L"So i guess you are here.. ", (int)GetClusterCount());
			return false;
		}

		_modelData.RemoveTemporaryTriangleColor();

		UpdateHighlights(_modelData);
		_glControl.PushEvent(GraphicsControlEvent::ModelHighlightsUpdated);
		return true;
	}

	void Segmenter::UpdateHighlights(ModelData& _modelData)
	{

	}
	void Segmenter::FinishSegmentation(ModelData& _inputModelData, ModelData& _outputModelData)
	{

	}

	bool Segmenter::HasPointCloudData()
	{
		return meshVertices.size() != 0;
	}

	bool Segmenter::ConvertToPointCloud(MeshContainer &_mesh)
	{
		CleanUp();

		meshVertices = _mesh.GetVertices();

		DebugUtility::DbgOut(L"Segmenter::ConvertToPointCloud::Mesh Vertices size: ", (int)meshVertices.size());

		if (meshVertices.size() == 0)
		{
			DebugUtility::DbgOut(L"Segmenter::ConvertToPointCloud::ERROR::Mesh has no vertices.");
			return false;
		}

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		
		cloud->width = meshVertices.size();
		cloud->height = 1;
		cloud->is_dense = true;
		cloud->points.resize(cloud->width * cloud->height);

		for (int i = 0; i < meshVertices.size(); i += 1)
		{
			cloud->points[i].x = meshVertices[i].x;
			cloud->points[i].y = meshVertices[i].y;
			cloud->points[i].z = meshVertices[i].z;
			cloud->points[i].r = meshVertices[i].r * 255;
			cloud->points[i].g = meshVertices[i].g * 255;
			cloud->points[i].b = meshVertices[i].b * 255;
			cloud->points[i].normal_x = meshVertices[i].normal_x;
			cloud->points[i].normal_y = meshVertices[i].normal_y;
			cloud->points[i].normal_z = meshVertices[i].normal_z;

			vertexMap[meshVertices[i].x].push_back(i);
			vertexMap[meshVertices[i].x].push_back(meshVertices[i].y);
			vertexMap[meshVertices[i].x].push_back(meshVertices[i].z);
		}

		vector<Triangle> meshTriangles = _mesh.GetTriangles();

		for (auto &triangle : meshTriangles)
		{
			indexMap[triangle.v1].push_back(triangle.v1);
			indexMap[triangle.v1].push_back(triangle.v2);
			indexMap[triangle.v1].push_back(triangle.v3);
			indexMap[triangle.v2].push_back(triangle.v1);
			indexMap[triangle.v2].push_back(triangle.v2);
			indexMap[triangle.v2].push_back(triangle.v3);
			indexMap[triangle.v3].push_back(triangle.v1);
			indexMap[triangle.v3].push_back(triangle.v2);
			indexMap[triangle.v3].push_back(triangle.v3);

		}

		pcl::copyPointCloud(*cloud, *mainCloud);

		DebugUtility::DbgOut(L"Segmenter::ConvertToPointCloud::Maincloud size: ", (int)mainCloud->points.size());

		return true;
	}

	void Segmenter::EstimateIndices()
	{
		for (auto& cluster : finalSegmentationClusters)
		{
			cluster->clear();
		}
		finalSegmentationClusters.clear();
		finalSegmentationClusterIndices.clear();

		for (auto& singleClusterIndices : temporarySegmentationClusterIndices)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr segmentedClusterWithAlignedIndices(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

			for (auto& index : singleClusterIndices.indices)
			{
				segmentedClusterWithAlignedIndices->points.push_back(mainCloud->points[index]);
			}

			segmentedClusterWithAlignedIndices->width = segmentedClusterWithAlignedIndices->points.size();
			segmentedClusterWithAlignedIndices->height = 1;
			segmentedClusterWithAlignedIndices->is_dense = true;

			finalSegmentationClusterIndices.push_back(CalculateIndicesForCluster(segmentedClusterWithAlignedIndices));

			finalSegmentationClusters.push_back(segmentedClusterWithAlignedIndices);
		}
	}

	std::vector<int> Segmenter::CalculateIndicesForCluster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
	{
		std::vector<int> calculatedIndices;

		for (auto& vertex : cloud->points)
		{
			vector<float> savedVertexValues = vertexMap[vertex.x];
			for (int j = 0; j < savedVertexValues.size(); j += 3)
			{
				if (savedVertexValues[j + 1] == vertex.y &&
					savedVertexValues[j + 2] == vertex.z)
				{
					calculatedIndices.push_back(savedVertexValues[j]);
					break;
				}
			}
		}
		return calculatedIndices;
	}

	bool Segmenter::Segment() 
	{
		return false;
	}

	MeshContainer Segmenter::ConvertToMesh(int _clusterIndex)
	{
		if (!IsValidClusterIndex(_clusterIndex))
		{
			DebugUtility::DbgOut(L"Segmenter::ConvertToMesh::ERROR:: Invalid Cluster Index");
			return MeshContainer();
		}
		if (meshVertices.size() == 0)
		{
			DebugUtility::DbgOut(L"Segmenter::ConvertToMesh::ERROR:: No mesh vertices available ");
			return MeshContainer();
		}

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster = finalSegmentationClusters[_clusterIndex];

		if (cluster->empty()) 
		{
			DebugUtility::DbgOut(L"Segmenter::ConvertToMesh::ERROR:: clusterCloud is empty ");
			return MeshContainer();
		}

		StopWatch stopWatch;
		stopWatch.Start();




		vector<Vertex> resultingVertices;
		vector<Triangle> resultingTriangles;

		Vertex centerVertex ( 0, 0, 0, 0, 0, 0, 0, 0, 0 );
		//vertices are straight-forward and are mapped 1:1

		for (auto& point : cluster->points)
		{
			Vertex vertex;
			vertex.x = point.x;
			vertex.y = point.y;
			vertex.z = point.z;
			vertex.r = point.r / 255.0f;
			vertex.g = point.g / 255.0f;
			vertex.b = point.b / 255.0f;
			vertex.normal_x = point.normal_x;
			vertex.normal_y = point.normal_y;
			vertex.normal_z = point.normal_z;
			resultingVertices.push_back(vertex);

			centerVertex.x += vertex.x;
			centerVertex.y += vertex.y;
			centerVertex.z += vertex.z;
		}

		centerVertex.x = centerVertex.x / (float)cluster->points.size();
		centerVertex.y = centerVertex.y / (float)cluster->points.size();
		centerVertex.z = centerVertex.z / (float)cluster->points.size();


		//for indices, we use a vector for iteration AND an unordered_map for retrieval for the best possible performance
		std::vector<int> cloudIndices = finalSegmentationClusterIndices[_clusterIndex];
		
		std::unordered_map<int, int> clusterIndexMap;

		//iterator for find-operations
		std::unordered_map<int, int>::const_iterator mapIterator;

		//fill indexMap with indices
		for (size_t i = 0; i < cloudIndices.size(); i++)
		{
			clusterIndexMap[cloudIndices[i]] = i;
		}

		//in this loop, we try to recreate the appropriate index values for the mesh cluster by using the data collected at ConvertToCloud()
		for (size_t i = 0; i < cloudIndices.size(); i++)
		{
			vector<int> savedIndexValues = indexMap[cloudIndices[i]];
			for (size_t j = 0; j < savedIndexValues.size(); j += 3)
			{
				Triangle triangle;
				/*if the index is the first value in the current face, look for 2nd and 3rd in the current cluster
				and either add them to the new face or add new vertices to fill the face */
				if (savedIndexValues[j] == cloudIndices[i])
				{
					//first index of face
					triangle.v1 = i;

					//second index of face

					//way faster than vector::find or unordered_set::find for that matter
					mapIterator = clusterIndexMap.find(savedIndexValues[j + 1]);

					size_t indexOfSecondVertex;
					if (mapIterator != clusterIndexMap.end())
					{
						indexOfSecondVertex = clusterIndexMap[savedIndexValues[j + 1]];
					}
					else
					{
						indexOfSecondVertex = resultingVertices.size();
						resultingVertices.push_back(meshVertices[savedIndexValues[j + 1]]);
					}
					triangle.v2 = indexOfSecondVertex;

					//third index of face
					mapIterator = clusterIndexMap.find(savedIndexValues[j + 2]);

					size_t indexOfThirdVertex;
					if (mapIterator != clusterIndexMap.end())
					{
						indexOfThirdVertex = clusterIndexMap[savedIndexValues[j + 2]];
					}
					else
					{
						indexOfThirdVertex = resultingVertices.size();
						resultingVertices.push_back(meshVertices[savedIndexValues[j + 2]]);
					}

					triangle.v3 = indexOfThirdVertex;

					resultingTriangles.push_back(triangle);
				}
				else if (savedIndexValues[j + 1] == cloudIndices[i])
				{
					//first index of face
					mapIterator = clusterIndexMap.find(savedIndexValues[j]);

					size_t indexOfFirstVertex;
					if (mapIterator != clusterIndexMap.end())
					{
						indexOfFirstVertex = clusterIndexMap[savedIndexValues[j]];
					}
					else
					{
						indexOfFirstVertex = resultingVertices.size();
						resultingVertices.push_back(meshVertices[savedIndexValues[j]]);
					}

					triangle.v1 = indexOfFirstVertex;

					//second index of face
					triangle.v2 = i;

					//third index of face
					mapIterator = clusterIndexMap.find(savedIndexValues[j + 2]);

					size_t indexOfThirdVertex;
					if (mapIterator != clusterIndexMap.end())
					{
						indexOfThirdVertex = clusterIndexMap[savedIndexValues[j + 2]];
					}
					else
					{
						indexOfThirdVertex = resultingVertices.size();
						resultingVertices.push_back(meshVertices[savedIndexValues[j + 2]]);
					}
					triangle.v3 = indexOfThirdVertex;

					resultingTriangles.push_back(triangle);
				}
				else if (savedIndexValues[j + 2] == cloudIndices[i])
				{
					//first index of face
					mapIterator = clusterIndexMap.find(savedIndexValues[j]);

					size_t indexOfFirstVertex;
					if (mapIterator != clusterIndexMap.end())
					{
						indexOfFirstVertex = clusterIndexMap[savedIndexValues[j]];
					}
					else
					{
						indexOfFirstVertex = resultingVertices.size();
						resultingVertices.push_back(meshVertices[savedIndexValues[j]]);
					}
					triangle.v1 = indexOfFirstVertex;

					//second index of face
					mapIterator = clusterIndexMap.find(savedIndexValues[j + 1]);

					size_t indexOfSecondVertex;
					if (mapIterator != clusterIndexMap.end())
					{
						indexOfSecondVertex = clusterIndexMap[savedIndexValues[j + 1]];
					}
					else
					{
						indexOfSecondVertex = resultingVertices.size();
						resultingVertices.push_back(meshVertices[savedIndexValues[j + 1]]);
					}
					triangle.v2 = indexOfSecondVertex;

					//third index of face
					triangle.v3 = i;
					resultingTriangles.push_back(triangle);
				}
			}
		}
		DebugUtility::DbgOut(L"Cluster to mesh with indexSet in ", stopWatch.Stop());

		return MeshContainer(resultingVertices, resultingTriangles);
	}


	bool Segmenter::IsValidClusterIndex(int _clusterIndex)
	{
		if (_clusterIndex >= 0 && _clusterIndex < GetClusterCount())
			return true;
		return false;
	}

	int Segmenter::GetClusterCount()
	{
		return finalSegmentationClusters.size();
	}

	std::vector<int> Segmenter::GetClusterIndices(int _clusterIndex)
	{
		std::vector<int> triangles;
		if (!IsValidClusterIndex(_clusterIndex))
		{
			DebugUtility::DbgOut(L"Segmenter::GetClusterIndices::ERROR:: not a valid cluster index");
			return triangles;
		}
		for (auto& index : temporarySegmentationClusterIndices[_clusterIndex].indices)
		{
			triangles.push_back(index);
		}
		return triangles;
	}

	void Segmenter::CleanUp()
	{
		mainCloud->clear();
		indexMap.clear();
		vertexMap.clear();
		temporarySegmentationClusterIndices.clear();
		finalSegmentationClusterIndices.clear();
		for (auto pointCloud : finalSegmentationClusters)
			pointCloud->clear();
		finalSegmentationClusters.clear();
		meshVertices.clear();
	}

}