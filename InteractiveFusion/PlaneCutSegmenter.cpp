#include "PlaneCutSegmenter.h"

#include "DebugUtility.h"
#include "ModelData.h"

namespace InteractiveFusion {



	PlaneCutSegmenter::PlaneCutSegmenter() :
		Segmenter()
	{
	}


	PlaneCutSegmenter::~PlaneCutSegmenter()
	{
	}

	void PlaneCutSegmenter::SetSegmentationParameters(PlaneCutSegmentationParams _segmentationParams)
	{
		DebugUtility::DbgOut(L"PlaneCutSegmenter::SetSegmentationParameters");

		PlaneCutSegmentationParams* temporaryParams = dynamic_cast<PlaneCutSegmentationParams*>(&_segmentationParams);
		if (temporaryParams != nullptr)
			segmentationParameters = *temporaryParams;
		else
			segmentationParameters = PlaneCutSegmentationParams();
	}

	bool PlaneCutSegmenter::ConvertToPointCloud(MeshContainer &_mesh)
	{
		CleanUp();
		Segmenter::ConvertToPointCloud(_mesh);
		return true;
	}

	bool PlaneCutSegmenter::UpdateSegmentation(GraphicsController& _glControl, ModelData& _modelData)
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
			return false;
		}
		return true;
	}

	bool PlaneCutSegmenter::InitializeSegmentation(ModelData& _modelData)
	{
		std::shared_ptr<MeshContainer> remainingMesh = _modelData.GetCurrentlySelectedMesh();
		if (remainingMesh == nullptr)
		{
			DebugUtility::DbgOut(L"PlaneCutSegmenter::InitializeSegmentation::No Mesh selected");
			return false;
		}
		if (!ConvertToPointCloud(*remainingMesh))
			return false;

		return true;
	}

	bool PlaneCutSegmenter::Segment()
	{
		DebugUtility::DbgOut(L"PlaneCutSegmenter::Segment()");
		temporarySegmentationClusterIndices.clear();

		pcl::PointIndices verticesAbove;
		pcl::PointIndices verticesBelow;

		Logger::WriteToLog(L"PlaneCutSegmentation with Parameters X = " + std::to_wstring(segmentationParameters.planeParameters.x)
			+ L", Y = " + std::to_wstring(segmentationParameters.planeParameters.y)
			+ L", Z = " + std::to_wstring(segmentationParameters.planeParameters.z)
			+ L", D = " + std::to_wstring(segmentationParameters.planeParameters.d), Logger::info);
		int index = 0;
		for (auto& vertex : mainCloud->points)
		{
			float iO = vertex.x*segmentationParameters.planeParameters.x + vertex.y*segmentationParameters.planeParameters.y + vertex.z * segmentationParameters.planeParameters.z - segmentationParameters.planeParameters.d;
			if (iO > 0)
				verticesAbove.indices.push_back(index);
			else
				verticesBelow.indices.push_back(index);
			index++;
		}
		temporarySegmentationClusterIndices.push_back(verticesAbove);
		temporarySegmentationClusterIndices.push_back(verticesBelow);

		Logger::WriteToLog(L"Plane cut segmentation produced " + std::to_wstring((int)verticesAbove.indices.size()) + L" above and "
			+ std::to_wstring((int)verticesAbove.indices.size()) + L"below.", Logger::info);

		return true;
	}

	void PlaneCutSegmenter::UpdateHighlights(ModelData& _modelData)
	{
		if (GetClusterCount() == 0)
			return;
		DebugUtility::DbgOut(L"UpdatePlaneCutSegmenterHighlights:: Cluster Count: ", GetClusterCount() - 1);
		std::vector<int> trianglesToBeColored = GetClusterIndices(GetClusterCount() - 1);

		_modelData.TemporarilyColorTriangles(0, trianglesToBeColored, ColorIF{ 0.5f, 0.0f, 0.0f }, true);
	}

	void PlaneCutSegmenter::FinishSegmentation(ModelData& _inputModelData, ModelData& _outputModelData)
	{
		DebugUtility::DbgOut(L"PlaneCutSegmenter::FinishSegmentation:: Cluster Count: ", GetClusterCount());

		_outputModelData.SetMeshAsDeleted(_outputModelData.GetCurrentlySelectedMeshIndex());
		for (int i = 0; i < GetClusterCount(); i++)
		{
			_outputModelData.AddObjectMeshToData(ConvertToMesh(i));
		}
	}

	void PlaneCutSegmenter::CleanUp()
	{
		Segmenter::CleanUp();
	}
}