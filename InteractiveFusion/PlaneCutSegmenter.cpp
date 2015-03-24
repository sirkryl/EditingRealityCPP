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

	bool PlaneCutSegmenter::UpdateSegmentation(GraphicsControl& _glControl, ModelData& _modelData)
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

		DebugUtility::DbgOut(L"PlaneCutSegmenter::Segment()::verticesAbove: ", (int)verticesAbove.indices.size());
		DebugUtility::DbgOut(L"PlaneCutSegmenter::Segment()::verticesBelow: ", (int)verticesBelow.indices.size());

		return true;
	}

	void PlaneCutSegmenter::UpdateHighlights(ModelData& _modelData)
	{
		if (GetClusterCount() == 0)
			return;
		DebugUtility::DbgOut(L"UpdatePlaneCutSegmenterHighlights:: ", GetClusterCount() - 1);
		std::vector<int> trianglesToBeColored = GetClusterIndices(GetClusterCount() - 1);

		_modelData.TemporarilyColorTriangles(0, trianglesToBeColored, ColorIF{ 0.5f, 0.0f, 0.0f }, true);
	}

	void PlaneCutSegmenter::FinishSegmentation(ModelData& _inputModelData, ModelData& _outputModelData)
	{
		DebugUtility::DbgOut(L"PlaneCutSegmenter::FinishSegmentation:: ", GetClusterCount());

		_outputModelData.SetMeshAsDeleted(_outputModelData.GetCurrentlySelectedMeshIndex());
		for (int i = 0; i < GetClusterCount(); i++)
		{
			_outputModelData.AddObjectMeshToData(ConvertToMesh(i));
		}
	}

	void PlaneCutSegmenter::CleanUp()
	{
		DebugUtility::DbgOut(L"PlaneCutSegmenter::CleanUp()");
		Segmenter::CleanUp();
	}
}