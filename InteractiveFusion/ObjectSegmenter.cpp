#include "ObjectSegmenter.h"
#include <random>
#include "DebugUtility.h"
#include "ModelData.h"
namespace InteractiveFusion {

	ObjectSegmenter::ObjectSegmenter() :
		Segmenter()
	{
	}


	ObjectSegmenter::~ObjectSegmenter()
	{
	}

	void ObjectSegmenter::SetSegmentationParameters(ObjectSegmentationParams& _segmentationParams)
	{
	}

	void ObjectSegmenter::FinishSegmentation(ModelData& _inputModelData, ModelData& _outputModelData)
	{
		_outputModelData.CopyPlanesFrom(_inputModelData);

		for (int i = 0; i < GetClusterCount(); i++)
		{
			_outputModelData.AddObjectMeshToData(ConvertToMesh(i));
		}
	}

	void ObjectSegmenter::UpdateHighlights(ModelData& _modelData)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.3f, 0.7f);

		int segmentedMeshIndex = _modelData.GetFirstMeshIndexThatIsNotPlane();
		DebugUtility::DbgOut(L"GraphicsControl::UpdateObjectSegmentationHighlights::segmentedMeshIndex: ", segmentedMeshIndex);
		DebugUtility::DbgOut(L"GraphicsControl::UpdateObjectSegmentationHighlights::clusterCount: ", GetClusterCount());
		for (int i = 0; i < GetClusterCount(); i++)
		{
			ColorIF color = { dis(gen), dis(gen), dis(gen) };
			std::vector<int> trianglesToBeColored = GetClusterIndices(i);
			_modelData.TemporarilyColorTriangles(segmentedMeshIndex, trianglesToBeColored, color, false);
		}
	}
}