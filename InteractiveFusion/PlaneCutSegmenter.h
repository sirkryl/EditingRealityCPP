#pragma once
#include "ObjectSegmenter.h"

namespace InteractiveFusion {

	class PlaneCutSegmenter : public Segmenter
	{
	public:
		PlaneCutSegmenter();
		~PlaneCutSegmenter();

		virtual void SetSegmentationParameters(PlaneCutSegmentationParams* _segmentationParams);

		virtual bool UpdateSegmentation(GraphicsControl* _glControl, ModelData* _modelData);

		virtual bool InitializeSegmentation(ModelData* _modelData);
		virtual void FinishSegmentation(ModelData* _inputModelData, ModelData* _outputModelData);

		void CleanUp();

	protected:
		PlaneCutSegmentationParams segmentationParameters;

		virtual bool Segment();
		virtual bool ConvertToPointCloud(MeshContainer &_mesh);
		virtual void UpdateHighlights(ModelData* _modelData);
	};
}

