#pragma once
#include "Segmenter.h"

namespace InteractiveFusion {
	class ObjectSegmenter :
		public Segmenter
	{
	public:
		ObjectSegmenter();
		~ObjectSegmenter();


		virtual void SetSegmentationParameters(ObjectSegmentationParams* _segmentationParams);
		virtual void FinishSegmentation(ModelData* _inputModelData, ModelData* _outputModelData);

	protected:
		virtual void UpdateHighlights(ModelData* _modelData);
	};
}

