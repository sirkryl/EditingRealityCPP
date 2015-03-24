#pragma once
#include "ObjectSegmenter.h"

namespace InteractiveFusion {

	class EuclideanSegmenter : public ObjectSegmenter
	{
	public:
		EuclideanSegmenter();
		~EuclideanSegmenter();

		virtual void SetSegmentationParameters(ObjectSegmentationParams& _segmentationParams);

		
	protected:
		EuclideanSegmentationParams segmentationParameters;

		virtual bool Segment();
	};
}

