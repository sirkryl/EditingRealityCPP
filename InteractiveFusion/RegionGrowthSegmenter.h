#pragma once
#include "ObjectSegmenter.h"

namespace InteractiveFusion {
	class RegionGrowthSegmenter : public ObjectSegmenter
	{
	public:
		RegionGrowthSegmenter();
		~RegionGrowthSegmenter();

		virtual void SetSegmentationParameters(ObjectSegmentationParams& _segmentationParams);
		
	protected:
		RegionGrowthSegmentationParams segmentationParameters;

		virtual bool Segment();
	};
}
