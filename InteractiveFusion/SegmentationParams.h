#pragma once
#include "EnumDeclarations.h"
#include "CommonStructs.h"

namespace InteractiveFusion {
	struct ObjectSegmentationParams{
		ObjectSegmentationParams(ObjectSegmentationType _type) : type(_type) {};
		virtual ~ObjectSegmentationParams() {};

		ObjectSegmentationType GetType() { return type; };
	protected:
		ObjectSegmentationType type;

	};
	struct EuclideanSegmentationParams : ObjectSegmentationParams
	{
		EuclideanSegmentationParams() : ObjectSegmentationParams(Euclidean), clusterTolerance(0.02f) {};
		virtual ~EuclideanSegmentationParams() {};
		float clusterTolerance;
	};

	struct RegionGrowthSegmentationParams : ObjectSegmentationParams
	{
		RegionGrowthSegmentationParams() :
			ObjectSegmentationParams(RegionGrowth),
			kSearchValue(20),
			numberOfNeighbors(20),
			smoothnessThreshold(100),
			curvatureThreshold(10) {};
		virtual ~RegionGrowthSegmentationParams() {};
		int kSearchValue;
		int numberOfNeighbors;
		double smoothnessThreshold;
		double curvatureThreshold;
	};

	struct PlaneSegmentationParams
	{
		PlaneSegmentationParams() :
			planeThickness(0.1f),
			planeSmoothness(0.05f) {};
		virtual ~PlaneSegmentationParams() {};
		float planeThickness;
		float planeSmoothness;
	};

	struct PlaneCutSegmentationParams
	{
		PlaneCutSegmentationParams() :
			planeParameters(PlaneParameters()) {};
		virtual ~PlaneCutSegmentationParams() {};
		PlaneParameters planeParameters;
	};
}