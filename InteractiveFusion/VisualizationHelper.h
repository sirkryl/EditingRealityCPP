#pragma once
#include "common.h"

class VisualizationHelper
{
public:
	void FillPointsToVisualize();
	void RenderHelpingVisuals();
	void InitializeRayVisual();

	void CleanUp();
private:
	GLuint rayVBO{ 0 }, rayVAO{ 0 };
	GLuint pointVBO{ 0 }, pointVAO{ 0 };
	std::vector<float> rayVertices;
	std::vector<float> pointVertices;
};
extern VisualizationHelper glHelper;