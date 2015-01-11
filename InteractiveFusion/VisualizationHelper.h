#pragma once
#include "common.h"

enum VisualizationMode { IF_VISUALIZATION_NONE, IF_VISUALIZATION_PLANE};

class VisualizationHelper
{
public:
	void FillPointsToVisualize();
	void RenderHelpingVisuals();
	void InitializeRayVisual();
	bool IsRayInitialized();
	void CleanUp();

	bool IsPlaneInitialized();
	void InitializePlane(glm::vec3 lowerBounds, glm::vec3 upperBounds);
	void DrawPlane();
	void SetPlaneTranslation(glm::vec3 point);
	VisualizationMode GetVisualizationMode();
	void SetVisualizationMode(VisualizationMode _visMode);
private:

	VisualizationMode visMode = IF_VISUALIZATION_NONE;

	GLuint rayVBO{ 0 }, rayVAO{ 0 };
	GLuint pointVBO{ 0 }, pointVAO{ 0 };
	GLuint planeVBO{ 0 }, planeVAO{ 0 };
	std::vector<Vertex> planeVertices;

	glm::mat4 planeTranslation;

	std::vector<float> rayVertices;
	std::vector<float> pointVertices;

};
extern VisualizationHelper glHelper;