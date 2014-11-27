#pragma once
#include "common.h"

class SegmentationHelper
{
public:
	void LoadClusterData();

	void StartSegmentation();

	bool InitializePreview();

	void RenderPreview();

	bool IsPreviewInitialized();

	void ClearPreviewVertices();

	bool IsCloudReady();

	glm::vec3 GetPreviewCenterPoint();

	void ClearForResume();

	void CleanUp();
private:

	//vertices for helper visualizations
	std::vector<Vertex> previewVertices;
	//thread related
	HANDLE segmentationThread;
	DWORD sThreadId;

	//vaos and vbos for helper visualizations
	GLuint segmentVBO{ 0 }, segmentVAO{ 0 };

	

	void GeneratePreviewBuffers();
};

extern SegmentationHelper glSegmentation;

int WINAPI SegThreadMain();