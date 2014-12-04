#pragma once
#include "common.h"

enum SegmentationMode { SEGMENTATION_REGIONGROWTH, SEGMENTATION_EUCLIDEAN};

class SegmentationHelper
{
public:
	void LoadClusterData();
	
	void StartSegmentation();

	bool InitializePreview();
	bool InitializePreview(std::vector<Vertex> vertices, std::vector<Triangle> triangles);
	void RenderPreview();
	void ResetInitializedStatus();
	bool IsPreviewInitialized();

	void ClearPreviewVertices();

	bool IsWallReady();
	bool IsCloudReady();

	SegmentationMode GetSegmentationMode();
	void SetSegmentationMode(SegmentationMode mode);

	glm::vec3 GetPreviewCenterPoint();
	
	void ClearForResume();

	void CleanUp();
private:

	//vertices for helper visualizations
	std::vector<Vertex> previewVertices;
	std::vector<Triangle> previewIndices;
	//thread related
	HANDLE segmentationThread;
	DWORD sThreadId;
	SegmentationMode segMode = SEGMENTATION_EUCLIDEAN;

	//vaos and vbos for helper visualizations
	GLuint segmentVBO{ 0 }, segmentVAO{ 0 }, segmentIBO{ 0 };

	

	void GeneratePreviewBuffers();
};

extern SegmentationHelper glSegmentation;

int WINAPI SegThreadMain();