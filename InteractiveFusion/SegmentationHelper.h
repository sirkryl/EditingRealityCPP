#pragma once
#include "common.h"

enum SegmentationMode { SEGMENTATION_REGIONGROWTH, SEGMENTATION_EUCLIDEAN};
enum SegmentationState { IF_SEGSTATE_NONE, IF_SEGSTATE_PLANE_SEGMENTATION, IF_SEGSTATE_OBJECT_SEGMENTATION, IF_SEGSTATE_FINISHED};

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

	SegmentationState GetSegmentationState();
	void SetSegmentationState(SegmentationState state);
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
	SegmentationState segState = IF_SEGSTATE_NONE;

	//vaos and vbos for helper visualizations
	GLuint segmentVBO{ 0 }, segmentVAO{ 0 }, segmentIBO{ 0 };

	

	void GeneratePreviewBuffers();
};

extern SegmentationHelper glSegmentation;

int WINAPI SegThreadMain();