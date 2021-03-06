#pragma once

namespace InteractiveFusion {
	class InteractionWindow;
	enum SegmentationMode { SEGMENTATION_REGIONGROWTH, SEGMENTATION_EUCLIDEAN };
	enum SegmentationState { IF_SEGSTATE_NONE, IF_SEGSTATE_PLANE_SEGMENTATION, IF_SEGSTATE_OBJECT_SEGMENTATION, IF_SEGSTATE_FINISHED };

	class SegmentationHelper
	{
	public:

		bool previewMode = false;
		bool segmentValuesChanged = false;

		void LoadClusterData();

		void StartSegmentation();

		bool InitializePreview();
		bool InitializeMinCutPreview();
		bool InitializePreview(std::vector<Vertex> vertices, std::vector<Triangle> triangles);
		//void RenderPreview();
		void ResetInitializedStatus();
		bool IsPreviewInitialized();

		void ClearPreviewVertices();

		bool IsWallReady();
		bool IsCloudReady();

		//void SetInteractionWindow(InteractionWindow* _interactionWindow);

		void SetSegmentationMesh(int index);
		void AddMinCutForegroundPoint(glm::vec3 fPoint);
		void AddMinCutBackgroundPoint(glm::vec3 bPoint);
		void DetermineMinCutRadius(glm::vec3 upperPoint, glm::vec3 centerPoint);
		void ResetMinCutValues();

		SegmentationState GetSegmentationState();
		void SetSegmentationState(SegmentationState state);
		SegmentationMode GetSegmentationMode();
		void SetSegmentationMode(SegmentationMode mode);

		glm::vec3 GetPreviewCenterPoint();

		void ClearForResume();

		void CleanUp();
	private:

		//InteractionWindow* interactionWindow;
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
}