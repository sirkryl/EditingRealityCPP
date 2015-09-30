#pragma once
#include "ImageRenderer.h"
#include <NuiSensorChooserUI.h>
#include "KinectFusionParams.h"
#include "KinectFusionProcessor.h"
#include "IScanner.h"
namespace InteractiveFusion {
	class KinectFusion : public IScanner
	{
	public:
		KinectFusion();
		~KinectFusion();

		void Initialize(HWND _parentHandle, HWND _depthView, HWND _reconstructionView, HWND _trackingResidualsView);

		void HandleCompletedFrame();

		

		void UpdateSensorStatus(WPARAM _wParam);

		void ResolvePotentialSensorConflict(LPARAM _lParam);

		void ChangeVolumeSize(int _voxelsPerMeter);

		float GetFramesPerSecond();
		int GetGpuMemory();

		glm::mat4 GetTransformedCameraMatrix();

		std::wstring GetAndResetStatus();

		void PrepareMeshSave();
		std::vector<Vertex>& GetScannedVertices();

		std::vector<Triangle>& GetScannedTriangles();
		void FinishMeshSave();

		void ResetScan();

		void PauseRendering();
		void PauseIntegration();
		void PauseProcessing(bool flag);
		void ResumeRendering();

		bool IsReconstructionReady();

		void CleanUp();

	private:

		NuiSensorChooserUI*         m_pSensorChooserUI;

		ImageRenderer              m_pDrawReconstruction;
		ImageRenderer              m_pDrawTrackingResiduals;
		ImageRenderer              m_pDrawDepth;

		ID2D1Factory*               m_pD2DFactory;

		KinectFusionParams      m_params;
		KinectFusionProcessor       m_processor;

		KinectFusionMeshTypes       m_saveMeshFormat;

		bool                        m_bInitializeError = false;
		bool                        m_bSavingMesh = false;
		bool                        m_bUIUpdated = false;
		bool                        m_bColorCaptured = false;
		bool                        pauseRendering = false;

		bool reconstructionReady = false;

		std::vector<Vertex> scannedVertices;
		std::vector<Triangle> scannedTriangles;
		void SetStatus(std::wstring _status);
	};

}