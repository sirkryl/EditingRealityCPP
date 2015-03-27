#pragma once
#include "EnumDeclarations.h"
#include "SegmentationParams.h"

#include <Windows.h>
#include <string>
#include <memory>
#include <vector>
namespace InteractiveFusion {
	class MeshContainer;
	class MainWindow
	{
	public:

		static const DWORD          updateFpsCounterInMilliSeconds = 5000;
		void InitApplication(HINSTANCE hInstance);
		void MoveModeButtonsOnResize(int width, int height);

		void SetFramesPerSecond(float fps);
		void SetStatusBarMessage(std::wstring _statusBarMessage);
		
		void ForwardCameraMatrix(glm::mat4& matrix);

		void ChangeState(WindowState _state);

		void SetAndShowHelpMessage(HelpMessage _state);
		bool IsHelpVisible();
		bool IsHelpEnabled();
		void ChangeScanVolumeSize(int _voxelsPerMeter);

		void InitializeOpenGLScene(std::vector<Vertex>& scannedVertices, std::vector<Triangle>& scannedTriangles);

		void ChangePlaneCutTransformation(PlaneCutTransformation _transformationMode);
		void ExecutePlaneCut();
		void ChangeInteractionMode(InteractionMode _interactionMode);
		void ChangePlaneCutAxis(PlaneCutAxis _axis);
		void SetCameraMode(OpenGLCameraMode _cameraMode);
		void SetCameraMovementEnabled(bool _flag);
		void ReloadModel();
		void ExportModel();
		void ResetModel();

		int FillHoles(int _holeSize);
		int RemoveConnectedComponents(int _maxComponentSize);
		
		void ResetPlaneSegmentation();
		void UpdateObjectSegmentation(ObjectSegmentationParams& _params);
		void UpdatePlaneSegmentation(PlaneSegmentationParams& _params);
		void ConfirmSegmentedPlane(PlaneSegmentationParams& _params);
		void RejectSegmentedPlane(PlaneSegmentationParams& _params);
		
		void FinishObjectSegmentation();
		
		void FinishProcessing();
		void ToggleHelp(bool flag);
		int GetScanVolumeSize();
		int GetGpuMemory();

		void SetGpuMemory(int _gpuMemory);


		static LRESULT CALLBACK     MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		LRESULT CALLBACK MainWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	private:
		
		void ShutdownWindow();
		void ProcessUI(WPARAM wParam, LPARAM lParam);

		void UpdateModeButtons();
		void UpdateStatusBarMessage();
		void UpdateUIActivation();
		void UpdateOpenGLControl();
		void UpdateSceneInformation();
		

		static const DWORD          statusTimeOutInMilliSeconds = 5000;
		bool isHelpEnabled = false;
		bool IsValidState(WindowState _state);

		void ShowHelp();
		void CleanUp();

	};
}