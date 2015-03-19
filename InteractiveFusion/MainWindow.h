#pragma once
#include "EnumDeclarations.h"
#include "SegmentationParams.h"

#include <Windows.h>
#include <string>
#include <memory>
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
		

		void ChangeState(WindowState _state);

		void ShowHelpMessage(HelpMessage _state);
		bool IsHelpVisible();
		void ChangeScanVolumeSize(int _voxelsPerMeter);

		void InitializeOpenGLScene(std::shared_ptr<MeshContainer> _scannedMesh);

		void SetPlaneRenderer(bool _flag);
		void ExecutePlaneCut();
		void ChangeInteractionMode(InteractionMode _interactionMode);
		void ChangePlaneCutMode(PlaneCutMode _mode);
		void SetCameraMode(OpenGLCameraMode _cameraMode);

		void ReloadModel();
		void ExportModel();
		void ResetModel();

		int FillHoles(int _holeSize);
		int RemoveConnectedComponents(int _maxComponentSize);
		
		void UpdateObjectSegmentation(ObjectSegmentationParams* _params);
		void UpdatePlaneSegmentation(PlaneSegmentationParams* _params);
		void ConfirmSegmentedPlane(PlaneSegmentationParams* _params);
		void RejectSegmentedPlane(PlaneSegmentationParams* _params);
		
		void FinishObjectSegmentation();
		
		void FinishProcessing();

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
		
		bool IsValidState(WindowState _state);


		void CleanUp();

	};
}