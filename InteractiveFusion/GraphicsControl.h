#pragma once
#include "EnumDeclarations.h"
#include "SegmentationParams.h"
#include "OpenGLShaderProgram.h"
#include "CommonStructs.h"
#include <glm/glm.hpp>
#include <boost/thread.hpp>
#include <Windows.h>
#include <string>
#include <memory>
namespace InteractiveFusion {

	class MeshContainer;
	class GraphicsControl
	{
	public:
		void Initialize(HWND _parentWindow, HINSTANCE _hInstance);

		void RunOpenGLThread();

		void UpdateApplicationState(WindowState _state);

		bool RequestsStateChange();

		glm::mat4* GetProjectionMatrix();
		glm::mat4 GetOrthoMatrix();
		glm::mat4* GetViewMatrix();
		void SetCameraMatrix(glm::mat4 viewMatrix);


		void ShowOpenGLWindow();
		void ResizeOpenGLWindow(int _parentWidth, int _parentHeight);
		void SetProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar);
		void SetOrtho2D(int width, int height);

		OpenGLShaderProgram GetShader(OpenGLShaderProgramType _type);

		void CountFPS();

		void CleanUp();

		void SwapBuffers();

		float GetFramesPerSecond();

		HWND GetOpenGLWindowHandle();
		int GetViewportWidth();
		int GetViewportHeight();

		int GetMouseWheelDelta();
		void SetMouseWheelDelta(int _mouseWheelDelta);


		void ResetCamera();

		GraphicsControl();

		bool IsRendering();

		void ChangeInteractionMode(InteractionMode _interactionMode);
		void ChangePlaneCutMode(PlaneCutMode _mode);
		void SetCameraMode(OpenGLCameraMode _cameraMode);
		void ResetModel();
		int FillHoles(int _holeSize);
		int RemoveConnectedComponents(int _maxComponentSize);
		
		void ExportModel();

		void UpdatePlaneSegmentation(PlaneSegmentationParams* _params);
		void UpdateObjectSegmentation(ObjectSegmentationParams* _params);

		

		void ConfirmSegmentedPlane(PlaneSegmentationParams* _params);

		void RejectSegmentedPlane(PlaneSegmentationParams* _params);

		void FinishObjectSegmentation();

		void FinishProcessing();

		int LoadAndSegmentModelDataFromScan(std::shared_ptr<MeshContainer> _scannedMesh);

		void PushEvent(OpenGLControlEvent _event);

		bool IsBusy();
		
		void ExecutePlaneCut();
		void PlaneCutPreview();
		std::wstring GetStatusMessage();
		int GetNumberOfVertices();
		int GetNumberOfTriangles();
		int GetNumberOfVisibleModels();

		void ShowPlaneRenderer(bool _flag);
		bool IsPlaneRendererVisible();
		void PrepareViewportResize();

	private:
		HWND hWnd;
		HWND parentWindow;

		HINSTANCE hInstance;

		boost::thread openGLThread;
		
		int mouseWheelDelta = 0;

		bool isBusy = false;

		int fpsCount, currentFps;
		clock_t tLastSecond;

		std::wstring statusMessage;

		glm::mat4 mProjection;
		glm::mat4 mOrtho;
		glm::mat4 mView;

		void SetBusy(bool _isBusy);
		void SetStatusMessage(std::wstring _message);

		int OpenGLThreadMessageLoop();

		int PlaneSelectionThread(PlaneSegmentationParams* _params);
		
		int SetupOpenGL();
		void SetupRenderer();
		void SetupSegmenter();
		void SetupSceneData();
		bool SetupShaders();
		void SetupSelectors();
		void SetupIconData();
		void SetupCutPlane();
		void UpdateCamera();
		void UpdateFrame();
		void HandleEvents();
		void HandleInput();
		

		void CleanUpShaders();
		void CleanUpRenderer();
		void CleanUpModels();
		void CleanUpSegmenter();
		void CleanUpIconData();
	};
}