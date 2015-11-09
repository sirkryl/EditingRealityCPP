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
	class GraphicsController
	{
	public:
		void Initialize(HWND _parentWindow, HINSTANCE _hInstance);

		void RunOpenGLThread();

		void SwitchToNewState();
		void UpdateApplicationState(WindowState _state);
		void SetScenarioType(ScenarioType type);
		bool RequestsStateChange();

		glm::mat4& GetProjectionMatrix();
		glm::mat4 GetOrthoMatrix();
		glm::mat4& GetViewMatrix();
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

		GraphicsController();

		bool IsRendering();

		void ChangeInteractionMode(InteractionMode _interactionMode);
		void ChangePlaneCutAxis(PlaneCutAxis _axis);
		void SetCameraMode(OpenGLCameraMode _cameraMode);
		void ResetModel();
		void ReloadCurrentState();
		int FillHoles(int _holeSize);
		int RemoveConnectedComponents(int _maxComponentSize);
		
		void ExportModel(ScenarioType type);

		void UpdatePlaneSegmentation(PlaneSegmentationParams& _params);
		void UpdateObjectSegmentation(ObjectSegmentationParams& _params);

		void ResetPlaneSegmentation();

		void ConfirmSegmentedPlane(PlaneSegmentationParams& _params);

		void RejectSegmentedPlane(PlaneSegmentationParams& _params);

		void FinishObjectSegmentation();

		void FinishProcessing();

		void SetScannedMesh(std::vector<Vertex>& _scannedVertices, std::vector<Triangle>& _scannedTriangles);
		

		void PushEvent(GraphicsControlEvent _event);

		bool IsBusy();
		

		void ExecutePlaneCut();
		void PlaneCutPreview();
		std::wstring GetStatusMessage();

		
		
		int GetNumberOfVertices();
		int GetNumberOfTriangles();
		int GetNumberOfVisibleModels();

		void SetCameraMovementEnabled(bool _flag);
		void ChangePlaneCutTransformation(PlaneCutTransformation _transformationMode);
		void PrepareViewportResize();
		int LoadAndSegmentModelDataFromScan(std::vector<Vertex>& _scannedVertices, std::vector<Triangle>& _scannedTriangles);

	private:
		HWND hWnd;
		HWND parentWindow;

		HINSTANCE hInstance;

		boost::thread openGLThread;
		
		int mouseWheelDelta = 0;

		bool isCameraMovementEnabled = true;
		bool isBusy = false;
		bool firstRelease = true;
		int fpsCount, currentFps;
		clock_t tLastSecond;

		std::wstring statusMessage;

		glm::mat4 mProjection;
		glm::mat4 mOrtho;
		glm::mat4 mView;

		int numberOfClusters = 0;
		int numberOfVertices = 0;
		int numberOfTriangles = 0;

		int holeSize = 0;

		

		void SetBusy(bool _isBusy);
		void SetStatusMessage(std::wstring _message);
		
		int OpenGLThreadMessageLoop();


		int PlaneSelectionThread(PlaneSegmentationParams& _params);
		
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
		void UpdateSceneInformation();

		void CleanUpShaders();
		void CleanUpRenderer();
		void CleanUpModels();
		void CleanUpSegmenter();
		void CleanUpIconData();
	};
}