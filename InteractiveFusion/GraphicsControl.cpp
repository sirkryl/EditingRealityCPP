#include "GraphicsControl.h"

#include "IFResources.h"

#include "DebugUtility.h"
#include "KeyState.h"
#include "StyleSheet.h"
#include "StopWatch.h"
#include <queue>
#include <unordered_map>

#include <gl/glew.h>
#include <gl/wglew.h>
#include <glm/gtc/matrix_transform.hpp>

#include "ModelData.h"
#include "IconData.h"

#include "GraphicsCamera.h"
#include "OpenGLWindow.h"

#include "InteractionRenderer.h"
#include "PlaneSelectionRenderer.h"
#include "ProcessingRenderer.h"
#include "SegmentationRenderer.h"
#include "PlaneCutRenderer.h"

#include "SimplePlaneRenderable3D.h"

#include "EuclideanSegmenter.h"
#include "PlaneSegmenter.h"
#include "RegionGrowthSegmenter.h"
#include "PlaneCutSegmenter.h"
#include "ColorSelector.h"
#include "DuplicateSelector.h"
#include "ManipulationSelector.h"
#include "PlaneSelector.h"
#include "TransformSelector.h"

#include "DialogExporter.h"
#include "ScenarioExporter.h"
#include "OpenGLShaderProgram.h"

#include "OpenGLContext.h"
#include "MeshContainer.h"

using namespace std;

namespace InteractiveFusion {

#pragma region

	

	WindowState currentApplicationState;

	std::unordered_map<WindowState, unique_ptr<OpenGLRenderer>> rendererMap;
	std::unordered_map<WindowState, unique_ptr<OpenGLRenderer>>::const_iterator activeRenderer = rendererMap.end();

	std::unordered_map<WindowState, unique_ptr<ModelData>> sceneMap;
	std::unordered_map<WindowState, unique_ptr<ModelData>>::const_iterator activeScene = sceneMap.end();
	std::unordered_map<WindowState, unique_ptr<IconData>> iconMap;
	std::unordered_map<WindowState, unique_ptr<IconData>>::const_iterator activeIcons = iconMap.end();

	ModelData temporarySceneData;
	ModelData temporaryPlaneCutDataForReset;
	InteractionMode interactionMode = InteractionMode::None;
	OpenGLCameraMode currentCameraMode = OpenGLCameraMode::Free;
	ObjectSegmentationType currentSegmentationType;

	std::unordered_map<ObjectSegmentationType, unique_ptr<ObjectSegmenter>> segmenterMap;

	PlaneSegmenter planeSegmenter;

	std::shared_ptr<SimplePlaneRenderable3D> cutPlane;

	std::unordered_map<InteractionMode, unique_ptr<Selector>> selectorMap;
	ColorSelector colorSelector;
	unique_ptr<PlaneSelector> planeSelector;

	OpenGLContext glContext;

	std::vector<Vertex> scannedVertices;
	std::vector<Triangle> scannedTriangles;
	std::shared_ptr<MeshContainer> loadedMesh;

	std::unordered_map<OpenGLShaderProgramType, OpenGLShaderProgram> shaderMap;
	//OpenGLShaderProgram defaultMaterial;
	//OpenGLShaderProgram orthoMaterial;

	queue<GraphicsControlEvent> eventQueue;

	GraphicsCamera glCamera;
	OpenGLWindow openGLWindow;

	bool quitMessageLoop = false;
	bool requestStateChange = false;
	bool showPlane = false;

	int parentWindowHeight;
	int parentWindowWidth;

#pragma endregion variables

#pragma region
	GraphicsControl::GraphicsControl()
	{
		fpsCount = 0;
		currentFps = 0;
	}
#pragma endregion Constructors

	void GraphicsControl::Initialize(HWND _parentWindow, HINSTANCE _hInstance)
	{
		parentWindow = _parentWindow;
		hInstance = _hInstance;

		RECT parentRect;
		GetClientRect(parentWindow, &parentRect);
		parentWindowWidth = parentRect.right;
		parentWindowHeight = parentRect.bottom;
	}

	void GraphicsControl::SwitchToNewState()
	{
		activeRenderer = rendererMap.find(currentApplicationState);
		activeScene = sceneMap.find(currentApplicationState);
		activeIcons = iconMap.find(currentApplicationState);
		if (activeRenderer != rendererMap.end())
		{
			currentApplicationState = currentApplicationState;
			if (currentApplicationState == WindowState::PlaneSelection)
				openGLWindow.SetMargins(0.07f, 0.042f, 0, 0);
			else
				openGLWindow.SetMargins(0.07f, 0.042f, 0.2f, 0);

			if (!temporarySceneData.IsEmpty())
			{
				eventQueue.push(GraphicsControlEvent::CopyTemporaryInNextStateModelData);
				if (currentApplicationState == WindowState::Segmentation)
					eventQueue.push(GraphicsControlEvent::UpdateSegmentation);
				//else 
				//if (currentApplicationState == Processing)
				//	eventQueue.push(UpdateHoleFilling);
			}
			if (currentApplicationState == WindowState::PlaneCut)
				eventQueue.push(GraphicsControlEvent::UpdateCutPlane);
			SetCameraMode(activeRenderer->second->GetCameraMode());

			//sceneMap[currentApplicationState]->UnselectMesh();

			openGLWindow.Show();
		}
		else
		{
			openGLWindow.Hide();
			currentApplicationState = currentApplicationState;
		}
	}

	void GraphicsControl::UpdateApplicationState(WindowState _state)
	{
		currentApplicationState = _state;
		eventQueue.push(GraphicsControlEvent::StateUpdate);
	}

	bool GraphicsControl::RequestsStateChange()
	{
		if (requestStateChange)
		{
			bool request = requestStateChange;
			requestStateChange = false;
			return request;
		}
		return requestStateChange;
	}


#pragma region

	void GraphicsControl::RunOpenGLThread()
	{
		openGLThread = boost::thread(&GraphicsControl::OpenGLThreadMessageLoop, this);
	}

	int GraphicsControl::OpenGLThreadMessageLoop()
	{
		if (SetupOpenGL() == -1)
		{
			DebugUtility::DbgOut(L"GraphicsControl::OpenGLThreadMessageLoop::ERROR::Could not setup OpenGL.");
			PostQuitMessage(0);
			return -1;
		}

		if (!SetupShaders())
		{
			DebugUtility::DbgOut(L"GraphicsControl::OpenGLThreadMessageLoop::ERROR::Could not create shaders.");
			PostQuitMessage(0);
			return -1;
		}
		SetupIconData();
		SetupRenderer();
		SetupSceneData();
		SetupSegmenter();
		SetupSelectors();
		DebugUtility::DbgOut(L"GraphicsControl::OpenGLThreadMessageLoop::After SETUP");
		MSG       msg = { 0 };
		while (WM_QUIT != msg.message && !quitMessageLoop)
		{
			while (PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE) > 0)
			{
				if (msg.message == WM_LBUTTONUP)
				{
					if (firstRelease)
					{
						firstRelease = false;
						break;
					}
					else
						firstRelease = true;
				}
				GetMessage(&msg, NULL, 0, 0);
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			HandleEvents();
			if (openGLWindow.IsVisible())
			{
				openGLWindow.HandleEvents(*this);
				UpdateFrame();
				HandleInput();
				CountFPS();
				UpdateSceneInformation();
			}
			
		}
		if (quitMessageLoop)
		{
			openGLWindow.CleanUp();
			CleanUpRenderer();
			CleanUpModels();
			CleanUpSegmenter();
			CleanUpIconData();
			CleanUpShaders();
			glContext.CleanUp();
		}
		return 0;
	}

	void GraphicsControl::UpdateFrame()
	{
		if (openGLWindow.IsVisible())
		{
			
			RECT rRect;
			GetClientRect(parentWindow, &rRect);

			//ResizeOpenGLWindow(rRect.right, rRect.bottom);

			UpdateCamera();

			if (activeRenderer != rendererMap.end() && activeScene != sceneMap.end() && activeIcons != iconMap.end())
			{
				activeRenderer->second->Render(*this, *activeScene->second.get(), *activeIcons->second.get());
			}
		}
	}

	void GraphicsControl::HandleEvents()
	{
		while (!eventQueue.empty())
		{
			
			GraphicsControlEvent event = eventQueue.front();

			switch (event)
			{
			case GraphicsControlEvent::ResizeOpenGLViewport:
				RECT parentRect;
				GetClientRect(parentWindow, &parentRect);
				ResizeOpenGLWindow(parentRect.right, parentRect.bottom);
				break;
			case GraphicsControlEvent::ModelDataUpdated:
				activeScene->second->GenerateBuffers();
				break;
			case GraphicsControlEvent::ModelHighlightsUpdated:
				activeScene->second->SwapToHighlightBuffers();
				break;
			case GraphicsControlEvent::RemoveModelHighlights:
				activeScene->second->GenerateBuffers();
				break;
			case GraphicsControlEvent::RemoveModelData:
				activeScene->second->CleanUp();
				break;
			case GraphicsControlEvent::FillHolesInScene:
			{
				SetBusy(true);

				int closedHoles = activeScene->second->FillHoles(holeSize);
				eventQueue.push(GraphicsControlEvent::ModelDataUpdated);
				SetBusy(false);
			}
			break;
			case GraphicsControlEvent::InitialLoading:
			{
				SetBusy(true);
				sceneMap[WindowState::PlaneSelection]->CleanUp();
				//boost::thread(&GraphicsControl::LoadAndSegmentModelDataFromScan, this);
				break;
			}
			case GraphicsControlEvent::FinishSegmentation:
				SetBusy(true);
				temporaryPlaneCutDataForReset.CopyFrom(temporarySceneData);
				SetBusy(false);
				break;
			case GraphicsControlEvent::CopyTemporaryInNextStateModelData:
				SetBusy(true);
				activeScene->second->CleanUp();
				activeScene->second->CopyFrom(temporarySceneData);
				temporarySceneData.CleanUp();
				activeScene->second->GenerateBuffers();

				SetBusy(false);
				break;
			case GraphicsControlEvent::ResetCurrentStateModelData:
				SetBusy(true);
				if (currentApplicationState > WindowState::PlaneSelection)
				{
					WindowState previousApplicationState = (WindowState)((int)currentApplicationState - 1);
					activeScene->second->CleanUp();
					if (currentApplicationState == WindowState::PlaneCut)
					{
						activeScene->second->CopyFrom(temporaryPlaneCutDataForReset);
						DebugUtility::DbgOut(L"YES I AM HERE");
					}
					else
						activeScene->second->CopyFrom(*sceneMap[previousApplicationState].get());
					activeScene->second->GenerateBuffers();
				}
				SetBusy(false);
				break;
			case GraphicsControlEvent::StateUpdate:
				SwitchToNewState();
				break;
			case GraphicsControlEvent::ResetModelData:
				activeScene->second->ResetToInitialState();
				activeScene->second->GenerateBuffers();
				break;
			case GraphicsControlEvent::UpdateSegmentation:
				boost::thread(&GraphicsControl::UpdateObjectSegmentation, this, EuclideanSegmentationParams());
				break;
			case GraphicsControlEvent::UpdateHoleFilling:
				boost::thread(&GraphicsControl::FillHoles, this, 100000);
				break;
			case GraphicsControlEvent::UpdateCutPlane:
				SetupCutPlane();
				break;
			case GraphicsControlEvent::SetupCutPlaneMode:
				SetupCutPlane();
				planeSelector->ApplyModeChange();
				PlaneCutPreview();
				break;
			}
			eventQueue.pop();
		}
	}

	void GraphicsControl::HandleInput()
	{
		if (KeyState::GetKeyStateOnce('C'))
		{
			if (currentCameraMode == OpenGLCameraMode::Sensor)
				SetCameraMode(OpenGLCameraMode::Free);
			else
				SetCameraMode(OpenGLCameraMode::Sensor);
		}
		if (openGLWindow.IsCursorInWindow() && !isBusy && activeScene != sceneMap.end())
		{
			if (currentApplicationState == WindowState::Interaction && currentCameraMode == OpenGLCameraMode::Sensor)
				selectorMap[interactionMode]->HandleSelection(*this, *activeScene->second.get(), *activeIcons->second.get());
			else if (currentApplicationState == WindowState::Processing)
				colorSelector.HandleSelection(*this, *activeScene->second.get(), *activeIcons->second.get());
			else if (currentApplicationState == WindowState::PlaneCut && !isCameraMovementEnabled)
				planeSelector->HandleSelection(*this, *activeScene->second.get(), *activeIcons->second.get());
		}
	}

	void GraphicsControl::SetupRenderer()
	{
		cutPlane = std::shared_ptr<SimplePlaneRenderable3D>(new SimplePlaneRenderable3D());

		RECT parentRect;
		GetClientRect(parentWindow, &parentRect);
		ResizeOpenGLWindow(parentRect.right, parentRect.bottom);

		rendererMap[WindowState::PlaneSelection] = unique_ptr<PlaneSelectionRenderer>(new PlaneSelectionRenderer(OpenGLCameraMode::Free));
		rendererMap[WindowState::PlaneSelection]->Initialize(*this);
		rendererMap[WindowState::Segmentation] = unique_ptr<SegmentationRenderer>(new SegmentationRenderer(OpenGLCameraMode::Free));
		rendererMap[WindowState::Segmentation]->Initialize(*this);
		rendererMap[WindowState::PlaneCut] = unique_ptr<PlaneCutRenderer>(new PlaneCutRenderer(OpenGLCameraMode::Free, cutPlane));
		rendererMap[WindowState::PlaneCut]->Initialize(*this);
		rendererMap[WindowState::Processing] = unique_ptr<ProcessingRenderer>(new ProcessingRenderer(OpenGLCameraMode::Free));
		rendererMap[WindowState::Processing]->Initialize(*this);
		rendererMap[WindowState::Interaction] = unique_ptr<InteractionRenderer>(new InteractionRenderer(OpenGLCameraMode::Sensor));
		rendererMap[WindowState::Interaction]->Initialize(*this);


	}

	void GraphicsControl::SetupSceneData()
	{
		sceneMap[WindowState::PlaneSelection] = unique_ptr<ModelData>(new ModelData());
		sceneMap[WindowState::PlaneSelection]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Default]);
		sceneMap[WindowState::Segmentation] = unique_ptr<ModelData>(new ModelData());
		sceneMap[WindowState::Segmentation]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Default]);
		sceneMap[WindowState::PlaneCut] = unique_ptr<ModelData>(new ModelData());
		sceneMap[WindowState::PlaneCut]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Default]);
		sceneMap[WindowState::Processing] = unique_ptr<ModelData>(new ModelData());
		sceneMap[WindowState::Processing]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Default]);
		sceneMap[WindowState::Interaction] = unique_ptr<ModelData>(new ModelData());
		sceneMap[WindowState::Interaction]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Default]);


	}

	bool GraphicsControl::SetupShaders()
	{
		OpenGLShader colorVertexShader;
		colorVertexShader.LoadShader("data\\shaders\\color.vert", GL_VERTEX_SHADER);
		OpenGLShader colorFragmentShader;
		colorFragmentShader.LoadShader("data\\shaders\\color.frag", GL_FRAGMENT_SHADER);

		shaderMap[OpenGLShaderProgramType::Default].CreateProgram();
		shaderMap[OpenGLShaderProgramType::Default].AddShaderToProgram(colorVertexShader);
		shaderMap[OpenGLShaderProgramType::Default].AddShaderToProgram(colorFragmentShader);
		if (!shaderMap[OpenGLShaderProgramType::Default].LinkProgram())
			return false;

		OpenGLShader vertexShader2d;
		vertexShader2d.LoadShader("data\\shaders\\shader2d.vert", GL_VERTEX_SHADER);
		OpenGLShader fragmentShader2d;
		fragmentShader2d.LoadShader("data\\shaders\\shader2d.frag", GL_FRAGMENT_SHADER);

		shaderMap[OpenGLShaderProgramType::Orthographic].CreateProgram();
		shaderMap[OpenGLShaderProgramType::Orthographic].AddShaderToProgram(vertexShader2d);
		shaderMap[OpenGLShaderProgramType::Orthographic].AddShaderToProgram(fragmentShader2d);
		if (!shaderMap[OpenGLShaderProgramType::Orthographic].LinkProgram())
			return false;

		return true;
	}

	void GraphicsControl::SetupSegmenter()
	{
		segmenterMap[ObjectSegmentationType::Euclidean] = unique_ptr<EuclideanSegmenter>(new EuclideanSegmenter());
		segmenterMap[ObjectSegmentationType::RegionGrowth] = unique_ptr<RegionGrowthSegmenter>(new RegionGrowthSegmenter());
		//planeSegmenter = new PlaneSegmenter();
	}

	void GraphicsControl::SetupSelectors()
	{
		selectorMap[InteractionMode::Transformation] = unique_ptr<TransformSelector>(new TransformSelector());
		selectorMap[InteractionMode::Duplication] = unique_ptr<DuplicateSelector>(new DuplicateSelector());
		selectorMap[InteractionMode::None] = unique_ptr<ManipulationSelector>(new ManipulationSelector());
		planeSelector = unique_ptr<PlaneSelector>(new PlaneSelector(cutPlane));
	}

	void GraphicsControl::SetupIconData()
	{
		iconMap[WindowState::PlaneSelection] = unique_ptr<IconData>(new IconData());
		iconMap[WindowState::PlaneSelection]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Orthographic]);
		iconMap[WindowState::Segmentation] = unique_ptr<IconData>(new IconData());
		iconMap[WindowState::Segmentation]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Orthographic]);
		iconMap[WindowState::PlaneCut] = unique_ptr<IconData>(new IconData());
		iconMap[WindowState::PlaneCut]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Orthographic]);
		iconMap[WindowState::Processing] = unique_ptr<IconData>(new IconData());
		iconMap[WindowState::Processing]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Orthographic]);
		iconMap[WindowState::Interaction] = unique_ptr<IconData>(new IconData());
		iconMap[WindowState::Interaction]->SetDefaultShaderProgram(shaderMap[OpenGLShaderProgramType::Orthographic]);
		iconMap[WindowState::Interaction]->LoadFromFile("data\\models\\trash.ply", "data\\models\\trash_open.ply", TRASH_BIN);
		iconMap[WindowState::Interaction]->GenerateBuffers();
	}

	void GraphicsControl::SetupCutPlane()
	{

		glm::vec3 upperBounds = activeScene->second->GetUpperBounds();
		glm::vec3 lowerBounds = activeScene->second->GetLowerBounds();


		Vertex vertex1
		{
			upperBounds.x, 0.0f, lowerBounds.z,
			0.8f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f
		};
		Vertex vertex2
		{
			upperBounds.x, 0.0f, upperBounds.z,
			0.8f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f
		};
		Vertex vertex3
		{
			lowerBounds.x, 0.0f, lowerBounds.z,
			0.8f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f
		};
		Vertex vertex4
		{
			lowerBounds.x, 0.0f, upperBounds.z,
			0.8f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f
		};

		std::vector<Vertex> planeVertices;
		planeVertices.push_back(vertex1);
		planeVertices.push_back(vertex3);
		planeVertices.push_back(vertex4);
		planeVertices.push_back(vertex2);
		planeVertices.push_back(vertex1);
		planeVertices.push_back(vertex4);

		cutPlane->SetVertices(planeVertices);
		cutPlane->SetShaderProgram(shaderMap[OpenGLShaderProgramType::Default]);

		cutPlane->UpdateEssentials();
		cutPlane->ApplyTransformation(activeScene->second->GetNegativeGroundAlignmentRotation(), glm::mat4(1.0f));
		//cutPlane->ApplyTransformation(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -cutPlane->GetCenterPoint().y, 0.0f)), glm::mat4(1.0f));
		//_glControl.SetPlaneCutParameters(plane->GetPlaneParameters());
		cutPlane->GenerateBuffers();

	}

#pragma endregion Main OpenGL Message Loop

#pragma region

	void GraphicsControl::ShowOpenGLWindow()
	{
		openGLWindow.Show();
	}

	void GraphicsControl::ResizeOpenGLWindow(int _parentWidth, int _parentHeight)
	{
		openGLWindow.Resize(_parentWidth, _parentHeight);
		float ratio = (float)openGLWindow.GetWidth() / (float)openGLWindow.GetHeight();
		SetProjection3D(45.0f, ratio, 0.1f, 1000.0f);
		SetOrtho2D(openGLWindow.GetWidth(), openGLWindow.GetHeight());
	}

	void GraphicsControl::SetCameraMovementEnabled(bool _flag)
	{
		isCameraMovementEnabled = _flag;
		if (currentApplicationState == WindowState::PlaneCut)
		{
			planeSelector->ResetPlaneRotation();
			PlaneCutPreview();
		}
	}

	void GraphicsControl::ChangePlaneCutTransformation(PlaneCutTransformation _transformationMode)
	{
		planeSelector->ChangeTransformation(_transformationMode);
	}


	void GraphicsControl::PrepareViewportResize()
	{
		eventQueue.push(GraphicsControlEvent::ResizeOpenGLViewport);
	}

	HWND GraphicsControl::GetOpenGLWindowHandle()
	{
		return openGLWindow.GetHandle();
	}

	int GraphicsControl::GetViewportWidth()
	{
		return openGLWindow.GetWidth();
	}

	int GraphicsControl::GetViewportHeight()
	{
		return openGLWindow.GetHeight();
	}

	void GraphicsControl::PushEvent(GraphicsControlEvent _event)
	{
		eventQueue.push(_event);
	}

#pragma endregion OpenGL Window

	void GraphicsControl::SwapBuffers()
	{
		glContext.SwapBuffers();
	}

	void GraphicsControl::CountFPS()
	{
		clock_t tCurrent = clock();
		if ((tCurrent - tLastSecond) >= CLOCKS_PER_SEC)
		{
			tLastSecond += CLOCKS_PER_SEC;
			fpsCount = currentFps;
			currentFps = 0;
		}
		currentFps++;
	}

	bool GraphicsControl::IsBusy()
	{
		return isBusy;
	}

	float GraphicsControl::GetFramesPerSecond()
	{
		return (float)fpsCount;
	}

	int GraphicsControl::GetMouseWheelDelta()
	{
		return mouseWheelDelta;
	}

	void GraphicsControl::SetMouseWheelDelta(int _mouseWheelDelta)
	{
		mouseWheelDelta = _mouseWheelDelta;
	}

	void GraphicsControl::ResetCamera()
	{
		glCamera.ResetCameraPosition();
	}

	void GraphicsControl::SetBusy(bool _isBusy)
	{
		isBusy = _isBusy;
	}

	void GraphicsControl::SetStatusMessage(wstring _message)
	{
		statusMessage = _message;
	}

	void GraphicsControl::ExecutePlaneCut()
	{
		//DebugUtility::DbgOut(L"GraphicsControl::ExecutePlaneCut");

		if (activeScene->second->GetCurrentlySelectedMeshIndex() != -1)
		{
			SetStatusMessage(L"Cutting object in two");
			SetBusy(true);

			PlaneCutSegmentationParams planeCutSegmentationParameters;
			planeCutSegmentationParameters.planeParameters = cutPlane->GetPlaneParameters();

			PlaneCutSegmenter planeCutSegmenter;
			planeCutSegmenter.SetSegmentationParameters(planeCutSegmentationParameters);

			planeCutSegmenter.UpdateSegmentation(*this, *activeScene->second.get());

			planeCutSegmenter.FinishSegmentation(*activeScene->second.get(), *activeScene->second.get());

			planeCutSegmenter.CleanUp();

			eventQueue.push(GraphicsControlEvent::ModelDataUpdated);
			SetBusy(false);
		}
	}

	void GraphicsControl::PlaneCutPreview()
	{
		SetBusy(true);
		SetStatusMessage(L"Showing plane cut preview");
		//DebugUtility::DbgOut(L"GraphicsControl::PlaneCutPreview");
		if (activeScene->second->GetCurrentlySelectedMeshIndex() != -1)
		{
			activeScene->second->PlaneCutPreview(activeScene->second->GetCurrentlySelectedMeshIndex(), cutPlane->GetPlaneParameters());
			eventQueue.push(GraphicsControlEvent::ModelHighlightsUpdated);
		}
		SetBusy(false);
	}

	wstring GraphicsControl::GetStatusMessage()
	{
		return statusMessage;
	}

	void GraphicsControl::UpdateCamera()
	{
		glCamera.Update(openGLWindow.IsCursorInWindow() && isCameraMovementEnabled, mouseWheelDelta);
		mouseWheelDelta = 0;
	}

#pragma region 

	void GraphicsControl::SetProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar)
	{
		mProjection = glm::perspective(fFOV, fAspectRatio, fNear, fFar);
	}

	void GraphicsControl::SetOrtho2D(int width, int height)
	{
		mOrtho = glm::ortho(0.0f, float(width), 0.0f, float(height), 0.1f, 1000.0f);
	}

	glm::mat4& GraphicsControl::GetProjectionMatrix()
	{
		return mProjection;
	}

	glm::mat4 GraphicsControl::GetOrthoMatrix()
	{
		return mOrtho;
	}

	void GraphicsControl::SetCameraMatrix(glm::mat4 viewMatrix)
	{
		mView = viewMatrix;

	}

	glm::mat4& GraphicsControl::GetViewMatrix()
	{
		if (currentCameraMode == OpenGLCameraMode::Sensor)
			return mView;
		else
			return glCamera.GetViewMatrix();
	}


#pragma endregion Get/Set Matrices

	OpenGLShaderProgram GraphicsControl::GetShader(OpenGLShaderProgramType _type)
	{
		return shaderMap[_type];
	}

#pragma region

	void GraphicsControl::ChangeInteractionMode(InteractionMode _interactionMode)
	{
		interactionMode = _interactionMode;
		activeScene->second->RemoveTemporaryMeshColor();
		activeScene->second->UnselectMesh();
	}

	void GraphicsControl::ChangePlaneCutAxis(PlaneCutAxis _axis)
	{
		planeSelector->ChangeAxis(_axis);
		eventQueue.push(GraphicsControlEvent::SetupCutPlaneMode);
	}

	void GraphicsControl::SetCameraMode(OpenGLCameraMode _cameraMode)
	{
		currentCameraMode = _cameraMode;
		if (currentApplicationState == WindowState::PlaneCut)
			isCameraMovementEnabled = false;
		else
			isCameraMovementEnabled = true;

		if (activeRenderer != rendererMap.end())
			activeRenderer->second->SetCameraMode(currentCameraMode);

		if (currentCameraMode == OpenGLCameraMode::Free)
			openGLWindow.ShowButtons();
		else
			openGLWindow.HideButtons();
	}

	bool GraphicsControl::IsRendering()
	{
		return openGLWindow.IsVisible();
	}

	void GraphicsControl::ResetModel()
	{
		eventQueue.push(GraphicsControlEvent::ResetModelData);
	}

	void GraphicsControl::ReloadCurrentState()
	{
		eventQueue.push(GraphicsControlEvent::ResetCurrentStateModelData);
	}

	int GraphicsControl::FillHoles(int _holeSize)
	{
		
		SetStatusMessage(L"Filling holes");
		SetBusy(true);
		
		holeSize = _holeSize;
		//boost::lock_guard<ModelData> guard(*activeScene->second.get());
		activeScene->second.get()->Lock();
		int closedHoles = activeScene->second->FillHoles(_holeSize);
		activeScene->second.get()->Unlock();
		eventQueue.push(GraphicsControlEvent::ModelDataUpdated);
		//eventQueue.push(FillHolesInScene);
		//int closedHoles = sceneMap[currentApplicationState]->FillHoles(_holeSize);
		//eventQueue.push(ModelDataUpdated);

		SetBusy(false);
		return closedHoles;
		//return closedHoles;

	}

	int GraphicsControl::RemoveConnectedComponents(int _maxComponentSize)
	{
		
		SetStatusMessage(L"Removing components");
		SetBusy(true);
		activeScene->second.get()->Lock();
		int removedComponents = activeScene->second->RemoveConnectedComponents(_maxComponentSize);
		activeScene->second.get()->Unlock();
		eventQueue.push(GraphicsControlEvent::ModelDataUpdated);

		SetBusy(false);
		return removedComponents;
	}



	void GraphicsControl::ExportModel(ScenarioType type)
	{
		activeScene->second.get()->Lock();
		SetStatusMessage(L"Exporting");
		SetBusy(true);

		if (type != ScenarioType::None)
		{
			DebugUtility::DbgOut(L"GraphicsControl::ExportModel::Scenario Exporter");
			ScenarioExporter exporter;
			exporter.Export(*activeScene->second.get(), type);
		}
		else
		{
			DebugUtility::DbgOut(L"GraphicsControl::ExportModel::Dialog Exporter");
			DialogExporter exporter;
			if (activeScene->second->GetCurrentlySelectedMeshIndex() != -1)
			{
				DebugUtility::DbgOut(L"GraphicsControl::ExportModel::Selected Mesh");
				exporter.Export(*activeScene->second.get(), activeScene->second->GetCurrentlySelectedMeshIndex());
			}
			else
			{
				DebugUtility::DbgOut(L"GraphicsControl::ExportModel::Whole Mesh");
				exporter.Export(*activeScene->second.get());
			}
		}

		SetBusy(false);
		activeScene->second.get()->Unlock();
	}

	void GraphicsControl::FinishProcessing()
	{
		SetStatusMessage(L"Finishing processing");
		SetBusy(true);

		temporarySceneData.CopyFrom(*activeScene->second.get());

		SetBusy(false);
	}

	void GraphicsControl::UpdateSceneInformation()
	{

		if (activeScene != sceneMap.end())
		{
			activeScene->second.get()->TryLock();
			numberOfVertices = activeScene->second->GetNumberOfVertices();
			numberOfTriangles = activeScene->second->GetNumberOfTriangles();
			numberOfClusters = activeScene->second->GetVisibleMeshCount();
			activeScene->second.get()->Unlock();
		}
	}

	int GraphicsControl::GetNumberOfVertices()
	{
		return numberOfVertices;

	}

	int GraphicsControl::GetNumberOfTriangles()
	{
		return numberOfTriangles;
	}

	int GraphicsControl::GetNumberOfVisibleModels()
	{
		return numberOfClusters;
	}

#pragma endregion Processing

#pragma region

	void GraphicsControl::SetScannedMesh(std::vector<Vertex>& _scannedVertices, std::vector<Triangle>& _scannedTriangles)
	{
		scannedVertices = _scannedVertices;
		scannedTriangles = _scannedTriangles;
		//loadedMesh = _scannedMesh;
		eventQueue.push(GraphicsControlEvent::InitialLoading);
	}

	int GraphicsControl::LoadAndSegmentModelDataFromScan(std::vector<Vertex>& _scannedVertices, std::vector<Triangle>& _scannedTriangles)
	{
		activeScene->second.get()->Lock();
		DebugUtility::DbgOut(L"GraphicsControl::LoadAndSegmentModelDataFromScan::Starting with Reconstruction");
		SetStatusMessage(L"Reconstructing model");
		SetBusy(true);
		//DebugUtility::DbgOut(L"GraphicsControl::LoadStuff:: ", _scannedMesh->GetNumberOfVertices());

		StopWatch stopWatch;
		stopWatch.Start();


		//activeScene->second->LoadFromFile("data\\models\\lowPolyTest.ply");

		//activeScene->second->LoadFromFile("data\\models\\testScene.ply");
		activeScene->second->LoadFromData(_scannedVertices, _scannedTriangles);

		DebugUtility::DbgOut(L"Initialized scene in  ", stopWatch.Stop());
		eventQueue.push(GraphicsControlEvent::ModelDataUpdated);

		glCamera.ResetCameraPosition();
		glCamera.SetRotationPoint(activeScene->second->GetCenterPoint());
		currentCameraMode = OpenGLCameraMode::Free;
		

		DebugUtility::DbgOut(L"GraphicsControl::LoadAndSegmentModelDataFromScan::Starting with segmentation");

		SetStatusMessage(L"Segmenting model");

		for (auto &segmenter : segmenterMap)
			segmenter.second->CleanUp();
		planeSegmenter.CleanUp();
		planeSegmenter.SetSegmentationParameters(PlaneSegmentationParams());
		planeSegmenter.UpdateSegmentation(*this, *activeScene->second.get());
		
		DebugUtility::DbgOut(L"End of Loading Mesh");
		SetBusy(false);
		activeScene->second.get()->Unlock();

		return 0;
	}

#pragma endregion Scene Initializing

#pragma region

	void GraphicsControl::UpdateObjectSegmentation(ObjectSegmentationParams& _params)
	{
		SetStatusMessage(L"Updating segmentation");
		SetBusy(true);
		currentSegmentationType = _params.GetType();
		segmenterMap[currentSegmentationType]->SetSegmentationParameters(_params);
		segmenterMap[currentSegmentationType]->UpdateSegmentation(*this, *activeScene->second.get());
		SetBusy(false);

	}

	void GraphicsControl::UpdatePlaneSegmentation(PlaneSegmentationParams& _params)
	{
		SetStatusMessage(L"Updating segmentation");
		SetBusy(true);

		planeSegmenter.RemoveLastSegmentForNewSegmentation();
		planeSegmenter.SetSegmentationParameters(_params);
		planeSegmenter.UpdateSegmentation(*this, *activeScene->second.get());

		SetBusy(false);
	}

	void GraphicsControl::ResetPlaneSegmentation()
	{
		SetStatusMessage(L"Resetting");
		SetBusy(true);
		planeSegmenter.CleanUp();
		planeSegmenter.SetSegmentationParameters(PlaneSegmentationParams());
		planeSegmenter.UpdateSegmentation(*this, *activeScene->second.get());
		SetBusy(false);
	}

	void GraphicsControl::ConfirmSegmentedPlane(PlaneSegmentationParams& _params)
	{

		SetStatusMessage(L"Updating plane segmentation");
		SetBusy(true);
		planeSegmenter.ConfirmLastSegment();

		boost::thread(&GraphicsControl::PlaneSelectionThread, this, boost::ref(_params));

	}

	void GraphicsControl::RejectSegmentedPlane(PlaneSegmentationParams& _params)
	{
		SetStatusMessage(L"Updating plane segmentation");
		SetBusy(true);
		planeSegmenter.RejectLastSegment();
		boost::thread(&GraphicsControl::PlaneSelectionThread, this, boost::ref(_params));
	}

	int GraphicsControl::PlaneSelectionThread(PlaneSegmentationParams& _params)
	{
		SetBusy(true);

		if (!planeSegmenter.UpdateSegmentation(*this, *activeScene->second.get()))
		{
			planeSegmenter.FinishSegmentation(*activeScene->second.get(), temporarySceneData);
			requestStateChange = true;
		}

		SetBusy(false);
		return 0;
	}

	void GraphicsControl::FinishObjectSegmentation()
	{
		SetStatusMessage(L"Finishing object segmentation");
		SetBusy(true);

		segmenterMap[currentSegmentationType]->FinishSegmentation(*activeScene->second.get(), temporarySceneData);
		eventQueue.push(GraphicsControlEvent::FinishSegmentation);
		
		SetBusy(false);

	}

#pragma endregion Segmentation

#pragma region

	int GraphicsControl::SetupOpenGL()
	{
		openGLWindow.Initialize(parentWindow, hInstance, 0, 0, 0, 0, L"OpenGL", StyleSheet::GetInstance()->GetInnerBackgroundColor());

		if (!glContext.InitOpenGL(hInstance, openGLWindow.GetHandle(), 3, 3))
		{
			MessageBox(0, L"Error with initOpenGL",
				L"ERROR!", MB_OK);
			return -1;
		}
		return 0;
	}

#pragma endregion OpenGL Initializing

#pragma region

	void GraphicsControl::CleanUp()
	{
		DebugUtility::DbgOut(L"GraphicsControl::CleanUp");
		quitMessageLoop = true;

		openGLThread.join();
	}

	void GraphicsControl::CleanUpShaders()
	{
		for (auto &shader : shaderMap)
			shader.second.DeleteProgram();
		shaderMap.clear();
	}

	void GraphicsControl::CleanUpRenderer()
	{
		DebugUtility::DbgOut(L"GraphicsControl::CleanUpRenderer");
		for (auto &renderer : rendererMap)
			renderer.second->CleanUp();
		rendererMap.clear();
	}

	void GraphicsControl::CleanUpModels()
	{
		DebugUtility::DbgOut(L"GraphicsControl::CleanUpModels");
		for (auto &model : sceneMap)
			model.second->CleanUp();
		sceneMap.clear();
		temporarySceneData.CleanUp();
		temporaryPlaneCutDataForReset.CleanUp();
		cutPlane->CleanUp();
	}

	void GraphicsControl::CleanUpSegmenter()
	{
		DebugUtility::DbgOut(L"GraphicsControl::CleanUpSegmenter");
		for (auto &segmenter : segmenterMap)
			segmenter.second->CleanUp();
		segmenterMap.clear();
		planeSegmenter.CleanUp();
	}

	void GraphicsControl::CleanUpIconData()
	{
		DebugUtility::DbgOut(L"GraphicsControl::CleanUpIconData");
		for (auto &icon : iconMap)
			icon.second->CleanUp();
		iconMap.clear();
	}
#pragma endregion CleanUp
}