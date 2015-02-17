#include "OpenGLControl.h"

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

#include "OpenGLCamera.h"
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
	std::unordered_map<WindowState, unique_ptr<ModelData>> sceneMap;
	std::unordered_map<WindowState, unique_ptr<IconData>> iconMap;

	ModelData temporarySceneData;
	InteractionMode interactionMode = None;
	OpenGLCameraMode currentCameraMode = Free;
	ObjectSegmentationType currentSegmentationType;

	std::unordered_map<ObjectSegmentationType, unique_ptr<ObjectSegmenter>> segmenterMap;

	PlaneSegmenter planeSegmenter;

	std::shared_ptr<SimplePlaneRenderable3D> cutPlane;

	std::unordered_map<InteractionMode, unique_ptr<Selector>> selectorMap;
	ColorSelector colorSelector;
	unique_ptr<PlaneSelector> planeSelector;

	OpenGLContext glContext;


	std::unordered_map<OpenGLShaderProgramType, OpenGLShaderProgram> shaderMap;
	//OpenGLShaderProgram defaultMaterial;
	//OpenGLShaderProgram orthoMaterial;

	queue<OpenGLControlEvent> eventQueue;

	OpenGLCamera glCamera;
	OpenGLWindow openGLWindow;

	bool quitMessageLoop = false;
	bool requestStateChange = false;
	bool showPlane = false;

	int parentWindowHeight;
	int parentWindowWidth;

#pragma endregion variables

#pragma region
	OpenGLControl::OpenGLControl()
	{
		fpsCount = 0;
		currentFps = 0;
	}
#pragma endregion Constructors

	void OpenGLControl::Initialize(HWND _parentWindow, HINSTANCE _hInstance)
	{
		parentWindow = _parentWindow;
		hInstance = _hInstance;		

		RECT parentRect;
		GetClientRect(parentWindow, &parentRect);
		parentWindowWidth = parentRect.right;
		parentWindowHeight = parentRect.bottom;
	}

	void OpenGLControl::UpdateApplicationState(WindowState _state)
	{
		DebugUtility::DbgOut(L"OpenGLControl::UpdateApplicationState : ", _state);
		std::unordered_map<WindowState, unique_ptr<OpenGLRenderer>>::iterator activeRenderer = rendererMap.find(_state);
		if (activeRenderer != rendererMap.end())
		{
			currentApplicationState = _state;
			if (currentApplicationState == PlaneSelection)
				openGLWindow.SetMargins(0.07f, 0.042f, 0, 0);
			else
				openGLWindow.SetMargins(0.07f, 0.042f, 0.2f, 0);

			if (!temporarySceneData.IsEmpty())
			{
				eventQueue.push(CopyTemporaryInNextStateModelData);
				if (currentApplicationState == Segmentation)
					eventQueue.push(UpdateSegmentation);
				//else 
				//if (currentApplicationState == Processing)
				//	eventQueue.push(UpdateHoleFilling);
			}
			if (currentApplicationState == PlaneCut)
				eventQueue.push(UpdateCutPlane);
			SetCameraMode(rendererMap[currentApplicationState]->GetCameraMode());
			
			DebugUtility::DbgOut(L"current Camera mode: ", currentCameraMode);
			//sceneMap[currentApplicationState]->UnselectMesh();
			
			openGLWindow.Show();
			DebugUtility::DbgOut(L"after show: ", currentCameraMode);
		}
		else
		{
			openGLWindow.Hide();
			currentApplicationState = _state;
		}
	}

	bool OpenGLControl::RequestsStateChange()
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

	void OpenGLControl::RunOpenGLThread()
	{
		openGLThread = boost::thread(&OpenGLControl::OpenGLThreadMessageLoop, this);
	}

	int OpenGLControl::OpenGLThreadMessageLoop()
	{
		if (SetupOpenGL() == -1)
		{
			DebugUtility::DbgOut(L"OpenGLControl::OpenGLThreadMessageLoop::ERROR::Could not setup OpenGL.");
			PostQuitMessage(0);
			return -1;
		}

		if (!SetupShaders())
		{
			DebugUtility::DbgOut(L"OpenGLControl::OpenGLThreadMessageLoop::ERROR::Could not create shaders.");
			PostQuitMessage(0);
			return -1;
		}
		SetupIconData();
		SetupRenderer();
		SetupSceneData();
		SetupSegmenter();
		SetupSelectors();
		DebugUtility::DbgOut(L"OpenGLControl::OpenGLThreadMessageLoop::After SETUP");
		MSG       msg = { 0 };
		while (WM_QUIT != msg.message && !quitMessageLoop)
		{
			while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) > 0)
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			if (openGLWindow.IsVisible())
			{
				UpdateFrame();
				HandleInput();
				HandleEvents();
			}
			CountFPS();
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

	void OpenGLControl::UpdateFrame()
	{
		if (openGLWindow.IsVisible())
		{
			RECT rRect;
			GetClientRect(parentWindow, &rRect);

			//ResizeOpenGLWindow(rRect.right, rRect.bottom);

			UpdateCamera();

			rendererMap[currentApplicationState]->Render(this, sceneMap[currentApplicationState].get(), iconMap[currentApplicationState].get());
		}
	}

	void OpenGLControl::HandleEvents()
	{
		openGLWindow.HandleEvents(this);

		while (!eventQueue.empty())
		{
			OpenGLControlEvent event = eventQueue.front();

			switch (event)
			{
			case ResizeOpenGLViewport:
				RECT parentRect;
				GetClientRect(parentWindow, &parentRect);
				ResizeOpenGLWindow(parentRect.right, parentRect.bottom);
				break;
			case ModelDataUpdated:
				DebugUtility::DbgOut(L"Before ModelDataUpdated");
				sceneMap[currentApplicationState]->GenerateBuffers();
				DebugUtility::DbgOut(L"After ModelDataUpdated");
				break;
			case ModelHighlightsUpdated:
				sceneMap[currentApplicationState]->SwapToHighlightBuffers();
				break;
			case RemoveModelHighlights:
				sceneMap[currentApplicationState]->GenerateBuffers();
				break;
			case RemoveModelData:
				sceneMap[currentApplicationState]->CleanUp();
				break;
			case CopyTemporaryInNextStateModelData:
				SetBusy(true);

				sceneMap[currentApplicationState]->CleanUp();
				sceneMap[currentApplicationState]->CopyFrom(temporarySceneData);
				temporarySceneData.CleanUp();
				sceneMap[currentApplicationState]->GenerateBuffers();

				SetBusy(false);
				break;
			case ResetModelData:
				sceneMap[currentApplicationState]->ResetToInitialState();
				sceneMap[currentApplicationState]->GenerateBuffers();
				break;
			case UpdateSegmentation:
				boost::thread(&OpenGLControl::UpdateObjectSegmentation, this, new EuclideanSegmentationParams());
				break;
			case UpdateHoleFilling:
				boost::thread(&OpenGLControl::FillHoles, this, 100000);
				break;
			case UpdateCutPlane:
				SetupCutPlane();
				break;
			case SetupCutPlaneMode:
				SetupCutPlane();
				planeSelector->ApplyModeChange();
				break;
			}
			eventQueue.pop();
		}
	}

	void OpenGLControl::HandleInput()
	{
		if (KeyState::GetKeyStateOnce('C'))
		{
			if (currentCameraMode == Sensor)
				SetCameraMode(Free);
			else
				SetCameraMode(Sensor);
		}
		if (openGLWindow.IsCursorInWindow() )
		{
			if (currentApplicationState == Interaction && currentCameraMode == Sensor)
				selectorMap[interactionMode]->HandleSelection(this, sceneMap[currentApplicationState].get(), iconMap[currentApplicationState].get());
			else if (currentApplicationState == Processing)
				colorSelector.HandleSelection(this, sceneMap[currentApplicationState].get(), iconMap[currentApplicationState].get());
			else if (currentApplicationState == PlaneCut && showPlane)
				planeSelector->HandleSelection(this, sceneMap[currentApplicationState].get(), iconMap[currentApplicationState].get());
		}
	}

	void OpenGLControl::SetupRenderer()
	{
		cutPlane = std::shared_ptr<SimplePlaneRenderable3D>(new SimplePlaneRenderable3D());
		
		RECT parentRect;
		GetClientRect(parentWindow, &parentRect);
		ResizeOpenGLWindow(parentRect.right, parentRect.bottom);

		rendererMap[PlaneSelection] = unique_ptr<PlaneSelectionRenderer>(new PlaneSelectionRenderer(Free));
		rendererMap[PlaneSelection]->Initialize(this);
		rendererMap[Segmentation] = unique_ptr<SegmentationRenderer>(new SegmentationRenderer(Free));
		rendererMap[Segmentation]->Initialize(this);
		rendererMap[PlaneCut] = unique_ptr<PlaneCutRenderer>(new PlaneCutRenderer(Free, cutPlane));
		rendererMap[PlaneCut]->Initialize(this);
		rendererMap[Processing] = unique_ptr<ProcessingRenderer>(new ProcessingRenderer(Free));
		rendererMap[Processing]->Initialize(this);
		rendererMap[Interaction] = unique_ptr<InteractionRenderer>(new InteractionRenderer(Sensor));
		rendererMap[Interaction]->Initialize(this);


	}

	void OpenGLControl::SetupSceneData()
	{
		sceneMap[PlaneSelection] = unique_ptr<ModelData>(new ModelData());
		sceneMap[PlaneSelection]->SetDefaultShaderProgram(shaderMap[Default]);
		sceneMap[Segmentation] = unique_ptr<ModelData>(new ModelData());
		sceneMap[Segmentation]->SetDefaultShaderProgram(shaderMap[Default]);
		sceneMap[PlaneCut] = unique_ptr<ModelData>(new ModelData());
		sceneMap[PlaneCut]->SetDefaultShaderProgram(shaderMap[Default]);
		sceneMap[Processing] = unique_ptr<ModelData>(new ModelData());
		sceneMap[Processing]->SetDefaultShaderProgram(shaderMap[Default]);
		sceneMap[Interaction] = unique_ptr<ModelData>(new ModelData());
		sceneMap[Interaction]->SetDefaultShaderProgram(shaderMap[Default]);

		
	}

	bool OpenGLControl::SetupShaders()
	{
		OpenGLShader colorVertexShader;
		colorVertexShader.LoadShader("data\\shaders\\color.vert", GL_VERTEX_SHADER);
		OpenGLShader colorFragmentShader;
		colorFragmentShader.LoadShader("data\\shaders\\color.frag", GL_FRAGMENT_SHADER);

		shaderMap[Default].CreateProgram();
		shaderMap[Default].AddShaderToProgram(colorVertexShader);
		shaderMap[Default].AddShaderToProgram(colorFragmentShader);
		if (!shaderMap[Default].LinkProgram())
			return false;

		OpenGLShader vertexShader2d;
		vertexShader2d.LoadShader("data\\shaders\\shader2d.vert", GL_VERTEX_SHADER);
		OpenGLShader fragmentShader2d;
		fragmentShader2d.LoadShader("data\\shaders\\shader2d.frag", GL_FRAGMENT_SHADER);

		shaderMap[Orthographic].CreateProgram();
		shaderMap[Orthographic].AddShaderToProgram(vertexShader2d);
		shaderMap[Orthographic].AddShaderToProgram(fragmentShader2d);
		if (!shaderMap[Orthographic].LinkProgram())
			return false;

		return true;
	}

	void OpenGLControl::SetupSegmenter()
	{
		segmenterMap[Euclidean] = unique_ptr<EuclideanSegmenter>(new EuclideanSegmenter());
		segmenterMap[RegionGrowth] = unique_ptr<RegionGrowthSegmenter>(new RegionGrowthSegmenter());
		//planeSegmenter = new PlaneSegmenter();
	}

	void OpenGLControl::SetupSelectors()
	{
		selectorMap[Transformation] = unique_ptr<TransformSelector>(new TransformSelector());
		selectorMap[Duplication] = unique_ptr<DuplicateSelector>(new DuplicateSelector());
		selectorMap[None] = unique_ptr<ManipulationSelector>(new ManipulationSelector());
		planeSelector = unique_ptr<PlaneSelector>(new PlaneSelector(cutPlane));
	}

	void OpenGLControl::SetupIconData()
	{
		iconMap[PlaneSelection] = unique_ptr<IconData>(new IconData());
		iconMap[PlaneSelection]->SetDefaultShaderProgram(shaderMap[Orthographic]);
		iconMap[Segmentation] = unique_ptr<IconData>(new IconData());
		iconMap[Segmentation]->SetDefaultShaderProgram(shaderMap[Orthographic]);
		iconMap[PlaneCut] = unique_ptr<IconData>(new IconData());
		iconMap[PlaneCut]->SetDefaultShaderProgram(shaderMap[Orthographic]);
		iconMap[Processing] = unique_ptr<IconData>(new IconData());
		iconMap[Processing]->SetDefaultShaderProgram(shaderMap[Orthographic]);
		iconMap[Interaction] = unique_ptr<IconData>(new IconData());
		iconMap[Interaction]->SetDefaultShaderProgram(shaderMap[Orthographic]);
		iconMap[Interaction]->LoadFromFile("data\\models\\trash.ply", "data\\models\\trash_open.ply", TRASH_BIN);
		iconMap[Interaction]->GenerateBuffers();
	}

	void OpenGLControl::SetupCutPlane()
	{

		glm::vec3 upperBounds = sceneMap[currentApplicationState]->GetUpperBounds();
		glm::vec3 lowerBounds = sceneMap[currentApplicationState]->GetLowerBounds();


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
		cutPlane->SetShaderProgram(shaderMap[Default]);

		cutPlane->UpdateEssentials();
		cutPlane->ApplyTransformation(sceneMap[currentApplicationState]->GetNegativeGroundAlignmentRotation(), glm::mat4(1.0f));
		//cutPlane->ApplyTransformation(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -cutPlane->GetCenterPoint().y, 0.0f)), glm::mat4(1.0f));
		//_glControl->SetPlaneCutParameters(plane->GetPlaneParameters());
		cutPlane->GenerateBuffers();

	}

#pragma endregion Main OpenGL Message Loop

#pragma region

	void OpenGLControl::ShowOpenGLWindow()
	{
		openGLWindow.Show();
	}

	void OpenGLControl::ResizeOpenGLWindow(int _parentWidth, int _parentHeight)
	{
		openGLWindow.Resize(_parentWidth, _parentHeight);
		float ratio = (float)openGLWindow.GetWidth() / (float)openGLWindow.GetHeight();
		SetProjection3D(45.0f, ratio, 0.1f, 1000.0f);
		SetOrtho2D(openGLWindow.GetWidth(), openGLWindow.GetHeight());
	}

	void OpenGLControl::ShowPlaneRenderer(bool _flag)
	{
		showPlane = _flag;
	}

	bool OpenGLControl::IsPlaneRendererVisible()
	{
		return showPlane;
	}
	void OpenGLControl::PrepareViewportResize()
	{
		eventQueue.push(ResizeOpenGLViewport);
	}

	HWND OpenGLControl::GetOpenGLWindowHandle()
	{
		return openGLWindow.GetHandle();
	}

	int OpenGLControl::GetViewportWidth()
	{
		return openGLWindow.GetWidth();
	}

	int OpenGLControl::GetViewportHeight()
	{
		return openGLWindow.GetHeight();
	}

	void OpenGLControl::PushEvent(OpenGLControlEvent _event)
	{
		eventQueue.push(_event);
	}

#pragma endregion OpenGL Window

	void OpenGLControl::SwapBuffers()
	{
		glContext.SwapBuffers();
	}

	void OpenGLControl::CountFPS()
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

	bool OpenGLControl::IsBusy()
	{
		return isBusy;
	}

	float OpenGLControl::GetFramesPerSecond()
	{
		return (float)fpsCount;
	}

	int OpenGLControl::GetMouseWheelDelta()
	{
		return mouseWheelDelta;
	}

	void OpenGLControl::SetMouseWheelDelta(int _mouseWheelDelta)
	{
		mouseWheelDelta = _mouseWheelDelta;
	}

	void OpenGLControl::ResetCamera()
	{
		glCamera.ResetCameraPosition();
	}

	void OpenGLControl::SetBusy(bool _isBusy)
	{
		isBusy = _isBusy;
	}

	void OpenGLControl::SetStatusMessage(wstring _message)
	{
		statusMessage = _message;
	}

	void OpenGLControl::ExecutePlaneCut()
	{
		//DebugUtility::DbgOut(L"OpenGLControl::ExecutePlaneCut");

		if (sceneMap[currentApplicationState]->GetCurrentlySelectedMeshIndex() != -1)
		{
			SetStatusMessage(L"Cutting object in two");
			SetBusy(true);

			PlaneCutSegmentationParams planeCutSegmentationParameters;
			planeCutSegmentationParameters.planeParameters = cutPlane->GetPlaneParameters();

			PlaneCutSegmenter planeCutSegmenter;
			planeCutSegmenter.SetSegmentationParameters(&planeCutSegmentationParameters);

			planeCutSegmenter.UpdateSegmentation(this, sceneMap[currentApplicationState].get());

			planeCutSegmenter.FinishSegmentation(sceneMap[currentApplicationState].get(), sceneMap[currentApplicationState].get());

			planeCutSegmenter.CleanUp();

			eventQueue.push(ModelDataUpdated);
			SetBusy(false);
		}
	}

	void OpenGLControl::PlaneCutPreview()
	{
		//DebugUtility::DbgOut(L"OpenGLControl::PlaneCutPreview");
		if (sceneMap[currentApplicationState]->GetCurrentlySelectedMeshIndex() != -1)
		{
			sceneMap[currentApplicationState]->PlaneCutPreview(sceneMap[currentApplicationState]->GetCurrentlySelectedMeshIndex(), cutPlane->GetPlaneParameters());
			eventQueue.push(ModelHighlightsUpdated);
		}
	}

	wstring OpenGLControl::GetStatusMessage()
	{
		return statusMessage;
	}

	void OpenGLControl::UpdateCamera()
	{
		glCamera.Update(openGLWindow.IsCursorInWindow() && (currentApplicationState != PlaneCut || !showPlane), mouseWheelDelta);
		mouseWheelDelta = 0;
	}

#pragma region 

	void OpenGLControl::SetProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar)
	{
		mProjection = glm::perspective(fFOV, fAspectRatio, fNear, fFar);
	}

	void OpenGLControl::SetOrtho2D(int width, int height)
	{
		mOrtho = glm::ortho(0.0f, float(width), 0.0f, float(height), 0.1f, 1000.0f);
	}

	glm::mat4* OpenGLControl::GetProjectionMatrix()
	{
		return &mProjection;
	}

	glm::mat4 OpenGLControl::GetOrthoMatrix()
	{
		return mOrtho;
	}

	void OpenGLControl::SetCameraMatrix(glm::mat4 viewMatrix)
	{
		mView = viewMatrix;

	}

	glm::mat4* OpenGLControl::GetViewMatrix()
	{
		if (currentCameraMode == Sensor)
			return &mView;
		else
			return &glCamera.GetViewMatrix();
	}


#pragma endregion Get/Set Matrices

	OpenGLShaderProgram OpenGLControl::GetShader(OpenGLShaderProgramType _type)
	{
		return shaderMap[_type];
	}

#pragma region

	void OpenGLControl::ChangeInteractionMode(InteractionMode _interactionMode)
	{
		interactionMode = _interactionMode;
		sceneMap[currentApplicationState]->RemoveTemporaryMeshColor();
		sceneMap[currentApplicationState]->UnselectMesh();
	}

	void OpenGLControl::ChangePlaneCutMode(PlaneCutMode _mode)
	{
		planeSelector->ChangeMode(_mode);
		eventQueue.push(SetupCutPlaneMode);
	}

	void OpenGLControl::SetCameraMode(OpenGLCameraMode _cameraMode)
	{
		currentCameraMode = _cameraMode;
		rendererMap[currentApplicationState]->SetCameraMode(currentCameraMode);

		if (currentCameraMode == Free)
			openGLWindow.ShowButtons();
		else
			openGLWindow.HideButtons();
	}

	bool OpenGLControl::IsRendering()
	{
		return openGLWindow.IsVisible();
	}

	void OpenGLControl::ResetModel()
	{
		eventQueue.push(ResetModelData);
	}

	int OpenGLControl::FillHoles(int _holeSize)
	{
		SetStatusMessage(L"Filling holes");
		SetBusy(true);

		int closedHoles = sceneMap[currentApplicationState]->FillHoles(_holeSize);
		eventQueue.push(ModelDataUpdated);


		SetBusy(false);
		return closedHoles;

	}

	int OpenGLControl::RemoveConnectedComponents(int _maxComponentSize)
	{
		SetStatusMessage(L"Removing components");
		SetBusy(true);

		int removedComponents = sceneMap[currentApplicationState]->RemoveConnectedComponents(_maxComponentSize);
		eventQueue.push(ModelDataUpdated);

		SetBusy(false);
		return removedComponents;
	}



	void OpenGLControl::ExportModel()
	{
		SetStatusMessage(L"Exporting");
		SetBusy(true);

		if (currentCameraMode == Free)
		{
			DebugUtility::DbgOut(L"OpenGLControl::ExportModel::Scenario Exporter");
			ScenarioExporter exporter;
			exporter.Export(sceneMap[currentApplicationState].get());
		}
		else
		{
			DebugUtility::DbgOut(L"OpenGLControl::ExportModel::Dialog Exporter");
			DialogExporter exporter;
			if (sceneMap[currentApplicationState]->GetCurrentlySelectedMeshIndex() != -1)
			{
				DebugUtility::DbgOut(L"OpenGLControl::ExportModel::Selected Mesh");
				exporter.Export(sceneMap[currentApplicationState].get(), sceneMap[currentApplicationState]->GetCurrentlySelectedMeshIndex());
			}
			else
			{
				DebugUtility::DbgOut(L"OpenGLControl::ExportModel::Whole Mesh");
				exporter.Export(sceneMap[currentApplicationState].get());
			}
		}

		SetBusy(false);
	}

	void OpenGLControl::FinishProcessing()
	{
		SetStatusMessage(L"Finishing processing");
		SetBusy(true);

		temporarySceneData.CopyFrom(*sceneMap[currentApplicationState].get());

		SetBusy(false);
	}


	int OpenGLControl::GetNumberOfVertices()
	{
		std::unordered_map<WindowState, unique_ptr<ModelData>>::iterator it = sceneMap.find(currentApplicationState);
		if (it != sceneMap.end())
		{
			return sceneMap[currentApplicationState]->GetNumberOfVertices();
		}
		return 0;
		
	}

	int OpenGLControl::GetNumberOfTriangles()
	{
		std::unordered_map<WindowState, unique_ptr<ModelData>>::iterator it = sceneMap.find(currentApplicationState);
		if (it != sceneMap.end())
		{
			return sceneMap[currentApplicationState]->GetNumberOfTriangles();
		}
		return 0;
	}

	int OpenGLControl::GetNumberOfVisibleModels()
	{
		std::unordered_map<WindowState, unique_ptr<ModelData>>::iterator it = sceneMap.find(currentApplicationState);
		if (it != sceneMap.end())
		{
			return sceneMap[currentApplicationState]->GetVisibleMeshCount();
		}
		return 0;
	}

#pragma endregion Processing

#pragma region

	int OpenGLControl::LoadAndSegmentModelDataFromScan(std::shared_ptr<MeshContainer> _scannedMesh)
	{
		DebugUtility::DbgOut(L"OpenGLControl::LoadAndSegmentModelDataFromScan::Starting with Reconstruction");
		SetStatusMessage(L"Reconstructing model");
		SetBusy(true);

		//DebugUtility::DbgOut(L"OpenGLControl::LoadStuff:: ", _scannedMesh->GetNumberOfVertices());
		
		StopWatch stopWatch;
		stopWatch.Start();

		sceneMap[PlaneSelection]->LoadFromFile("data\\models\\testScene.ply");
		//sceneMap[PlaneSelection]->LoadFromData(_scannedMesh);

		DebugUtility::DbgOut(L"Initialized scene in  ", stopWatch.Stop());



		glCamera.SetRotationPoint(sceneMap[PlaneSelection]->GetCenterPoint());
		currentCameraMode = Free;
		eventQueue.push(ModelDataUpdated);

		DebugUtility::DbgOut(L"OpenGLControl::LoadAndSegmentModelDataFromScan::Starting with segmentation");

		SetStatusMessage(L"Segmenting model");

		for (auto &segmenter : segmenterMap)
			segmenter.second->CleanUp();
		planeSegmenter.CleanUp();
		planeSegmenter.SetSegmentationParameters(new PlaneSegmentationParams());
		planeSegmenter.UpdateSegmentation(this, sceneMap[currentApplicationState].get());

		DebugUtility::DbgOut(L"End of Loading Mesh");
		SetBusy(false);

		return 0;
	}

#pragma endregion Scene Initializing

#pragma region
	
	void OpenGLControl::UpdateObjectSegmentation(ObjectSegmentationParams* _params)
	{
		SetStatusMessage(L"Updating segmentation");
		SetBusy(true);
		currentSegmentationType = _params->GetType();
		segmenterMap[currentSegmentationType]->SetSegmentationParameters(_params);
		segmenterMap[currentSegmentationType]->UpdateSegmentation(this, sceneMap[currentApplicationState].get());
		SetBusy(false);
		
	}

	void OpenGLControl::UpdatePlaneSegmentation(PlaneSegmentationParams* _params)
	{
		SetStatusMessage(L"Updating segmentation");
		SetBusy(true);

		planeSegmenter.RemoveLastSegmentForNewSegmentation();
		planeSegmenter.SetSegmentationParameters(_params);
		planeSegmenter.UpdateSegmentation(this, sceneMap[currentApplicationState].get());

		SetBusy(false);
	}

	void OpenGLControl::ConfirmSegmentedPlane(PlaneSegmentationParams* _params)
	{
		SetStatusMessage(L"Updating plane segmentation");
		SetBusy(true);
		planeSegmenter.ConfirmLastSegment();

		boost::thread(&OpenGLControl::PlaneSelectionThread, this, _params);

	}

	void OpenGLControl::RejectSegmentedPlane(PlaneSegmentationParams* _params)
	{
		SetStatusMessage(L"Updating plane segmentation");
		SetBusy(true);
		planeSegmenter.RejectLastSegment();
		boost::thread(&OpenGLControl::PlaneSelectionThread, this, _params);
	}

	int OpenGLControl::PlaneSelectionThread(PlaneSegmentationParams* _params)
	{
		SetBusy(true);

		if (!planeSegmenter.UpdateSegmentation(this, sceneMap[currentApplicationState].get()))
		{
			planeSegmenter.FinishSegmentation(sceneMap[currentApplicationState].get(), &temporarySceneData);
			requestStateChange = true;
		}

		SetBusy(false);
		return 0;
	}

	void OpenGLControl::FinishObjectSegmentation()
	{
		SetStatusMessage(L"Finishing object segmentation");
		SetBusy(true);

		segmenterMap[currentSegmentationType]->FinishSegmentation(sceneMap[currentApplicationState].get(), &temporarySceneData);

		SetBusy(false);

	}

#pragma endregion Segmentation

#pragma region

	int OpenGLControl::SetupOpenGL()
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

	void OpenGLControl::CleanUp()
	{
		DebugUtility::DbgOut(L"OpenGLControl::CleanUp");
		quitMessageLoop = true;

		openGLThread.join();
	}

	void OpenGLControl::CleanUpShaders()
	{
		for (auto &shader : shaderMap)
			shader.second.DeleteProgram();
		shaderMap.clear();
	}

	void OpenGLControl::CleanUpRenderer()
	{
		DebugUtility::DbgOut(L"OpenGLControl::CleanUpRenderer");
		for (auto &renderer : rendererMap)
			renderer.second->CleanUp();
		rendererMap.clear();
	}

	void OpenGLControl::CleanUpModels()
	{
		DebugUtility::DbgOut(L"OpenGLControl::CleanUpModels");
		for (auto &model : sceneMap)
			model.second->CleanUp();
		sceneMap.clear();
		temporarySceneData.CleanUp();
		cutPlane->CleanUp();
	}

	void OpenGLControl::CleanUpSegmenter()
	{
		DebugUtility::DbgOut(L"OpenGLControl::CleanUpSegmenter");
		for (auto &segmenter : segmenterMap)
			segmenter.second->CleanUp();
		segmenterMap.clear();
		planeSegmenter.CleanUp();
	}

	void OpenGLControl::CleanUpIconData()
	{
		DebugUtility::DbgOut(L"OpenGLControl::CleanUpIconData");
		for (auto &icon : iconMap)
			icon.second->CleanUp();
		iconMap.clear();
	}
#pragma endregion CleanUp
}