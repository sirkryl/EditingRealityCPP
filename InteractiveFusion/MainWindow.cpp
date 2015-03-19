#include "MainWindow.h"

#include <memory>
#include <boost/thread.hpp>

#include "IFResources.h"

#include "DebugUtility.h"
#include "KeyState.h"
#include "StyleSheet.h"

#include "GraphicsControl.h"
#include "StringConverter.h"
#include "ButtonLayout.h"
#include "GUIContainer.h"

#include "MeshContainer.h"
#include "HelpWindow.h"
#include "InteractionWindow.h"
#include "PlaneSelectionWindow.h"
#include "PrepareWindow.h"
#include "ProcessingWindow.h"
#include "ScanWindow.h"
#include "SegmentationWindow.h"
#include "PlaneCutWindow.h"
#include <ctime>
#include <unordered_map>
using namespace std;

namespace InteractiveFusion {
#pragma region

	HWND hWndMain;
	HINSTANCE appInstance;
	WindowState currentState;
	
	GraphicsControl glControl;

	std::unordered_map<WindowState, unique_ptr<SubWindow>> subWindowMap;

	HelpWindow helpWindow;

	std::unordered_map<WindowState, HWND> modeButtonMap;

	TCHAR KeyState::keysPressed[256] = { 0 };
	bool KeyState::leftMouseDown = false;
	//STATUS BAR
	HWND statusHandle;

	HWND fpsText, fpsCount;
	HWND verticesLabel, verticesCount;
	HWND trianglesLabel, trianglesCount;

	//WINDOW MODE TABS
	GUIContainer  modeUi;
	HWND hModePrepare, hModeScan, hModeSegmentation, hModeInteraction, hModeProcessing;
	GUIContainer segmentationModes;
	HWND hSubModeBackground;
	HWND hSubModePlaneSegmentation, hSubModeObjectSegmentation, hSubModePlaneCut;
	int totalGpuMemory = 0;
	int scanVolumeVoxelsPerMeter;
	int currentVertexCount = 0;
	int currentTriangleCount = 0;

	DWORD tickLastStatus;
	DWORD tickLastFpsUpdate;
	float fFrameInterval;
	clock_t tLastFrame;

	std::wstring nextStatusBarMessage;
	std::wstring currentStatusBarMessage;

	std::unordered_map<HWND, ButtonLayout> buttonLayout;

	bool mainInitDone = false;

	bool quit = false;

	HBRUSH backgroundBrush;
	HFONT statusBarFont;
	HFONT uiFontMedium, uiFontSmall;
#pragma endregion variables

	void MainWindow::InitApplication(HINSTANCE hInstance)
	{
		appInstance = hInstance;

		StyleSheet::GetInstance()->CreateStyles();
		uiFontMedium = CreateFont(30, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());
		uiFontSmall = CreateFont(13, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());

		statusBarFont = CreateFont(22, 10, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, 0, 0, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontName().c_str());

		backgroundBrush = CreateSolidBrush(RGB(StyleSheet::GetInstance()->GetGlobalBackgroundColor().r, StyleSheet::GetInstance()->GetGlobalBackgroundColor().g, StyleSheet::GetInstance()->GetGlobalBackgroundColor().b));


		WNDCLASSEX wc = { 0 };
		wc.cbSize = sizeof(WNDCLASSEX);
		wc.lpfnWndProc = (WNDPROC)MainWindow::MessageRouter;
		wc.style = 0; 
		wc.hInstance = hInstance;
		wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = backgroundBrush;
		wc.lpszMenuName = NULL;
		wc.lpszClassName = L"MainWindow";
		wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
		ATOM ClassAtom = RegisterClassExW(&wc);

		hWndMain = CreateWindowExW(WS_EX_CONTROLPARENT, (LPCTSTR)MAKELONG(ClassAtom, 0), 0, WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_CAPTION | WS_SYSMENU,
			0, 0, 960, 545, 0,
			NULL, hInstance, this);


		ShowWindow(hWndMain, SW_SHOW);

		hModePrepare = CreateWindowEx(0, L"BUTTON", L"PREPARE", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hWndMain, (HMENU)IDC_MODE_BUTTON_PREPARE, NULL, 0);

		buttonLayout.emplace(hModePrepare, ButtonLayout());
		buttonLayout[hModePrepare].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
		hModeSegmentation = CreateWindowEx(0, L"BUTTON", L"SEGMENT", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hWndMain, (HMENU)IDC_MODE_BUTTON_SEGMENTATION, NULL, 0);

		buttonLayout.emplace(hModeSegmentation, ButtonLayout());
		buttonLayout[hModeSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		hModeScan = CreateWindowEx(0, L"BUTTON", L"SCAN", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hWndMain, (HMENU)IDC_MODE_BUTTON_SCAN, NULL, 0);
		buttonLayout.emplace(hModeScan, ButtonLayout());
		buttonLayout[hModeScan].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		hModeProcessing = CreateWindowEx(0, L"BUTTON", L"PROCESS", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hWndMain, (HMENU)IDC_MODE_BUTTON_PROCESSING, NULL, 0);
		buttonLayout.emplace(hModeProcessing, ButtonLayout());
		buttonLayout[hModeProcessing].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		hModeInteraction = CreateWindowEx(0, L"BUTTON", L"MANIPULATE", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hWndMain, (HMENU)IDC_MODE_BUTTON_INTERACTION, NULL, 0);
		buttonLayout.emplace(hModeInteraction, ButtonLayout());
		buttonLayout[hModeInteraction].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		statusHandle = CreateWindowEx(WS_EX_TOPMOST, L"STATIC", L"", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTERIMAGE, 250, 50, 150, 50, hWndMain, (HMENU)IDC_STATUS, NULL, 0);

		SendMessage(statusHandle, WM_SETFONT, (WPARAM)statusBarFont, 0);
		ShowWindow(statusHandle, SW_SHOW);

		fpsText = CreateWindowEx(0, L"STATIC", L"FPS:", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, hWndMain, (HMENU)IDC_FRAMES_PER_SECOND_TEXT, NULL, 0);
		fpsCount = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, hWndMain, (HMENU)IDC_FRAMES_PER_SECOND, NULL, 0);

		SendMessage(fpsText, WM_SETFONT, (WPARAM)uiFontMedium, 0);
		SendMessage(fpsCount, WM_SETFONT, (WPARAM)uiFontMedium, 0);

		verticesLabel = CreateWindowEx(0, L"STATIC", L"Vertices:", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTERIMAGE , 250, 50, 150, 50, hWndMain, (HMENU)0, NULL, 0);
		verticesCount = CreateWindowEx(0, L"STATIC", L"0", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, hWndMain, (HMENU)IDC_VERTICES_COUNT, NULL, 0);
		trianglesLabel = CreateWindowEx(0, L"STATIC", L"Triangles:", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, hWndMain, (HMENU)0, NULL, 0);
		trianglesCount = CreateWindowEx(0, L"STATIC", L"0", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, hWndMain, (HMENU)IDC_TRIANGLES_COUNT, NULL, 0);

		SendMessage(verticesLabel, WM_SETFONT, (WPARAM)uiFontSmall, 0);
		SendMessage(trianglesLabel, WM_SETFONT, (WPARAM)uiFontSmall, 0);
		SendMessage(verticesCount, WM_SETFONT, (WPARAM)uiFontSmall, 0);
		SendMessage(trianglesCount, WM_SETFONT, (WPARAM)uiFontSmall, 0);

		SetWindowPos(verticesLabel, HWND_TOP, 0, 0, 0, 0, 0);
		SetWindowPos(trianglesLabel, verticesLabel, 0, 0, 0, 0, 0);
		SetWindowPos(verticesCount, trianglesLabel, 0, 0, 0, 0, 0);
		SetWindowPos(trianglesCount, verticesCount, 0, 0, 0, 0, 0);
		SetWindowPos(statusHandle, trianglesCount, 0, 0, 0, 0, 0);
		hSubModeBackground = CreateWindowEx(0, (LPCTSTR)MAKELONG(ClassAtom, 0), L"", WS_CHILD | WS_CLIPSIBLINGS, 250, 50, 150, 50, hWndMain, (HMENU)0, NULL, 0);

		hSubModePlaneSegmentation = CreateWindowEx(0, L"BUTTON", L"", WS_CHILD | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hSubModeBackground, (HMENU)IDC_MODE_BUTTON_SEG_PLANE, NULL, 0);

		buttonLayout.emplace(hSubModePlaneSegmentation, ButtonLayout());
		buttonLayout[hSubModePlaneSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		hSubModeObjectSegmentation = CreateWindowEx(0, L"BUTTON", L"", WS_CHILD | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hSubModeBackground, (HMENU)IDC_MODE_BUTTON_SEG_OBJECT, NULL, 0);

		buttonLayout.emplace(hSubModeObjectSegmentation, ButtonLayout());
		buttonLayout[hSubModeObjectSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		hSubModePlaneCut = CreateWindowEx(0, L"BUTTON", L"", WS_CHILD | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, hSubModeBackground, (HMENU)IDC_MODE_BUTTON_SEG_PLANECUT, NULL, 0);

		buttonLayout.emplace(hSubModePlaneCut, ButtonLayout());
		buttonLayout[hSubModePlaneCut].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		

		segmentationModes.Add(hSubModePlaneSegmentation);
		segmentationModes.Add(hSubModeObjectSegmentation);
		segmentationModes.Add(hSubModePlaneCut);
		segmentationModes.Add(hSubModeBackground);

		modeUi.Add(hModeProcessing);
		modeUi.Add(hModePrepare);
		modeUi.Add(hModeSegmentation);
		modeUi.Add(hModeScan);
		modeUi.Add(hModeInteraction);

		modeButtonMap[Prepare] = hModePrepare;
		modeButtonMap[Scan] = hModeScan;
		modeButtonMap[PlaneSelection] = hModeSegmentation;
		modeButtonMap[Segmentation] = hModeSegmentation;
		modeButtonMap[PlaneCut] = hModeSegmentation;
		modeButtonMap[Processing] = hModeProcessing;
		modeButtonMap[Interaction] = hModeInteraction;

		subWindowMap[Prepare] = unique_ptr<PrepareWindow>(new PrepareWindow());
		subWindowMap[Prepare]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0, 0, L"Prepare", StyleSheet::GetInstance()->GetInnerBackgroundColor());
		subWindowMap[Scan] = unique_ptr<ScanWindow>(new ScanWindow());
		subWindowMap[Scan]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0, 0, L"Scan", StyleSheet::GetInstance()->GetGlobalBackgroundColor());
		subWindowMap[PlaneSelection] = unique_ptr<PlaneSelectionWindow>(new PlaneSelectionWindow());
		subWindowMap[PlaneSelection]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0.0f, 0.0f, L"PlaneSelection", StyleSheet::GetInstance()->GetGlobalBackgroundColor());
		subWindowMap[Segmentation] = unique_ptr<SegmentationWindow>(new SegmentationWindow());
		subWindowMap[Segmentation]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0, 0.8f, L"Segmentation", StyleSheet::GetInstance()->GetGlobalBackgroundColor());
		subWindowMap[PlaneCut] = unique_ptr<PlaneCutWindow>(new PlaneCutWindow());
		subWindowMap[PlaneCut]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0, 0.8f, L"PlaneCut", StyleSheet::GetInstance()->GetGlobalBackgroundColor());
		subWindowMap[Processing] = unique_ptr<ProcessingWindow>(new ProcessingWindow());
		subWindowMap[Processing]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0, 0.8f, L"Processing", StyleSheet::GetInstance()->GetGlobalBackgroundColor());
		subWindowMap[Interaction] = unique_ptr<InteractionWindow>(new InteractionWindow());
		subWindowMap[Interaction]->Initialize(hWndMain, hInstance, 0.07f, 0.042f, 0, 0.8f, L"Interaction", StyleSheet::GetInstance()->GetGlobalBackgroundColor());

		helpWindow.Initialize(hWndMain, hInstance, 0.35f, 0.35f, 0.225f, 0.225f, L"Help", StyleSheet::GetInstance()->GetGlobalBackgroundColor());
		

		glControl.Initialize(hWndMain, hInstance);
		glControl.RunOpenGLThread();

		ChangeState(Prepare);

		ShowWindow(hWndMain, SW_MAXIMIZE);
		MSG       msg = { 0 };
		while (WM_QUIT != msg.message)
		{
			while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) > 0) 
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			
			subWindowMap[currentState]->HandleEvents(this);

			UpdateUIActivation();
			UpdateStatusBarMessage();

			if (glControl.IsRendering())
			{
				UpdateOpenGLControl();
				UpdateSceneInformation();
			}
			if (helpWindow.IsVisible())
				helpWindow.HandleEvents(this);
		}
		quit = true;
		CleanUp();
		return;
	}

	void MainWindow::ChangeState(WindowState _state)
	{
		DebugUtility::DbgOut(L"ChangeState ", currentState);
		if (!IsValidState(_state))
			return;
		if (IsValidState(currentState))
			subWindowMap[currentState]->Hide();

		
		currentState = _state;
		
		RECT rRect;
		GetClientRect(hWndMain, &rRect);
		
		UpdateModeButtons();
		subWindowMap[currentState]->Show();
		glControl.UpdateApplicationState(currentState);
		
		if (currentState == PlaneSelection)
			helpWindow.Show();
		if (currentState == Prepare || currentState == Scan || currentState == Interaction)
			dynamic_cast<ScanWindow*>(subWindowMap[Scan].get())->UnpauseScan();
		else
			dynamic_cast<ScanWindow*>(subWindowMap[Scan].get())->PauseScan();

		MainWindowProc(hWndMain, WM_SIZE, 0, MAKELPARAM(rRect.right, rRect.bottom));

		DebugUtility::DbgOut(L"End ChangeState ", currentState);
	}

	void MainWindow::UpdateModeButtons()
	{
		HWND activeHandle = NULL;
		for (auto modeButton : modeButtonMap)
		{
			if (modeButton.first == currentState)
			{
				//buttonLayout[modeButton.second].CleanUp();
				//DebugUtility::DbgOut(L"modeButtonGettingActive");
				//buttonLayout[modeButton.second].CleanUp();
				buttonLayout[modeButton.second].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ActiveMode));
				activeHandle = modeButton.second;
				
				//EnableWindow(modeButton.second, false);
			}
			else if (modeButton.second != activeHandle)
			{
				//buttonLayout[modeButton.second].CleanUp();
				//buttonLayout[modeButton.second].CleanUp();
				buttonLayout[modeButton.second].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
				//buttonLayout[modeButton.second] = modeLayout;
				//EnableWindow(modeButton.second, true);
			}
		}
		if (currentState >= PlaneSelection & currentState <= PlaneCut)
		{
			segmentationModes.Show();
			buttonLayout[hSubModePlaneSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
			buttonLayout[hSubModeObjectSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
			buttonLayout[hSubModePlaneCut].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
			switch (currentState)
			{
			case PlaneSelection:
				buttonLayout[hSubModePlaneSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
				break;
			case Segmentation:
				buttonLayout[hSubModeObjectSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
				break;
			case PlaneCut:
				buttonLayout[hSubModePlaneCut].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
				break;
			}
			segmentationModes.Redraw();
		}
		else
			segmentationModes.Hide();

		modeUi.Redraw();
	}

	void MainWindow::ChangeScanVolumeSize(int _voxelsPerMeter)
	{
		scanVolumeVoxelsPerMeter = _voxelsPerMeter;
	}

	void MainWindow::ChangeInteractionMode(InteractionMode _interactionMode)
	{
		glControl.ChangeInteractionMode(_interactionMode);
	}

	void MainWindow::ChangePlaneCutMode(PlaneCutMode _mode)
	{
		glControl.ChangePlaneCutMode(_mode);
	}

	void MainWindow::SetCameraMode(OpenGLCameraMode _cameraMode)
	{
		glControl.SetCameraMode(_cameraMode);
	}

	void MainWindow::ReloadModel()
	{
		glControl.ReloadCurrentState();
		SetStatusBarMessage(L"Reloading current state.");
	}

	void MainWindow::ResetModel()
	{
		glControl.ResetModel();
		SetStatusBarMessage(L"Model reset.");
	}

	int MainWindow::FillHoles(int _holeSize)
	{
		int closedHoles = glControl.FillHoles(_holeSize);
		SetStatusBarMessage(to_wstring(closedHoles) + L" holes have been closed.");
		return closedHoles;
	}

	int MainWindow::RemoveConnectedComponents(int _maxComponentSize)
	{
		int removedComponents = glControl.RemoveConnectedComponents(_maxComponentSize);
		SetStatusBarMessage(to_wstring(removedComponents) + L" components have been removed.");
		return removedComponents;
	}

	void MainWindow::ExportModel()
	{
		SetStatusBarMessage(L"Exporting model...");
		glControl.ExportModel();
	}

	void MainWindow::UpdateObjectSegmentation(ObjectSegmentationParams* _params)
	{
		SetStatusBarMessage(L"Updating object segmentation...");
		boost::thread(&GraphicsControl::UpdateObjectSegmentation, &glControl, _params);
		//glControl.UpdateObjectSegmentation(_params);
	}

	void MainWindow::UpdatePlaneSegmentation(PlaneSegmentationParams* _params)
	{
		SetStatusBarMessage(L"Updating plane segmentation...");
		boost::thread(&GraphicsControl::UpdatePlaneSegmentation, &glControl, _params);
		//glControl.UpdatePlaneSegmentation(_params);
	}

	void MainWindow::ConfirmSegmentedPlane(PlaneSegmentationParams* _params)
	{
		SetStatusBarMessage(L"Confirmed plane, looking for more...");
		glControl.ConfirmSegmentedPlane(_params);
	}

	void MainWindow::RejectSegmentedPlane(PlaneSegmentationParams* _params)
	{
		SetStatusBarMessage(L"Rejected plane, looking for more...");
		glControl.RejectSegmentedPlane(_params);
	}

	void MainWindow::FinishObjectSegmentation()
	{
		SetStatusBarMessage(L"Finishing object segmentation...");
		glControl.FinishObjectSegmentation();
	}

	void MainWindow::FinishProcessing()
	{
		SetStatusBarMessage(L"Finishing processing...");
		glControl.FinishProcessing();
	}

	void MainWindow::InitializeOpenGLScene(std::shared_ptr<MeshContainer> _scannedMesh)
	{
		SetStatusBarMessage(L"Reconstructing and cleaning scene...");
		glControl.LoadAndSegmentModelDataFromScan(_scannedMesh);
		//boost::thread(&GraphicsControl::LoadAndSegmentModelDataFromScan, &glControl, _scannedMesh);
	}

	void MainWindow::SetPlaneRenderer(bool _flag)
	{
		glControl.ShowPlaneRenderer(_flag);
	}

	void MainWindow::ExecutePlaneCut()
	{
		boost::thread(&GraphicsControl::ExecutePlaneCut, &glControl);
	}

	int MainWindow::GetScanVolumeSize()
	{
		return scanVolumeVoxelsPerMeter;
	}

	void MainWindow::SetFramesPerSecond(float fps)
	{
		//DebugUtility::DbgOut(L"MainWindow::SetFramesPerSecond:: ", fps);
		if (fps >= 0)
		{
			//std::wstringstream out;
			//out << std::setprecision(2) << fps;
			SetDlgItemText(hWndMain, IDC_FRAMES_PER_SECOND, std::to_wstring((int)fps).c_str());
		}
	}

	void MainWindow::SetStatusBarMessage(wstring _statusBarMessage)
	{
		nextStatusBarMessage = _statusBarMessage;
	}

	void MainWindow::UpdateStatusBarMessage()
	{
		if (nextStatusBarMessage.length() > 0 && nextStatusBarMessage != currentStatusBarMessage)
		{
			DebugUtility::DbgOut(L"MainWindow::UpdateStatusBarMessage()::UpdatingStatusBarMessage");
			currentStatusBarMessage = nextStatusBarMessage;
			DebugUtility::DbgOut(L"MainWindow::UpdateStatusBarMessage()::NEW UpdatingStatusBarMessage: " + currentStatusBarMessage);
			SetDlgItemText(hWndMain, IDC_STATUS, currentStatusBarMessage.c_str());

			DebugUtility::DbgOut(L"MainWindow::UpdateStatusBarMessage()::NEW MESSAGE: " + currentStatusBarMessage);
			tickLastStatus = GetTickCount();
			DebugUtility::DbgOut(L"MainWindow::UpdateStatusBarMessage()::After Clearing: " + currentStatusBarMessage);
		}
		else if (GetTickCount() - tickLastStatus > statusTimeOutInMilliSeconds)
		{
			SetDlgItemText(hWndMain, IDC_STATUS, L"");
			tickLastStatus = GetTickCount();
			currentStatusBarMessage.clear();
		}
		nextStatusBarMessage.clear();
	}

#pragma region


	LRESULT CALLBACK MainWindow::MessageRouter(
		HWND hWnd,
		UINT uMsg,
		WPARAM wParam,
		LPARAM lParam)
	{
		MainWindow* pThis = nullptr;

		if (WM_CREATE == uMsg)
		{
			DebugUtility::DbgOut(L"Setting 'this'");
			pThis = reinterpret_cast<MainWindow*>(lParam);
			SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
		}
		else
		{
			pThis = reinterpret_cast<MainWindow*>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
		}

		if (pThis)
		{
			return pThis->MainWindowProc(hWnd, uMsg, wParam, lParam);
		}
		else return DefWindowProc(hWnd, uMsg, wParam, lParam);

		return 0;
	}

	void MainWindow::UpdateOpenGLControl()
	{
		if (currentState == Interaction)
			glControl.SetCameraMatrix(dynamic_cast<ScanWindow*>(subWindowMap[Scan].get())->GetCameraMatrix());

		if (GetTickCount() - tickLastFpsUpdate > updateFpsCounterInMilliSeconds)
		{
			SetFramesPerSecond(glControl.GetFramesPerSecond());
			tickLastFpsUpdate = GetTickCount();
		}

		if (glControl.RequestsStateChange())
		{
			if ((int)currentState+1 <= Interaction)
				ChangeState((WindowState)((int)currentState + 1));
			
		}
	}

	void MainWindow::UpdateSceneInformation()
	{
		if (glControl.GetNumberOfVertices() != currentVertexCount)
		{
			currentVertexCount = glControl.GetNumberOfVertices();
			std::wstringstream out;
			out << std::setprecision(2) << currentVertexCount;
			SetDlgItemText(hWndMain, IDC_VERTICES_COUNT, out.str().c_str());
		}
		if (glControl.GetNumberOfTriangles() != currentTriangleCount)
		{
			currentTriangleCount = glControl.GetNumberOfTriangles();
			std::wstringstream out;
			out << std::setprecision(2) << currentTriangleCount;
			SetDlgItemText(hWndMain, IDC_TRIANGLES_COUNT, out.str().c_str());
		}
	}

	/// <summary>
	/// Handle windows messages for the class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK MainWindow::MainWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		switch (message)
		{
			case WM_CLOSE:
				DestroyWindow(hWnd);
				ShutdownWindow();
				PostQuitMessage(0);
				break;
			case WM_DESTROY:
				break;
			case WM_COMMAND:
				if (!glControl.IsBusy())
					ProcessUI(wParam, lParam);
				break;
			case WM_DRAWITEM:
			{
				auto currentButtonLayout = buttonLayout.find(((LPDRAWITEMSTRUCT)lParam)->hwndItem);
				if (currentButtonLayout != buttonLayout.end()) {
					return currentButtonLayout->second.Draw(lParam);
				}
			}
				break;
			case WM_CTLCOLORSTATIC:
			{
				HDC hdc = reinterpret_cast<HDC>(wParam);
				SetBkMode((HDC)wParam, TRANSPARENT);
				SetTextColor(hdc, RGB(StyleSheet::GetInstance()->GetDefaultTextColor().r, StyleSheet::GetInstance()->GetDefaultTextColor().g, StyleSheet::GetInstance()->GetDefaultTextColor().b));

				return (LRESULT)backgroundBrush;
			}
			break;
			case WM_MOUSEWHEEL:
				glControl.SetMouseWheelDelta(GET_WHEEL_DELTA_WPARAM(wParam));
				break;
			case WM_NCLBUTTONDBLCLK:
			case WM_SIZE:
				MoveModeButtonsOnResize(LOWORD(lParam), HIWORD(lParam));
				helpWindow.Resize(LOWORD(lParam), HIWORD(lParam));
				if (IsValidState(currentState))
				{
					subWindowMap[currentState]->Resize(LOWORD(lParam), HIWORD(lParam));
					glControl.PrepareViewportResize();
				}
				break;
			default:
				return DefWindowProc(hWnd, message, wParam, lParam);
		}
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	void MainWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_MODE_BUTTON_PREPARE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != Prepare)
				ChangeState(Prepare);
		}
		if (IDC_MODE_BUTTON_SCAN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != Scan)
				ChangeState(Scan);
		}
		if (IDC_MODE_BUTTON_SEGMENTATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState < PlaneSelection || currentState > PlaneCut)
				ChangeState(Segmentation);

			segmentationModes.Show();
		}
		if (IDC_MODE_BUTTON_SEG_OBJECT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != Segmentation)
				ChangeState(Segmentation);
		}
		if (IDC_MODE_BUTTON_SEG_PLANE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != PlaneSelection)
				ChangeState(PlaneSelection);
		}
		if (IDC_MODE_BUTTON_SEG_PLANECUT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != PlaneCut)
				ChangeState(PlaneCut);
		}
		if (IDC_MODE_BUTTON_PROCESSING == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != Processing)
				ChangeState(Processing);
		}
		if (IDC_MODE_BUTTON_INTERACTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (currentState != Interaction)
				ChangeState(Interaction);
		}
	}


	void MainWindow::MoveModeButtonsOnResize(int width, int height)
	{
		DebugUtility::DbgOut(L"MainWindow::MoveModeButtonsOnResize()");
		MoveWindow(hModePrepare, width / 2 - 380, 8, 130, (int)(0.07f*height) - 8, true);
		MoveWindow(hModeScan, width / 2 - 250, 8, 110, (int)(0.07f*height) - 8, true);
		
		MoveWindow(hModeSegmentation, width / 2 - 140, 8, 205, (int)(0.07f*height) - 8, true);
		MoveWindow(hModeProcessing, width / 2 + 65, 8, 130, (int)(0.07f*height) - 8, true);

		MoveWindow(hModeInteraction, width / 2 + 195, 8, 185, (int)(0.07f*height) - 8, true);

		MoveWindow(statusHandle, 10, height - 30, width-10, 30, true);

		MoveWindow(hSubModeBackground, width / 2 - 140, (int)(0.07f*height), 205, 30, true);

		MoveWindow(hSubModePlaneSegmentation, 20, 5, 20, 20, true);
		MoveWindow(hSubModeObjectSegmentation, 93, 5, 20, 20, true);
		MoveWindow(hSubModePlaneCut, 165, 5, 20, 20, true);

		MoveWindow(fpsText, width - 95, 0, 40, (int)(0.07f*height), true);
		MoveWindow(fpsCount, width - 50, 0, 40, (int)(0.07f*height), true);

		MoveWindow(verticesLabel, width-120, height - 29, 60, 12, true);
		MoveWindow(verticesCount, width-60, height - 29, 60, 12, true);
		MoveWindow(trianglesLabel, width-120, height - 15, 60, 12, true);
		MoveWindow(trianglesCount, width-60, height - 15, 60, 12, true);
	}

	void MainWindow::ShutdownWindow()
	{
		DebugUtility::DbgOut(L"MainWindow::ShutdownWindow()");
		DeleteObject(backgroundBrush);
		DeleteObject(statusBarFont);
		DeleteObject(uiFontMedium);
		DeleteObject(uiFontSmall);

		if (hWndMain)
			DestroyWindow(hWndMain);

	}

	void MainWindow::UpdateUIActivation()
	{
		if (glControl.IsBusy() || helpWindow.IsVisible())
		{
			if (subWindowMap[currentState]->IsActive())
			{
				subWindowMap[currentState]->Deactivate();
			}
		}
		else if (!subWindowMap[currentState]->IsActive())
		{
			subWindowMap[currentState]->Activate();
			UpdateModeButtons();
		}
	}

	int MainWindow::GetGpuMemory()
	{
		return totalGpuMemory;
	}

	void MainWindow::SetGpuMemory(int _gpuMemory)
	{
		DebugUtility::DbgOut(L"MainWindow::SetGpuMemory::", _gpuMemory);
		totalGpuMemory = _gpuMemory;
	}

	bool MainWindow::IsValidState(WindowState _state)
	{
		auto foundWindow = subWindowMap.find(_state);
		if (foundWindow != subWindowMap.end())
			return true;
		else
		{
			DebugUtility::DbgOut(L"MainWindow::IsValidState::State not found in Window Map");
			return false;
		}
	}

	void MainWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"MainWindow::CleanUp");
		glControl.CleanUp();

		DebugUtility::DbgOut(L"MainWindow::CleanUpWindows");

		for (auto &subWindow : subWindowMap)
		{
			DebugUtility::DbgOut(L"MainWindow::CleanUp::SubWindowCleanUp");
			subWindow.second->CleanUp();
		}
		subWindowMap.clear();
		DebugUtility::DbgOut(L"MainWindow::CleanUpButtonLayout");
		for (auto style : buttonLayout)
			style.second.CleanUp();
		buttonLayout.clear();
	}

#pragma endregion Parent Window Thread
}