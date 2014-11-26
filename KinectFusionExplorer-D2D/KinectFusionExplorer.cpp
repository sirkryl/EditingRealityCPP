//------------------------------------------------------------------------------
// <copyright file="KinectFusionExplorer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// System includes
#include "stdafx.h"
#include <chrono>
#include <thread>
#include <map>
// Project includes
#include "KFKeys.h"
#include "resource.h"
#include "KinectFusionExplorer.h"
#include "KinectFusionProcessorFrame.h"
#include "KinectFusionHelper.h"

TCHAR KFKeys::kp[256] = { 0 };

LRESULT CALLBACK FusionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

//OpenGLWin openGLWin;
bool initDone = false;
float ratio = 480.0f / 640.0f;
bool debugMode = true;
bool resumed = false;
bool interactionMode = false;
HWND hWndApp;
HWND hButtonTestOne, hButtonTestTwo, hButtonInteractionMode;
HWND hButtonStart;
HWND hButtonResetReconstruction;
HWND fusionDebugHandle;
HWND hStatusText, hStatusTextSmall, hCountdownText, hHelpText;

std::vector<HWND> fusionUiElements;

HBRUSH bDefaultBrush, bPressedBrush;
HBRUSH bGreenBrush, bRedBrush;
HBRUSH bGreenPressedBrush, bRedPressedBrush;
HBRUSH bBackground = CreateSolidBrush(RGB(30, 30, 30));
HPEN bDefaultPen, bPressedPen, bInactivePen;
HFONT guiFont, countdownFont, smallFont, buttonFont;

void(*ptrStartOpenGL)(int);
#define MIN_DEPTH_DISTANCE_MM 350   // Must be greater than 0
#define MAX_DEPTH_DISTANCE_MM 8000
#define MIN_INTEGRATION_WEIGHT 1    // Must be greater than 0
#define MAX_INTEGRATION_WEIGHT 1000

#define WM_FRAMEREADY           (WM_USER + 0)
#define WM_UPDATESENSORSTATUS   (WM_USER + 1)


/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
//int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
void StartKinectFusion(HWND parent, HINSTANCE hInstance, void(*a_ptrStartOpenGL)(int), CKinectFusionExplorer*& expl, HWND &fusionHandle)
{
	
	CKinectFusionExplorer application;
	expl = &application;
	ptrStartOpenGL = a_ptrStartOpenGL;
	application.Run(parent, hInstance, 0, fusionHandle);// nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CKinectFusionExplorer::CKinectFusionExplorer() :
m_hWnd(nullptr),
m_pD2DFactory(nullptr),
m_pDrawReconstruction(nullptr),
m_pDrawTrackingResiduals(nullptr),
m_pDrawDepth(nullptr),
m_bSavingMesh(false),
m_saveMeshFormat(Stl),
m_bInitializeError(false),
m_pSensorChooserUI(nullptr),
m_bColorCaptured(false),
m_bUIUpdated(false)
{
}
/// <summary>
/// Destructor
/// </summary>
CKinectFusionExplorer::~CKinectFusionExplorer()
{
	// clean up sensor chooser UI
	SAFE_DELETE(m_pSensorChooserUI);

	// clean up Direct2D renderer
	SAFE_DELETE(m_pDrawReconstruction);

	// clean up Direct2D renderer
	SAFE_DELETE(m_pDrawTrackingResiduals);

	// clean up Direct2D renderer
	SAFE_DELETE(m_pDrawDepth);

	// clean up Direct2D
	SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CKinectFusionExplorer::Run(HWND parent, HINSTANCE hInstance, int nCmdShow, HWND &fusionHandle)
{
	MSG       msg = { 0 };
	WNDCLASS  wc = { 0 };


	// Dialog custom window class
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	wc.hbrBackground = bBackground;
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"KinectFusionExplorerAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	hWndApp = CreateDialogParamW(
		hInstance,
		MAKEINTRESOURCE(IDD_APP),
		parent,
		(DLGPROC)CKinectFusionExplorer::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	fusionHandle = hWndApp;
	fusionDebugHandle = CreateDialog(hInstance, MAKEINTRESOURCE(IDD_ADVANCED_OPTIONS), hWndApp, (DLGPROC)FusionDebugRouter);
	ShowWindow(fusionDebugHandle, SW_HIDE);
	// Show window
	guiFont = CreateFont(40, 0, 0, 0, FW_REGULAR, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");
	buttonFont = CreateFont(40, 0, 0, 0, FW_REGULAR, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");
	countdownFont = CreateFont(300, 0, 0, 0, FW_LIGHT, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");
	smallFont = CreateFont(20, 0, 0, 0, FW_LIGHT, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");

	bDefaultBrush = CreateSolidBrush(RGB(20, 20, 20));
	bPressedBrush = CreateSolidBrush(RGB(40, 40, 40));
	bGreenBrush = CreateSolidBrush(RGB(0, 170, 0));
	bGreenPressedBrush = CreateSolidBrush(RGB(0, 100, 0));
	bRedBrush = CreateSolidBrush(RGB(170, 0, 0));
	bRedPressedBrush = CreateSolidBrush(RGB(100, 0, 0));
	bDefaultPen = CreatePen(PS_SOLID, 2, RGB(100, 100, 100));
	bPressedPen = CreatePen(PS_SOLID, 2, RGB(180, 180, 180));
	bInactivePen = CreatePen(PS_SOLID, 1, RGB(50, 50, 50));
	hButtonInteractionMode = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWndApp, (HMENU)IDC_BUTTON_INTERACTION_MODE, hInstance, 0);
	hButtonTestOne = CreateWindowEx(0, L"Button", L"Load Test Interaction", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 250, 150, 50, hWndApp, (HMENU)IDC_BUTTON_TEST_INTERACTION, hInstance, 0);
	hButtonTestTwo = CreateWindowEx(0, L"Button", L"Test Open GL", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_BUTTON_TEST_OPENGL, hInstance, 0);
	hButtonResetReconstruction = CreateWindowEx(0, L"Button", L"RESET", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_BUTTON_RESET_RECONSTRUCTION, hInstance, 0);
	ShowWindow(parent, SW_SHOWMAXIMIZED);
	ShowWindow(hWndApp, SW_SHOW);
	//HANDLE thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ThreadMain, (LPVOID)hWndApp, 0, 0);
	
	hButtonStart = CreateWindowEx(0, L"Button", L"START", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_BUTTON_START, hInstance, 0);

	hStatusText = GetDlgItem(hWndApp, IDC_FUSION_STATIC_STATUS);
	
	SetDlgItemText(hWndApp, IDC_FUSION_STATIC_STATUS, L"Let's start with scanning your scene.");
	SendMessage(hStatusText, WM_SETFONT, (WPARAM)guiFont, TRUE);

	hStatusTextSmall = GetDlgItem(hWndApp, IDC_FUSION_STATIC_STATUS_SMALL);

	SetDlgItemText(hWndApp, IDC_FUSION_STATIC_STATUS_SMALL, L"Please move the sensor slowly and smoothly.");
	SendMessage(hStatusTextSmall, WM_SETFONT, (WPARAM)smallFont, TRUE);
	//ShowWindow(hStatusText, SW_HIDE);

	hCountdownText = GetDlgItem(hWndApp, IDC_FUSION_STATIC_COUNTDOWN);
	SetDlgItemText(hWndApp, IDC_FUSION_STATIC_COUNTDOWN, L"3");
	SendMessage(hCountdownText, WM_SETFONT, (WPARAM)countdownFont, TRUE);

	hHelpText = GetDlgItem(hWndApp, IDC_FUSION_STATIC_HELP);

	SetDlgItemText(hWndApp, IDC_FUSION_STATIC_HELP, L"Press 'Reset' to start over and 'Finished' if you're satisfied with your scan.");
	SendMessage(hHelpText, WM_SETFONT, (WPARAM)smallFont, TRUE);

	fusionUiElements.push_back(hButtonTestOne);
	fusionUiElements.push_back(hButtonTestTwo);
	fusionUiElements.push_back(hButtonInteractionMode);
	fusionUiElements.push_back(hButtonResetReconstruction);
	fusionUiElements.push_back(hButtonStart);
	fusionUiElements.push_back(hStatusText);
	fusionUiElements.push_back(hStatusTextSmall);
	fusionUiElements.push_back(hHelpText);
	fusionUiElements.push_back(hCountdownText);
	fusionUiElements.push_back(GetDlgItem(m_hWnd, IDC_RECONSTRUCTION_VIEW));

	SetWindowState(START);

	//SetForegroundWindow(parent);
	// Main message loop
	while (WM_QUIT != msg.message)
	{
		if (GetMessage(&msg, nullptr, 0, 0))
		{
			// If a dialog message will be taken care of by the dialog proc
			if ((hWndApp != nullptr) && IsDialogMessageW(hWndApp, &msg))
			{
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}
	//if (interactionMode)
		//PostThreadMessage(openGLWin.GetThreadID(), WM_QUIT, (WPARAM)NULL, (LPARAM)NULL);
		//WaitForSingleObject(openGLWin.interactionThread, INFINITE);
		//CloseHandle(openGLWin.interactionThread);
	return static_cast<int>(msg.wParam);
}

LRESULT CALLBACK FusionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	CKinectFusionExplorer* pThis = reinterpret_cast<CKinectFusionExplorer*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA));
	//cDebug::DbgOut(L"msg?");
	switch (message)
	{
	case WM_COMMAND:
		if (pThis)
			pThis->ProcessUI(wParam, lParam);
		break;
	case  WM_HSCROLL:
		//cDebug::DbgOut(L"slider update");
		if (pThis)
			pThis->UpdateHSliders();
		break;
	}
	return FALSE;
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CKinectFusionExplorer::MessageRouter(
	HWND hWnd,
	UINT uMsg,
	WPARAM wParam,
	LPARAM lParam)
{
	CKinectFusionExplorer* pThis = nullptr;

	if (WM_INITDIALOG == uMsg && !initDone)
	{
		pThis = reinterpret_cast<CKinectFusionExplorer*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
		initDone = true;
	}
	else
	{
		pThis = reinterpret_cast<CKinectFusionExplorer*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
			
		if (pThis->GetWindowState() == SCAN)
			return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
		else
			return pThis->StartProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

bool CKinectFusionExplorer::DrawButton(LPARAM lParam)
{
	LPDRAWITEMSTRUCT Item;
	Item = (LPDRAWITEMSTRUCT)lParam;
	SelectObject(Item->hDC, buttonFont);
	FillRect(Item->hDC, &Item->rcItem, bBackground);
	SelectObject(Item->hDC, bDefaultBrush);
	if (!IsWindowEnabled(Item->hwndItem))
	{
		SetTextColor(Item->hDC, RGB(50, 50, 50));
		SelectObject(Item->hDC, bInactivePen);
	}
	else if (Item->itemState & ODS_SELECTED)
	{
		SetTextColor(Item->hDC, RGB(245, 245, 245));
		if (Item->hwndItem == hButtonInteractionMode)
		{
			SelectObject(Item->hDC, bGreenPressedBrush);
			SelectObject(Item->hDC, bDefaultPen);
		}
		else if (Item->hwndItem == hButtonResetReconstruction)
		{
			SelectObject(Item->hDC, bRedPressedBrush);
			SelectObject(Item->hDC, bDefaultPen);
		}
		else
		{
			SelectObject(Item->hDC, bPressedBrush);
			SelectObject(Item->hDC, bPressedPen);
		}
	}
	else
	{
		if (Item->hwndItem == hButtonInteractionMode)
		{
			SelectObject(Item->hDC, bGreenBrush);
		}
		else if (Item->hwndItem == hButtonResetReconstruction)
		{
			SelectObject(Item->hDC, bRedBrush);
		}
		SetTextColor(Item->hDC, RGB(240, 240, 240));
		SelectObject(Item->hDC, bDefaultPen);
	}
	SetBkMode(Item->hDC, TRANSPARENT);
	if (Item->hwndItem == hButtonInteractionMode || Item->hwndItem == hButtonResetReconstruction)
		RoundRect(Item->hDC, Item->rcItem.left, Item->rcItem.top, Item->rcItem.right, Item->rcItem.bottom, 500, 500);
	else
		RoundRect(Item->hDC, Item->rcItem.left, Item->rcItem.top, Item->rcItem.right, Item->rcItem.bottom, 20, 20);
	int len;
	len = GetWindowTextLength(Item->hwndItem);
	LPSTR lpBuff;
	lpBuff = new char[len + 1];
	GetWindowTextA(Item->hwndItem, lpBuff, len + 1);

	DrawTextA(Item->hDC, lpBuff, len, &Item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

	return TRUE;
}

LRESULT CALLBACK CKinectFusionExplorer::StartProc(
	HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HandleKeyInput();

	switch (message)
	{
	case WM_INITDIALOG:						  // Bind application window handle
		
		m_hWnd = hWnd;
		break;
		// Handle button press
	case WM_COMMAND:
		OutputDebugString(L"COMMAND");
		ProcessUI(wParam, lParam);
		break;

		// Handle sliders
	case  WM_HSCROLL:
		UpdateHSliders();
		break;
	case WM_CTLCOLORDLG:
		return (LRESULT)bBackground;
		break;
	case WM_CTLCOLORSTATIC:
		if ((HWND)lParam == hStatusText ||
			(HWND)lParam == hCountdownText ||
			(HWND)lParam == hStatusTextSmall ||
			(HWND)lParam == hHelpText)
		{

			HDC hdc = reinterpret_cast<HDC>(wParam);
			SetBkMode((HDC)wParam, TRANSPARENT);
			SetTextColor(hdc, RGB(255, 255, 255));

			return (LRESULT)bBackground;
		}
		break;
	case WM_UPDATESENSORSTATUS:
		if (m_pSensorChooserUI != nullptr)
		{
			m_pSensorChooserUI->UpdateSensorStatus(static_cast<DWORD>(wParam));
		}
		break;
	case WM_SIZE:
		MoveUIOnResize();
		break;
	case WM_DRAWITEM:
	{
		if (IDC_BUTTON_INTERACTION_MODE == LOWORD(wParam)
			|| IDC_BUTTON_TEST_INTERACTION == LOWORD(wParam)
			|| IDC_BUTTON_TEST_OPENGL == LOWORD(wParam)
			|| IDC_BUTTON_RESET_RECONSTRUCTION == LOWORD(wParam)
			|| IDC_BUTTON_START == LOWORD(wParam))
		{
			return DrawButton(lParam);
			
		}
	}
		break;
	}


	return FALSE;
}


/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CKinectFusionExplorer::DlgProc(
	HWND hWnd,
	UINT message,
	WPARAM wParam,
	LPARAM lParam)
{
	HandleKeyInput();

	switch (message)
	{
	case WM_INITDIALOG:
	{
		// Bind application window handle
		m_hWnd = hWnd;

		// Init Direct2D
		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		int width = m_params.m_cDepthWidth;
		int height = m_params.m_cDepthHeight;

		// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		// We'll use this to draw the data we receive from the Kinect to the screen
		m_pDrawDepth = new ImageRenderer();
		HRESULT hr = m_pDrawDepth->Initialize(
			GetDlgItem(m_hWnd, IDC_DEPTH_VIEW),
			m_pD2DFactory,
			width,
			height,
			width * sizeof(long));

		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
			m_bInitializeError = true;
		}

		m_pDrawReconstruction = new ImageRenderer();
		hr = m_pDrawReconstruction->Initialize(
			GetDlgItem(m_hWnd, IDC_RECONSTRUCTION_VIEW),
			m_pD2DFactory,
			width,
			height,
			width * sizeof(long));

		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
			m_bInitializeError = true;
		}

		m_pDrawTrackingResiduals = new ImageRenderer();
		hr = m_pDrawTrackingResiduals->Initialize(
			GetDlgItem(m_hWnd, IDC_TRACKING_RESIDUALS_VIEW),
			m_pD2DFactory,
			width,
			height,
			width * sizeof(long));

		if (FAILED(hr))
		{
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
			m_bInitializeError = true;
		}

		if (FAILED(m_processor.SetWindow(m_hWnd, WM_FRAMEREADY, WM_UPDATESENSORSTATUS)) ||
			FAILED(m_processor.SetParams(m_params)) ||
			FAILED(m_processor.StartProcessing()))
		{
			m_bInitializeError = true;
		}

		m_saveMeshFormat = m_params.m_saveMeshType;
	}
		break;

	case WM_DESTROY:
		// Quit the main message pump
		DestroyWindow(fusionDebugHandle);
		DeleteObject(bBackground);
		DeleteObject(bDefaultBrush);
		DeleteObject(bGreenBrush);
		DeleteObject(bGreenPressedBrush);
		DeleteObject(bRedBrush);
		DeleteObject(bRedPressedBrush);
		DeleteObject(bDefaultPen);
		DeleteObject(bInactivePen);
		DeleteObject(bPressedBrush);
		DeleteObject(bPressedPen);
		DeleteObject(guiFont);
		DeleteObject(buttonFont);
		DeleteObject(smallFont);
		DeleteObject(countdownFont);
		m_processor.StopProcessing();
		PostQuitMessage(0);
		break;

		// Handle button press
	case WM_COMMAND:
		ProcessUI(wParam, lParam);
		break;

		// Handle sliders
	case  WM_HSCROLL:
		UpdateHSliders();
		break;
	case WM_CTLCOLORDLG:
		return (LRESULT)bBackground;
		break;
	case WM_NOTIFY:
	{
		const NMHDR* pNMHeader = reinterpret_cast<const NMHDR*>(lParam);
		if (pNMHeader->code == NSCN_REFRESH && pNMHeader->idFrom == IDC_SENSORCHOOSER)
		{
			m_processor.ResolveSensorConflict();
		}
	}
		break;
	case WM_CTLCOLORSTATIC:
		if ((HWND)lParam == hStatusText ||
			(HWND)lParam == hCountdownText ||
			(HWND)lParam == hStatusTextSmall ||
			(HWND)lParam == hHelpText)
		{
			
			HDC hdc = reinterpret_cast<HDC>(wParam);
			SetBkMode((HDC)wParam, TRANSPARENT);
			SetTextColor(hdc, RGB(255, 255, 255));

			return (LRESULT)bBackground;
		}
		break;
	case WM_FRAMEREADY:
		HandleCompletedFrame();
		break;

	case WM_UPDATESENSORSTATUS:
		if (m_pSensorChooserUI != nullptr)
		{
			m_pSensorChooserUI->UpdateSensorStatus(static_cast<DWORD>(wParam));
		}
		break;
	case WM_KEYDOWN:
		//cDebug::DbgOut(L"Keydown indeed");
		if (wParam == 'S')
		{
			//cDebug::DbgOut(L"S down indeed");
			if (IsWindowVisible(fusionDebugHandle))
				ShowWindow(fusionDebugHandle, SW_HIDE);
			else
			{
				ShowWindow(fusionDebugHandle, SW_SHOW);
				InitializeUIControls();
			}

		}
		break;
	case WM_SIZE:
	{
		MoveUIOnResize();
		
	}
		break;
	case WM_DRAWITEM:
	{

		if (IDC_BUTTON_INTERACTION_MODE == LOWORD(wParam)
			|| IDC_BUTTON_TEST_INTERACTION == LOWORD(wParam)
			|| IDC_BUTTON_TEST_OPENGL == LOWORD(wParam)
			|| IDC_BUTTON_RESET_RECONSTRUCTION == LOWORD(wParam)
			|| IDC_BUTTON_START == LOWORD(wParam))
		{
			return DrawButton(lParam);
		}
	}
		break;
	}
	

	return FALSE;
}

/// <summary>
/// Handle a completed frame from the Kinect Fusion processor.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
void CKinectFusionExplorer::HandleCompletedFrame()
{
	KinectFusionProcessorFrame const* pFrame = nullptr;

	// Flush any extra WM_FRAMEREADY messages from the queue
	MSG msg;
	while (PeekMessage(&msg, m_hWnd, WM_FRAMEREADY, WM_FRAMEREADY, PM_REMOVE)) {}

	m_processor.LockFrame(&pFrame);

	if (!m_bSavingMesh && !interactionMode) // don't render while a mesh is being saved
	{
		if (m_processor.IsVolumeInitialized())
		{
			m_pDrawDepth->Draw(pFrame->m_pDepthRGBX, pFrame->m_cbImageSize);
			m_pDrawReconstruction->Draw(pFrame->m_pReconstructionRGBX, pFrame->m_cbImageSize);
			m_pDrawTrackingResiduals->Draw(pFrame->m_pTrackingDataRGBX, pFrame->m_cbImageSize);
		}

		SetStatusMessage(pFrame->m_statusMessage);
		SetFramesPerSecond(pFrame->m_fFramesPerSecond);
	}

	if (pFrame->m_bIntegrationResumed)
	{
		m_params.m_bPauseIntegration = false;
		CheckDlgButton(m_hWnd, IDC_CHECK_PAUSE_INTEGRATION, BST_UNCHECKED);
		m_processor.SetParams(m_params);
	}
	else if (m_processor.IsCameraPoseFinderAvailable() && !m_params.m_bPauseIntegration)
	{
		m_params.m_bPauseIntegration = true;
		CheckDlgButton(m_hWnd, IDC_CHECK_PAUSE_INTEGRATION, BST_CHECKED);
		m_processor.SetParams(m_params);
	}

	if (!m_bUIUpdated && m_processor.IsVolumeInitialized())
	{
		const int Mebi = 1024 * 1024;

		// We now create both a color and depth volume, doubling the required memory, so we restrict
		// which resolution settings the user can choose when the graphics card is limited in memory.
		if (pFrame->m_deviceMemory <= 1 * Mebi)  // 1GB
		{
			// Disable 640 voxel resolution in all axes - cards with only 1GB cannot handle this
			HWND hButton = GetDlgItem(m_hWnd, IDC_VOXELS_X_640);
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_640);
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Z_640);
			EnableWindow(hButton, FALSE);
			if (Is64BitApp() == FALSE)
			{
				// Also disable 512 voxel resolution in one arbitrary axis on 32bit machines
				hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_512);
				EnableWindow(hButton, FALSE);
			}
		}
		else if (pFrame->m_deviceMemory <= 2 * Mebi)  // 2GB
		{
			if (Is64BitApp() == FALSE)
			{
				// Disable 640 voxel resolution in one arbitrary axis on 32bit machines
				HWND hButton = GetDlgItem(m_hWnd, IDC_VOXELS_Y_640);
				EnableWindow(hButton, FALSE);
			}
			// True 64 bit apps seem to be more able to cope with large volume sizes.
		}

		m_bUIUpdated = true;
	}

	m_bColorCaptured = pFrame->m_bColorCaptured;

	m_processor.UnlockFrame();
}

/// <summary>
/// Save Mesh to disk.
/// </summary>
/// <param name="mesh">The mesh to save.</param>
/// <returns>indicates success or failure</returns>
HRESULT CKinectFusionExplorer::SaveMeshFile(INuiFusionColorMesh* pMesh, KinectFusionMeshTypes saveMeshType)
{
	HRESULT hr = S_OK;

	if (nullptr == pMesh)
	{
		return E_INVALIDARG;
	}

	//temporary interaction start
	std::wstring fileName = L"data\\models\\output.ply";
	LPOLESTR fileString = W2OLE((wchar_t*)fileName.c_str());
	hr = WriteAsciiPlyMeshFile(pMesh, fileString, true, m_bColorCaptured);
	/*interactionMode = true;
	if (openGLWin.StartOpenGLThread(m_hWnd, GetModuleHandle(NULL), &m_processor))
	{
		return hr;
	}
	interactionMode = false;*/
	//interactionThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ThreadMain, (LPVOID)m_hWnd, 0, 0);
	//return hr;
	//temporary interaction end

	CComPtr<IFileSaveDialog> pSaveDlg;

	// Create the file save dialog object.
	hr = pSaveDlg.CoCreateInstance(__uuidof(FileSaveDialog));

	if (FAILED(hr))
	{
		return hr;
	}

	// Set the dialog title
	hr = pSaveDlg->SetTitle(L"Save Kinect Fusion Mesh");
	if (SUCCEEDED(hr))
	{
		// Set the button text
		hr = pSaveDlg->SetOkButtonLabel(L"Save");
		if (SUCCEEDED(hr))
		{
			// Set a default filename
			if (Stl == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"MeshedReconstruction.stl");
			}
			else if (Obj == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"MeshedReconstruction.obj");
			}
			else if (Ply == saveMeshType)
			{
				hr = pSaveDlg->SetFileName(L"MeshedReconstruction.ply");
			}

			if (SUCCEEDED(hr))
			{
				// Set the file type extension
				if (Stl == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"stl");
				}
				else if (Obj == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"obj");
				}
				else if (Ply == saveMeshType)
				{
					hr = pSaveDlg->SetDefaultExtension(L"ply");
				}

				if (SUCCEEDED(hr))
				{
					// Set the file type filters
					if (Stl == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Stl mesh files", L"*.stl" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}
					else if (Obj == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Obj mesh files", L"*.obj" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}
					else if (Ply == saveMeshType)
					{
						COMDLG_FILTERSPEC allPossibleFileTypes[] = {
							{ L"Ply mesh files", L"*.ply" },
							{ L"All files", L"*.*" }
						};

						hr = pSaveDlg->SetFileTypes(
							ARRAYSIZE(allPossibleFileTypes),
							allPossibleFileTypes);
					}

					if (SUCCEEDED(hr))
					{
						// Show the file selection box
						hr = pSaveDlg->Show(m_hWnd);

						// Save the mesh to the chosen file.
						if (SUCCEEDED(hr))
						{
							CComPtr<IShellItem> pItem;
							hr = pSaveDlg->GetResult(&pItem);

							if (SUCCEEDED(hr))
							{
								LPOLESTR pwsz = nullptr;
								hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pwsz);

								if (SUCCEEDED(hr))
								{
									SetStatusMessage(L"Saving mesh file, please wait...");
									SetCursor(LoadCursor(nullptr, MAKEINTRESOURCE(IDC_WAIT)));

									if (Stl == saveMeshType)
									{
										hr = WriteBinarySTLMeshFile(pMesh, pwsz);
									}
									else if (Obj == saveMeshType)
									{
										hr = WriteAsciiObjMeshFile(pMesh, pwsz);
									}
									else if (Ply == saveMeshType)
									{
										hr = WriteAsciiPlyMeshFile(pMesh, pwsz, true, m_bColorCaptured);
									}

									CoTaskMemFree(pwsz);
								}
							}
						}
					}
				}
			}
		}
	}

	return hr;
}

///////////////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Initialize the UI
/// </summary>
void CKinectFusionExplorer::InitializeUIControls()
{
	// Create NuiSensorChooser UI control
	RECT rc;
	GetClientRect(m_hWnd, &rc);

	POINT ptCenterTop;
	ptCenterTop.x = (rc.right - rc.left) / 2;
	ptCenterTop.y = 0;

	// Create the sensor chooser UI control to show sensor status
	m_pSensorChooserUI = new NuiSensorChooserUI(m_hWnd, IDC_SENSORCHOOSER, ptCenterTop);
	m_pSensorChooserUI->UpdateSensorStatus(NuiSensorChooserStatusInitializing);

	// Set slider ranges
	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SLIDER_DEPTH_MIN,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

	SendDlgItemMessage(fusionDebugHandle,
		IDC_SLIDER_DEPTH_MAX,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_INTEGRATION_WEIGHT_SLIDER,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_INTEGRATION_WEIGHT, MAX_INTEGRATION_WEIGHT));

	// Set slider positions
	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SLIDER_DEPTH_MAX,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fMaxDepthThreshold * 1000);

	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SLIDER_DEPTH_MIN,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fMinDepthThreshold * 1000);

	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_INTEGRATION_WEIGHT_SLIDER,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_cMaxIntegrationWeight);

	// Set intermediate slider tics at meter intervals
	for (int i = 1; i<(MAX_DEPTH_DISTANCE_MM / 1000); i++)
	{
		SendDlgItemMessage(fusionDebugHandle, IDC_SLIDER_DEPTH_MAX, TBM_SETTIC, 0, i * 1000);
		SendDlgItemMessage(fusionDebugHandle, IDC_SLIDER_DEPTH_MIN, TBM_SETTIC, 0, i * 1000);
	}

	// Update slider text
	WCHAR str[MAX_PATH];
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_MIN_DIST_TEXT, str);
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_MAX_DIST_TEXT, str);

	swprintf_s(str, ARRAYSIZE(str), L"%d", m_params.m_cMaxIntegrationWeight);
	SetDlgItemText(fusionDebugHandle, IDC_INTEGRATION_WEIGHT_TEXT, str);

	// Set the radio buttons for Volume Parameters
	switch ((int)m_params.m_reconstructionParams.voxelsPerMeter)
	{
	case 768:
		CheckDlgButton(fusionDebugHandle, IDC_VPM_768, BST_CHECKED);
		break;
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_VPM_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_VPM_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_VPM_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_VPM_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_VPM_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_VPM_256, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountX)
	{
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_X_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_X_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_X_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_X_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_X_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_X_384, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountY)
	{
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Y_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Y_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Y_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Y_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Y_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Y_384, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountZ)
	{
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Z_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Z_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Z_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Z_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Z_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_VOXELS_Z_384, BST_CHECKED);
		break;
	}

	if (Stl == m_saveMeshFormat)
	{
		CheckDlgButton(fusionDebugHandle, IDC_MESH_FORMAT_STL_RADIO, BST_CHECKED);
	}
	else if (Obj == m_saveMeshFormat)
	{
		CheckDlgButton(fusionDebugHandle, IDC_MESH_FORMAT_OBJ_RADIO, BST_CHECKED);
	}
	else if (Ply == m_saveMeshFormat)
	{
		CheckDlgButton(fusionDebugHandle, IDC_MESH_FORMAT_PLY_RADIO, BST_CHECKED);
	}

	if (m_params.m_bCaptureColor)
	{
		CheckDlgButton(fusionDebugHandle, IDC_CHECK_CAPTURE_COLOR, BST_CHECKED);
	}

	if (m_params.m_bAutoFindCameraPoseWhenLost)
	{
		CheckDlgButton(fusionDebugHandle, IDC_CHECK_CAMERA_POSE_FINDER, BST_CHECKED);
	}
}


int WINAPI CountdownThread()
{
	
	int secondTimer = 3;
	CKinectFusionExplorer* pThis = reinterpret_cast<CKinectFusionExplorer*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA));
	pThis->SetWindowState(COUNTDOWN);
	//pThis->HideAllUIElements();
	ShowWindow(hCountdownText, SW_SHOW);
	//SetDlgItemText(hWndApp, IDC_FUSION_STATIC_COUNTDOWN, L"3");
	//CKinectFusionExplorer* explorer = reinterpret_cast<CKinectFusionExplorer*>(exp);
	while (secondTimer > 0)
	{
		SetDlgItemText(hWndApp, IDC_FUSION_STATIC_COUNTDOWN, std::to_wstring(secondTimer).c_str());
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		secondTimer--;
	}
	//OutputDebugString(L"wel")
	pThis->SetWindowState(SCAN);

	return 0;
}

/// <summary>
/// Process the UI inputs
/// </summary>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
void CKinectFusionExplorer::ProcessUI(WPARAM wParam, LPARAM lParam)
{
	// If it was for the near mode control and a clicked event, change near mode
	if (IDC_CHECK_NEARMODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		m_params.m_bNearMode = !m_params.m_bNearMode;
	}
	// If it was for the display surface normals toggle this variable
	if (IDC_CHECK_CAPTURE_COLOR == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle capture color
		m_params.m_bCaptureColor = !m_params.m_bCaptureColor;
	}
	// If it was for the display surface normals toggle this variable
	if (IDC_CHECK_MIRROR_DEPTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle depth mirroring
		m_params.m_bMirrorDepthFrame = !m_params.m_bMirrorDepthFrame;

		m_processor.ResetReconstruction();
	}
	if (IDC_CHECK_CAMERA_POSE_FINDER == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bAutoFindCameraPoseWhenLost = !m_params.m_bAutoFindCameraPoseWhenLost;

		if (!m_params.m_bAutoFindCameraPoseWhenLost)
		{
			// Force pause integration off when unchecking use of camera pose finder
			m_params.m_bPauseIntegration = false;
		}
	}
	// If it was the reset button clicked, clear the volume
	if (IDC_BUTTON_RESET_RECONSTRUCTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_processor.ResetReconstruction();
	}
	if (IDC_BUTTON_TEST_INTERACTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		FinishScan(1);
		/*interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(reinterpret_cast<CKinectFusionExplorer*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA)), 1);*/
		//ptrStartOpenGL(&m_processor, 1);

		//openGLWin.testMode = 1;
		//openGLWin.StartOpenGLThread(m_hWnd, GetModuleHandle(NULL));

	}
	if (IDC_BUTTON_TEST_OPENGL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		FinishScan(2);
		/*interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(reinterpret_cast<CKinectFusionExplorer*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA)), 2);*/
		//openGLWin.testMode = 2;
		//openGLWin.StartOpenGLThread(m_hWnd, GetModuleHandle(NULL));
	}
	// If it was the mesh button clicked, mesh the volume and save
	if (IDC_BUTTON_MESH_RECONSTRUCTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetStatusMessage(L"Creating and saving mesh of reconstruction, please wait...");
		m_bSavingMesh = true;

		// Pause integration while we're saving
		bool wasPaused = m_params.m_bPauseIntegration;
		m_params.m_bPauseIntegration = true;
		m_processor.SetParams(m_params);

		INuiFusionColorMesh *mesh = nullptr;
		HRESULT hr = m_processor.CalculateMesh(&mesh, 3);
		if (SUCCEEDED(hr))
		{
			// Save mesh
			hr = SaveMeshFile(mesh, m_saveMeshFormat);

			if (SUCCEEDED(hr))
			{
				SetStatusMessage(L"Saved Kinect Fusion mesh.");
			}
			else if (HRESULT_FROM_WIN32(ERROR_CANCELLED) == hr)
			{
				SetStatusMessage(L"Mesh save canceled.");
			}
			else
			{
				SetStatusMessage(L"Error saving Kinect Fusion mesh!");
			}

			// Release the mesh
			SafeRelease(mesh);
		}
		else
		{
			SetStatusMessage(L"Failed to create mesh of reconstruction.");
		}

		// Restore pause state of integration
		m_params.m_bPauseIntegration = wasPaused;
		m_processor.SetParams(m_params);

		m_bSavingMesh = false;
	}
	if (IDC_BUTTON_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&CountdownThread, 0, 0, 0);
		//SetWindowState(SCAN);
	}
	if (IDC_BUTTON_INTERACTION_MODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		FinishScan(0);
	}
	if (IDC_CHECK_PAUSE_INTEGRATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle the pause state of the reconstruction
		m_params.m_bPauseIntegration = !m_params.m_bPauseIntegration;
	}
	if (IDC_VPM_768 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 768.0f;
	}
	if (IDC_VPM_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 640.0f;
	}
	if (IDC_VPM_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 512.0f;
	}
	if (IDC_VPM_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 384.0f;
	}
	if (IDC_VPM_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;
	}
	if (IDC_VPM_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 128.0f;
	}
	if (IDC_VOXELS_X_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 640;
	}
	if (IDC_VOXELS_X_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 512;
	}
	if (IDC_VOXELS_X_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 384;
	}
	if (IDC_VOXELS_X_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 256;
	}
	if (IDC_VOXELS_X_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 128;
	}
	if (IDC_VOXELS_Y_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 640;
	}
	if (IDC_VOXELS_Y_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 512;
	}
	if (IDC_VOXELS_Y_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 384;
	}
	if (IDC_VOXELS_Y_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 256;
	}
	if (IDC_VOXELS_Y_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 128;
	}
	if (IDC_VOXELS_Z_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 640;
	}
	if (IDC_VOXELS_Z_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 512;
	}
	if (IDC_VOXELS_Z_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 384;
	}
	if (IDC_VOXELS_Z_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 256;
	}
	if (IDC_VOXELS_Z_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 128;
	}
	if (IDC_MESH_FORMAT_STL_RADIO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Stl;
	}
	if (IDC_MESH_FORMAT_OBJ_RADIO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Obj;
	}
	if (IDC_MESH_FORMAT_PLY_RADIO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Ply;
	}

	m_processor.SetParams(m_params);
}

void CKinectFusionExplorer::FinishScan(int testMode)
{
	if (testMode == 0)
	{
		//SetFocus(hWndApp);
		SetForegroundWindow(hWndApp);
		//openGLWin.testMode = 0;
		//SetStatusMessage(L"Creating mesh and entering interaction mode. Please wait...");
		m_bSavingMesh = true;

		// Pause integration while we're saving
		bool wasPaused = m_params.m_bPauseIntegration;
		m_params.m_bPauseIntegration = true;
		m_processor.SetParams(m_params);

		INuiFusionColorMesh *mesh = nullptr;
		HRESULT hr = m_processor.CalculateMesh(&mesh, 2);

		if (SUCCEEDED(hr))
		{
			// Save mesh
			std::wstring fileName = L"data\\models\\output.ply";
			LPOLESTR fileString = W2OLE((wchar_t*)fileName.c_str());
			hr = WriteAsciiPlyMeshFile(mesh, fileString, true, m_bColorCaptured);
			interactionMode = true;
			ShowWindow(hWndApp, SW_HIDE);
			ptrStartOpenGL(0);
			hr = S_OK;
			// Release the mesh
			SafeRelease(mesh);
		}
		else
		{
			SetStatusMessage(L"Failed to enter interaction mode.");
		}

		// Restore pause state of integration
		m_params.m_bPauseIntegration = wasPaused;
		m_processor.SetParams(m_params);

		m_bSavingMesh = false;
	}
	else if (testMode == 1)
	{
		interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(1);
	}
	else if (testMode == 2)
	{
		interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(2);
	}
	
}

/// <summary>
/// Update the internal variable values from the UI Horizontal sliders.
/// </summary>
void CKinectFusionExplorer::UpdateHSliders()
{
	int mmMinPos = (int)SendDlgItemMessage(fusionDebugHandle, IDC_SLIDER_DEPTH_MIN, TBM_GETPOS, 0, 0);

	if (mmMinPos >= MIN_DEPTH_DISTANCE_MM && mmMinPos <= MAX_DEPTH_DISTANCE_MM)
	{
		m_params.m_fMinDepthThreshold = (float)mmMinPos * 0.001f;
	}

	int mmMaxPos = (int)SendDlgItemMessage(fusionDebugHandle, IDC_SLIDER_DEPTH_MAX, TBM_GETPOS, 0, 0);

	if (mmMaxPos >= MIN_DEPTH_DISTANCE_MM && mmMaxPos <= MAX_DEPTH_DISTANCE_MM)
	{
		m_params.m_fMaxDepthThreshold = (float)mmMaxPos * 0.001f;
	}

	int maxWeight = (int)SendDlgItemMessage(fusionDebugHandle, IDC_INTEGRATION_WEIGHT_SLIDER, TBM_GETPOS, 0, 0);
	m_params.m_cMaxIntegrationWeight = maxWeight % (MAX_INTEGRATION_WEIGHT + 1);


	// update text
	WCHAR str[MAX_PATH];
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_MIN_DIST_TEXT, str);
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_MAX_DIST_TEXT, str);

	swprintf_s(str, ARRAYSIZE(str), L"%d", m_params.m_cMaxIntegrationWeight);
	SetDlgItemText(fusionDebugHandle, IDC_INTEGRATION_WEIGHT_TEXT, str);

	m_processor.SetParams(m_params);
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void CKinectFusionExplorer::SetStatusMessage(const WCHAR * szMessage)
{
	size_t length = 0;
	if (FAILED(StringCchLength(
		szMessage,
		KinectFusionProcessorFrame::StatusMessageMaxLen,
		&length)))
	{
		length = 0;
	}

	if (length > 0)
	{
		SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
		m_tickLastStatus = GetTickCount();
	}
	else
	{
		// Clear the status message after a timeout (as long as frames are flowing)
		if (GetTickCount() - m_tickLastStatus > cStatusTimeoutInMilliseconds &&
			m_fFramesPerSecond > 0)
		{
			SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, 0);
			m_tickLastStatus = GetTickCount();
		}
	}
}

/// <summary>
/// Set the frames-per-second message
/// </summary>
/// <param name="fFramesPerSecond">current frame rate</param>
void CKinectFusionExplorer::SetFramesPerSecond(float fFramesPerSecond)
{
	if (fFramesPerSecond != m_fFramesPerSecond)
	{
		m_fFramesPerSecond = fFramesPerSecond;
		WCHAR str[MAX_PATH] = { 0 };
		if (fFramesPerSecond > 0)
		{
			swprintf_s(str, ARRAYSIZE(str), L"%5.2f FPS", fFramesPerSecond);
		}

		SendDlgItemMessageW(m_hWnd, IDC_FRAMES_PER_SECOND, WM_SETTEXT, 0, (LPARAM)str);
	}
}

void CKinectFusionExplorer::HandleKeyInput()
{
	if (KFKeys::GetKeyStateOnce(VK_F10))
	{
		//cDebug::DbgOut(L"Keydown indeed");
		if (IsWindowVisible(fusionDebugHandle))
		{
			ShowWindow(hButtonInteractionMode, SW_SHOW);
			ShowWindow(hButtonResetReconstruction, SW_SHOW);
			ShowWindow(hButtonTestOne, SW_SHOW);
			//ShowWindow(hButtonTestTwo, SW_SHOW);
			ShowWindow(fusionDebugHandle, SW_HIDE);
		}
		else
		{
			RECT rRect;
			GetClientRect(m_hWnd, &rRect);
			ShowWindow(fusionDebugHandle, SW_SHOW);
			ShowWindow(hButtonInteractionMode, SW_HIDE);
			ShowWindow(hButtonResetReconstruction, SW_HIDE);
			ShowWindow(hButtonTestOne, SW_HIDE);
			//ShowWindow(hButtonTestTwo, SW_HIDE);
			MoveWindow(fusionDebugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
			InitializeUIControls();
		}

	}
}

void CKinectFusionExplorer::MoveUIOnResize()
{
	RECT rRect;
	GetClientRect(GetParent(m_hWnd), &rRect);

	int recoHeight = (rRect.right - 400)*ratio;
	int recoWidth = rRect.right - 400;
	if (recoHeight >= rRect.bottom - 30)
	{
		recoHeight = rRect.bottom - 30;
		recoWidth = recoHeight / ratio;
	}
	MoveWindow(GetDlgItem(m_hWnd, IDC_RECONSTRUCTION_VIEW), (rRect.right - (recoWidth- (55/ratio)))/2, 55, recoWidth- (55/ratio), recoHeight-55, true);
	MoveWindow(GetDlgItem(m_hWnd, IDC_BUTTON_INTERACTION_MODE), rRect.right - 200, rRect.bottom/2 + 50, 150, 150, true);
	MoveWindow(GetDlgItem(m_hWnd, IDC_BUTTON_TEST_INTERACTION), rRect.right - 350, 10, 300, 50, true);
	MoveWindow(GetDlgItem(m_hWnd, IDC_BUTTON_TEST_OPENGL), rRect.right - 350, 400, 300, 50, true);
	MoveWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RESET_RECONSTRUCTION), rRect.right - 200, rRect.bottom/2 - 200, 150, 150, true);
	
	if (GetWindowState() == START)
	{
		//MoveWindow(hButtonYes, width / 2 - 175, height - 150, 150, 50, true);
		//MoveWindow(hButtonNo, width / 2 + 25, height - 150, 150, 50, true);
		RECT rect;
		GetClientRect(hStatusText, &rect);
		MoveWindow(hStatusText, recoWidth / 2 - 250, recoHeight / 2 - 60, 500, 40, true);
		SetDlgItemText(m_hWnd, IDC_FUSION_STATIC_STATUS, L"Let's start with scanning your scene.");

		InvalidateRect(hStatusText, &rect, TRUE);
		MapWindowPoints(hStatusText, m_hWnd, (POINT *)&rect, 2);
		RedrawWindow(hStatusText, &rect, NULL, RDW_ERASE | RDW_INVALIDATE);

		RECT smallRect;
		GetClientRect(hStatusTextSmall, &smallRect);
		MoveWindow(hStatusTextSmall, recoWidth / 2 - 250, recoHeight / 2 - 20, 500, 30, true);
		SetDlgItemText(m_hWnd, IDC_FUSION_STATIC_STATUS_SMALL, L"Please move the sensor slowly and smoothly.");

		InvalidateRect(hStatusTextSmall, &smallRect, TRUE);
		MapWindowPoints(hStatusTextSmall, m_hWnd, (POINT *)&smallRect, 2);
		RedrawWindow(hStatusTextSmall, &smallRect, NULL, RDW_ERASE | RDW_INVALIDATE);

		MoveWindow(hButtonStart, recoWidth / 2 - 100, recoHeight / 2 + 20, 200, 50, true);


		RECT cRect;
		GetClientRect(hCountdownText, &cRect);
		MoveWindow(hCountdownText, recoWidth / 2 - 250, recoHeight / 2 - 150, 500, 300, true);
		//SetDlgItemText(m_hWnd, IDC_FUSION_STATIC_COUNTDOWN, L"3");

		InvalidateRect(hCountdownText, &cRect, TRUE);
		MapWindowPoints(hCountdownText, m_hWnd, (POINT *)&cRect, 2);
		RedrawWindow(hCountdownText, &cRect, NULL, RDW_ERASE | RDW_INVALIDATE);



		RECT helpRect;
		GetClientRect(hHelpText, &helpRect);
		MoveWindow(hHelpText, recoWidth / 2 - 350, recoHeight / 2 + 200, 700, 40, true);
		SetDlgItemText(m_hWnd, IDC_FUSION_STATIC_HELP, L"Press 'Reset' to start over and 'Finished' if you're satisfied with your scan.");

		InvalidateRect(hHelpText, &helpRect, TRUE);
		MapWindowPoints(hHelpText, m_hWnd, (POINT *)&helpRect, 2);
		RedrawWindow(hHelpText, &helpRect, NULL, RDW_ERASE | RDW_INVALIDATE);
		
	}

	MoveWindow(GetDlgItem(m_hWnd, IDC_FRAMES_PER_SECOND), 0, rRect.bottom - 30, 100, 30, true);
	MoveWindow(GetDlgItem(m_hWnd, IDC_STATUS), 100, rRect.bottom - 30, rRect.right - 100, 30, true);
	MoveWindow(fusionDebugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
}

void CKinectFusionExplorer::InitializeFusionUI()
{

}

void CKinectFusionExplorer::HideAllUIElements()
{

	std::vector<HWND>::iterator iter;
	for (iter = fusionUiElements.begin(); iter != fusionUiElements.end(); ++iter)
	{
		ShowWindow ((*iter), SW_HIDE);
	}
}

FusionState CKinectFusionExplorer::GetWindowState()
{
	return state;
}

void CKinectFusionExplorer::SetWindowState(FusionState fState)
{
	state = fState;

	HideAllUIElements();
	MoveUIOnResize();
	if (state == START)
	{		
		
		ShowWindow(hButtonInteractionMode, SW_SHOW);
		ShowWindow(hButtonResetReconstruction, SW_SHOW);
		ShowWindow(hButtonTestOne, SW_SHOW);
		//ShowWindow(hButtonTestTwo, SW_SHOW);
		ShowWindow(hButtonStart, SW_SHOW);
		ShowWindow(hStatusText, SW_SHOW);
		ShowWindow(hStatusTextSmall, SW_SHOW);
		ShowWindow(hHelpText, SW_SHOW);

		EnableWindow(hButtonInteractionMode, false);
		EnableWindow(hButtonResetReconstruction, false);
		EnableWindow(hButtonTestOne, false);
		//EnableWindow(hButtonTestTwo, false);
	}
	else if (state == COUNTDOWN)
	{
		ShowWindow(hCountdownText, SW_SHOW);
		ShowWindow(hButtonInteractionMode, SW_SHOW);
		ShowWindow(hButtonResetReconstruction, SW_SHOW);
		ShowWindow(hButtonTestOne, SW_SHOW);
		//ShowWindow(hButtonTestTwo, SW_SHOW);
		ShowWindow(hHelpText, SW_SHOW);
		EnableWindow(hButtonInteractionMode, false);
		EnableWindow(hButtonResetReconstruction, false);
		EnableWindow(hButtonTestOne, false);
		//EnableWindow(hButtonTestTwo, false);
	}
	else if (state == SCAN)
	{
		EnableWindow(hButtonInteractionMode, true);
		EnableWindow(hButtonResetReconstruction, true);
		EnableWindow(hButtonTestOne, true);
		//EnableWindow(hButtonTestTwo, true);
		ShowWindow(GetDlgItem(m_hWnd, IDC_RECONSTRUCTION_VIEW), SW_SHOW);
		ShowWindow(GetDlgItem(m_hWnd, IDC_BUTTON_INTERACTION_MODE), SW_SHOW);
		ShowWindow(GetDlgItem(m_hWnd, IDC_BUTTON_TEST_INTERACTION), SW_SHOW);
		//ShowWindow(GetDlgItem(m_hWnd, IDC_BUTTON_TEST_OPENGL), SW_SHOW);
		ShowWindow(GetDlgItem(m_hWnd, IDC_BUTTON_RESET_RECONSTRUCTION), SW_SHOW);
		DlgProc(m_hWnd, WM_INITDIALOG, 0, 0);
	}
	
}

void ResumeKinectFusion()
{
	ShowWindow(hWndApp, SW_SHOW);
	interactionMode = false;
	resumed = true;
}