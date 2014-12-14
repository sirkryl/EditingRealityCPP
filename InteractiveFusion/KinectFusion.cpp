//------------------------------------------------------------------------------
// <copyright file="KinectFusionExplorer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
#include "KinectFusion.h"
//own namespace includes
#include "Keys.h"
#include "InteractiveFusion.h"

// Fusion Library includes
#include "stdafx.h"
#include "FusionResources.h"
#include "KinectFusionProcessorFrame.h"
#include "KinectFusionHelper.h"

//3rd party includes
#include <boost/thread/thread.hpp>

//OpenGLWin openGLWin;
bool initDone = false;
bool windowsInitialized = false;
float fusionRatio = 480.0f / 640.0f;

bool mouseDown = false;

HWND hWndApp;

//PREPARE UI
std::vector<HWND> prepareUi;
HWND hButtonStart;
HWND hStartTextBig, hStartTextSmall, hHelpText;
HWND hStartMeshQuality;

std::vector<HWND> countdownUi;
HWND hCountdownText;

//SLIDER IN PREPARE UI
HWND hButtonSlider, hSliderBackground, hSliderText, hSliderDescription;
int sliderPos = -1;
POINT sliderAnchor;

//DEBUG TESTS
HWND hButtonTestOne, hButtonTestTwo;

//SCAN UI
std::vector<HWND> scanUi;
HWND hButtonScanDone, hButtonScanReset;
HWND hReconstructionView, hDepthView, hResidualsView;

//DEBUG
HWND fusionDebugHandle;

//STATUS HANDLE
HWND fusionStatusHandle;

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
void StartKinectFusion(HWND parent, HINSTANCE hInstance, KinectFusion*& expl, HWND &fusionHandle)
{
	KinectFusion application;
	expl = &application;
	application.Run(parent, hInstance, 0, fusionHandle);// nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
KinectFusion::KinectFusion() :
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
KinectFusion::~KinectFusion()
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
int KinectFusion::Run(HWND parent, HINSTANCE hInstance, int nCmdShow, HWND &fusionHandle)
{
	MSG       msg = { 0 };
	WNDCLASS  wc = { 0 };


	// Dialog custom window class
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	wc.hbrBackground = openGLWin.hBackground;
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_SCAN));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"KinectFusionExplorerAppDlgWndClass";

	if (!RegisterClassW(&wc))
	{
		return 0;
	}

	hWndApp = CreateDialogParamW(
		hInstance,
		MAKEINTRESOURCE(IDD_SCAN),
		parent,
		(DLGPROC)KinectFusion::MessageRouter,
		reinterpret_cast<LPARAM>(this));

	fusionHandle = hWndApp;
	fusionDebugHandle = CreateDialog(hInstance, MAKEINTRESOURCE(IDD_SCAN_ADVANCED_OPTIONS), hWndApp, (DLGPROC)FusionDebugRouter);
	ShowWindow(fusionDebugHandle, SW_HIDE);
	// Show window
	
	fusionStatusHandle = GetDlgItem(hWndApp, IDC_SCAN_STATUS);

	SendMessage(fusionStatusHandle, WM_SETFONT, (WPARAM)openGLWin.statusFont, 0);

	
	hButtonScanDone = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWndApp, (HMENU)IDC_SCAN_BUTTON_DONE, hInstance, 0);
	hButtonTestOne = CreateWindowEx(0, L"Button", L"Load Test Interaction", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 250, 150, 50, hWndApp, (HMENU)IDC_DEBUG_BUTTON_TEST_INTERACTION, hInstance, 0);
	hButtonTestTwo = CreateWindowEx(0, L"Button", L"Test Open GL", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_DEBUG_BUTTON_TEST_OPENGL, hInstance, 0);
	hButtonScanReset = CreateWindowEx(0, L"Button", L"RESET", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_SCAN_BUTTON_RESET, hInstance, 0);
	ShowWindow(parent, SW_SHOWMAXIMIZED);
	ShowWindow(hWndApp, SW_SHOW);
	//HANDLE thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ThreadMain, (LPVOID)hWndApp, 0, 0);

	hReconstructionView = GetDlgItem(m_hWnd, IDC_SCAN_RECONSTRUCTION_VIEW);
	hDepthView = GetDlgItem(m_hWnd, IDC_SCAN_DEPTH_VIEW);
	hResidualsView = GetDlgItem(m_hWnd, IDC_SCAN_TRACKING_RESIDUALS_VIEW);

	scanUi.push_back(hButtonScanDone);
	scanUi.push_back(hButtonScanReset);
	scanUi.push_back(hReconstructionView);
	scanUi.push_back(hDepthView);
	scanUi.push_back(hResidualsView);


	hButtonStart = CreateWindowEx(0, L"Button", L"START", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_PREPARE_BUTTON_START, hInstance, 0);
	hButtonSlider = CreateWindowEx(0, L"Button", L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_PREPARE_SLIDER_BUTTON, hInstance, 0);
	hSliderBackground = CreateWindowEx(0, L"Button", L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_PREPARE_SLIDER_BACKGROUND, hInstance, 0);
	hSliderText = CreateWindowEx(0, L"STATIC", L"0m", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTER | SS_CENTERIMAGE, 50, 50, 100, 50, hWndApp, (HMENU)IDC_PREPARE_SLIDER_TEXT, hInstance, 0);

	SendMessage(hSliderText, WM_SETFONT, (WPARAM)openGLWin.smallUiFont, TRUE);

	hSliderDescription = CreateWindowEx(0, L"STATIC", L"Size", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS, 50, 50, 100, 50, hWndApp, (HMENU)IDC_PREPARE_SLIDER_LABEL, hInstance, 0);
	SendMessage(hSliderDescription, WM_SETFONT, (WPARAM)openGLWin.bigUiFont, TRUE);

	hStartTextBig = CreateWindowEx(0, L"STATIC", L"Let's start with scanning your scene.", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWndApp, (HMENU)IDC_PREPARE_STATUS_BIG, openGLWin.appInstance, 0);
	
	SendMessage(hStartTextBig, WM_SETFONT, (WPARAM)openGLWin.bigUiFont, TRUE);

	hStartTextSmall = CreateWindowEx(0, L"STATIC", L"Please move the sensor slowly and smoothly.", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWndApp, (HMENU)IDC_PREPARE_STATUS_SMALL, openGLWin.appInstance, 0);

	SendMessage(hStartTextSmall, WM_SETFONT, (WPARAM)openGLWin.smallUiFont, TRUE);
	//ShowWindow(hStatusText, SW_HIDE);

	hCountdownText = CreateWindowEx(0, L"STATIC", L"3", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWndApp, (HMENU)IDC_PREPARE_COUNTDOWN, openGLWin.appInstance, 0);

	SendMessage(hCountdownText, WM_SETFONT, (WPARAM)openGLWin.countDownFont, TRUE);

	hHelpText = CreateWindowEx(0, L"STATIC", L"Press 'RESET' to start over and 'DONE' if you're satisfied with your scan.", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWndApp, (HMENU)IDC_SDEBUG_TEXT_HELP, openGLWin.appInstance, 0);

	SendMessage(hHelpText, WM_SETFONT, (WPARAM)openGLWin.mediumUiFont, TRUE);

	hStartMeshQuality = CreateWindowEx(0, L"STATIC", L"Mesh Quality: LOW", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWndApp, (HMENU)IDC_PREPARE_TEXT_MESHQUALITY, openGLWin.appInstance, 0);

	SendMessage(hStartMeshQuality, WM_SETFONT, (WPARAM)openGLWin.smallUiFont, TRUE);


	EnableWindow(hButtonSlider, false);
	EnableWindow(hSliderBackground, false);


	prepareUi.push_back(hButtonStart);
	prepareUi.push_back(hButtonSlider);
	prepareUi.push_back(hSliderBackground);
	prepareUi.push_back(hSliderText);
	prepareUi.push_back(hSliderDescription);
	prepareUi.push_back(hStartTextBig);
	prepareUi.push_back(hStartTextSmall);
	prepareUi.push_back(hHelpText);
	prepareUi.push_back(hButtonScanDone);
	prepareUi.push_back(hButtonScanReset);
	prepareUi.push_back(hStartMeshQuality);

	countdownUi.push_back(hCountdownText);
	countdownUi.push_back(hHelpText);
	countdownUi.push_back(hButtonScanDone);
	countdownUi.push_back(hButtonScanReset);


	ShowWindow(hButtonTestOne, SW_HIDE);
	ShowWindow(hButtonTestTwo, SW_HIDE);
	//fusionUiElements.push_back(hButtonTestOne);
	//fusionUiElements.push_back(hButtonTestTwo);
	
	SetWindowState(IF_FUSION_STATE_START);
	//ShowWindow(GetDlgItem(m_hWnd, IDC_STATUS), SW_HIDE);
	ShowWindow(GetDlgItem(m_hWnd, IDC_SCAN_FRAMES_PER_SECOND), SW_HIDE);

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
	KinectFusion* pThis = reinterpret_cast<KinectFusion*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA));
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
LRESULT CALLBACK KinectFusion::MessageRouter(
	HWND hWnd,
	UINT uMsg,
	WPARAM wParam,
	LPARAM lParam)
{
	KinectFusion* pThis = nullptr;

	if (WM_INITDIALOG == uMsg && !initDone)
	{
		pThis = reinterpret_cast<KinectFusion*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
		initDone = true;
	}
	else
	{
		pThis = reinterpret_cast<KinectFusion*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{

		if (pThis->GetWindowState() == IF_FUSION_STATE_SCAN)
			return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
		else
			return pThis->StartProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

bool KinectFusion::DrawButton(LPARAM lParam)
{
	LPDRAWITEMSTRUCT Item;
	Item = (LPDRAWITEMSTRUCT)lParam;
	SelectObject(Item->hDC, openGLWin.bigUiFont);
	FillRect(Item->hDC, &Item->rcItem, openGLWin.hBackground);
	SelectObject(Item->hDC, openGLWin.buttonDefaultBrush);
	if (!IsWindowEnabled(Item->hwndItem))
	{
		SetTextColor(Item->hDC, RGB(50, 50, 50));
		if (Item->hwndItem == hButtonScanDone)
		{
			SelectObject(Item->hDC, openGLWin.buttonGreenInactiveBrush);
			SelectObject(Item->hDC, openGLWin.buttonDefaultPen);
		}
		else if (Item->hwndItem == hButtonScanReset)
		{
			SelectObject(Item->hDC, openGLWin.buttonRedInactiveBrush);
			SelectObject(Item->hDC, openGLWin.buttonDefaultPen);
		}
		else
		{
			SelectObject(Item->hDC, openGLWin.buttonInactivePen);
		}
	}
	else if (Item->itemState & ODS_SELECTED)
	{
		SetTextColor(Item->hDC, RGB(245, 245, 245));
		if (Item->hwndItem == hButtonScanDone)
		{
			SelectObject(Item->hDC, openGLWin.buttonGreenPressedBrush);
			SelectObject(Item->hDC, openGLWin.buttonDefaultPen);
		}
		else if (Item->hwndItem == hButtonScanReset)
		{
			SelectObject(Item->hDC, openGLWin.buttonRedPressedBrush);
			SelectObject(Item->hDC, openGLWin.buttonDefaultPen);
		}
		else
		{
			SelectObject(Item->hDC, openGLWin.buttonPressedBrush);
			SelectObject(Item->hDC, openGLWin.buttonPressedPen);
		}
	}
	else
	{
		if (Item->hwndItem == hButtonScanDone)
		{
			SelectObject(Item->hDC, openGLWin.buttonGreenBrush);
		}
		else if (Item->hwndItem == hButtonScanReset)
		{
			SelectObject(Item->hDC, openGLWin.buttonRedBrush);
		}
		SetTextColor(Item->hDC, RGB(240, 240, 240));
		SelectObject(Item->hDC, openGLWin.buttonDefaultPen);
	}
	SetBkMode(Item->hDC, TRANSPARENT);
	if (Item->hwndItem == hButtonScanDone || Item->hwndItem == hButtonScanReset)
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

bool KinectFusion::DrawButtonSlider(LPARAM lParam)
{
	LPDRAWITEMSTRUCT Item;
	Item = (LPDRAWITEMSTRUCT)lParam;
	SelectObject(Item->hDC, openGLWin.smallUiFont);
	FillRect(Item->hDC, &Item->rcItem, openGLWin.hBackground);
	SelectObject(Item->hDC, openGLWin.buttonPressedBrush);
	SetTextColor(Item->hDC, RGB(240, 240, 240));
	SelectObject(Item->hDC, openGLWin.buttonDefaultPen);
	SetBkMode(Item->hDC, TRANSPARENT);

	RoundRect(Item->hDC, Item->rcItem.left, Item->rcItem.top, Item->rcItem.right, Item->rcItem.bottom, 20, 20);
	int len;
	len = GetWindowTextLength(Item->hwndItem);
	LPSTR lpBuff;
	lpBuff = new char[len + 1];
	GetWindowTextA(Item->hwndItem, lpBuff, len + 1);

	DrawTextA(Item->hDC, lpBuff, len, &Item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

	if (sliderPos == -1)
		UpdateButtonSliderValue();

	return TRUE;
}


bool KinectFusion::DrawSliderBackground(LPARAM lParam)
{
	LPDRAWITEMSTRUCT Item;
	Item = (LPDRAWITEMSTRUCT)lParam;
	SelectObject(Item->hDC, openGLWin.bigUiFont);
	FillRect(Item->hDC, &Item->rcItem, openGLWin.hBackground);
	SelectObject(Item->hDC, openGLWin.buttonDefaultBrush);
	SetTextColor(Item->hDC, RGB(240, 240, 240));
	SelectObject(Item->hDC, openGLWin.buttonDefaultPen);

	SetBkMode(Item->hDC, TRANSPARENT);

	RoundRect(Item->hDC, Item->rcItem.left, Item->rcItem.top, Item->rcItem.right, Item->rcItem.bottom, 20, 20);
	int len;
	len = GetWindowTextLength(Item->hwndItem);
	LPSTR lpBuff;
	lpBuff = new char[len + 1];
	GetWindowTextA(Item->hwndItem, lpBuff, len + 1);

	DrawTextA(Item->hDC, lpBuff, len, &Item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

	return TRUE;
}
void KinectFusion::UpdateButtonSlider()
{
	POINT p;
	GetCursorPos(&p);
	RECT borderRect; GetClientRect(hSliderBackground, &borderRect);
	RECT sliderRect; GetClientRect(hButtonSlider, &sliderRect);
	MapWindowPoints(hSliderBackground, m_hWnd, (POINT *)&borderRect, 2);

	ScreenToClient(m_hWnd, &p);
	if (p.x != sliderAnchor.x && p.x > borderRect.left + (sliderRect.right / 2) && p.x < borderRect.right - (sliderRect.right / 2))
	{
		sliderAnchor.x = p.x;
		RECT rect; GetClientRect(m_hWnd, &rect);

		sliderPos = p.x - (int)(sliderRect.right / 2.0f) - borderRect.left;
		MoveButtonSlider(sliderPos);
		//MoveWindow(hButtonSlider, p.x-50, rect.bottom / 2 + 300, 100, 100, true);
		//RECT sliderRect; GetWindowRect(hButtonSlider, &sliderRect);
		//32 <--> 256
		float channelWidth = borderRect.right - (sliderRect.right / 2.0f) - borderRect.left - (sliderRect.right / 2.0f);
		float percent = sliderPos / channelWidth;
		int value = (int)(percent * 224.0f) - 1;
		m_params.m_reconstructionParams.voxelsPerMeter = (float)(256 - value);

		float volumeWidth = m_params.m_reconstructionParams.voxelCountX / m_params.m_reconstructionParams.voxelsPerMeter;
		float volumeHeight = m_params.m_reconstructionParams.voxelCountY / m_params.m_reconstructionParams.voxelsPerMeter;
		float volumeDepth = m_params.m_reconstructionParams.voxelCountZ / m_params.m_reconstructionParams.voxelsPerMeter;

		wchar_t buffer[256];
		std::wstring s;
		int slen = swprintf(buffer, 255, L"%4.1fm x %4.1fm x %4.1fm", volumeWidth, volumeHeight, volumeDepth);
		s.assign(buffer, slen);

		SetDlgItemText(hWndApp, IDC_PREPARE_SLIDER_TEXT, s.c_str());

		if (m_params.m_reconstructionParams.voxelsPerMeter > 192)
		{
			openGLWin.meshQuality = IF_QUALITY_VERYHIGH;
			SetDlgItemText(hWndApp, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: Very High");
		}
		else if (m_params.m_reconstructionParams.voxelsPerMeter > 150)
		{
			openGLWin.meshQuality = IF_QUALITY_HIGH;
			SetDlgItemText(hWndApp, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: High");
		}
		else if (m_params.m_reconstructionParams.voxelsPerMeter > 110)
		{
			openGLWin.meshQuality = IF_QUALITY_MEDIUM;
			SetDlgItemText(hWndApp, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: Medium");
		}
		else if (m_params.m_reconstructionParams.voxelsPerMeter > 64)
		{
			openGLWin.meshQuality = IF_QUALITY_LOW;
			SetDlgItemText(hWndApp, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: Low");
		}
		else
		{
			openGLWin.meshQuality = IF_QUALITY_VERYLOW;
			SetDlgItemText(hWndApp, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: Very Low");
		}

	}
}

void KinectFusion::UpdateButtonSliderValue()
{



	RECT borderRect; GetWindowRect(hSliderBackground, &borderRect);
	RECT sliderRect; GetClientRect(hButtonSlider, &sliderRect);

	int value = 256 - (int)m_params.m_reconstructionParams.voxelsPerMeter;
	float percent = (value - 1) / 224.0f;
	float channelWidth = borderRect.right - (sliderRect.right / 2.0f) - borderRect.left - (sliderRect.right / 2.0f);
	sliderPos = (int)(percent * channelWidth);
	//MoveButtonSlider(sliderPos);

	float volumeWidth = m_params.m_reconstructionParams.voxelCountX / m_params.m_reconstructionParams.voxelsPerMeter;
	float volumeHeight = m_params.m_reconstructionParams.voxelCountY / m_params.m_reconstructionParams.voxelsPerMeter;
	float volumeDepth = m_params.m_reconstructionParams.voxelCountZ / m_params.m_reconstructionParams.voxelsPerMeter;

	wchar_t buffer[256];
	std::wstring s;
	int slen = swprintf(buffer, 255, L"%4.1fm x %4.1fm x %4.1fm", volumeWidth, volumeHeight, volumeDepth);
	s.assign(buffer, slen);

	SetDlgItemText(hWndApp, IDC_PREPARE_SLIDER_TEXT, s.c_str());
	MoveButtonSlider(sliderPos);
}

LRESULT CALLBACK KinectFusion::StartProc(
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
		ProcessUI(wParam, lParam);
		break;

		// Handle sliders
	case  WM_HSCROLL:
		UpdateHSliders();
		break;
	case WM_CTLCOLORDLG:
		return (LRESULT)openGLWin.hBackground;
		break;
	case WM_CTLCOLORSTATIC:
	{
		HDC hdc = reinterpret_cast<HDC>(wParam);
		SetBkMode((HDC)wParam, TRANSPARENT);
		SetTextColor(hdc, RGB(255, 255, 255));

		return (LRESULT)openGLWin.hBackground;
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
		if (IDC_SCAN_BUTTON_DONE == LOWORD(wParam)
			|| IDC_DEBUG_BUTTON_TEST_INTERACTION == LOWORD(wParam)
			|| IDC_DEBUG_BUTTON_TEST_OPENGL == LOWORD(wParam)
			|| IDC_SCAN_BUTTON_RESET == LOWORD(wParam)
			|| IDC_PREPARE_BUTTON_START == LOWORD(wParam))
		{
			return DrawButton(lParam);
		}
		else if (IDC_PREPARE_SLIDER_BACKGROUND == LOWORD(wParam))
			return DrawSliderBackground(lParam);
		else if (IDC_PREPARE_SLIDER_BUTTON == LOWORD(wParam))
			return DrawButtonSlider(lParam);

	}
	break;
	case WM_LBUTTONDOWN:
		mouseDown = true;
		break;
	case WM_LBUTTONUP:
		mouseDown = false;
		break;
	case WM_MOUSEMOVE:
		if (mouseDown)
		{
			if (IsMouseInHandle(hButtonSlider))
				UpdateButtonSlider();
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
LRESULT CALLBACK KinectFusion::DlgProc(
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
			hDepthView,
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
			hReconstructionView,
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
			hResidualsView,
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
		return (LRESULT)openGLWin.hBackground;
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
	{
		HDC hdc = reinterpret_cast<HDC>(wParam);
		SetBkMode((HDC)wParam, TRANSPARENT);
		SetTextColor(hdc, RGB(255, 255, 255));

		return (LRESULT)openGLWin.hBackground;
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

		if (IDC_SCAN_BUTTON_DONE == LOWORD(wParam)
			|| IDC_DEBUG_BUTTON_TEST_INTERACTION == LOWORD(wParam)
			|| IDC_DEBUG_BUTTON_TEST_OPENGL == LOWORD(wParam)
			|| IDC_SCAN_BUTTON_RESET == LOWORD(wParam)
			|| IDC_PREPARE_BUTTON_START == LOWORD(wParam))
		{
			return DrawButton(lParam);
		}
		else if (IDC_PREPARE_SLIDER_BUTTON == LOWORD(wParam))
			return DrawButtonSlider(lParam);
		else if (IDC_PREPARE_SLIDER_BACKGROUND == LOWORD(wParam))
			return DrawSliderBackground(lParam);
	}
	break;
	case WM_LBUTTONDOWN:
		mouseDown = true;
		break;
	case WM_LBUTTONUP:
		mouseDown = false;
		break;
	case WM_MOUSEMOVE:
		if (mouseDown)
		{
			if (IsMouseInHandle(hButtonSlider))
				UpdateButtonSlider();
		}
		break;
	}

	return FALSE;
}

/// <summary>
/// Handle a completed frame from the Kinect Fusion processor.
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
void KinectFusion::HandleCompletedFrame()
{
	KinectFusionProcessorFrame const* pFrame = nullptr;

	// Flush any extra WM_FRAMEREADY messages from the queue
	MSG msg;
	while (PeekMessage(&msg, m_hWnd, WM_FRAMEREADY, WM_FRAMEREADY, PM_REMOVE)) {}

	m_processor.LockFrame(&pFrame);

	if (!m_bSavingMesh && openGLWin.GetWindowMode() == IF_MODE_SCAN && GetWindowState() == IF_FUSION_STATE_SCAN) // don't render while a mesh is being saved
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
		CheckDlgButton(m_hWnd, IDC_SDEBUG_CHECK_PAUSE_INTEGRATION, BST_UNCHECKED);
		m_processor.SetParams(m_params);
	}
	else if (m_processor.IsCameraPoseFinderAvailable() && !m_params.m_bPauseIntegration)
	{
		m_params.m_bPauseIntegration = true;
		CheckDlgButton(m_hWnd, IDC_SDEBUG_CHECK_PAUSE_INTEGRATION, BST_CHECKED);
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
			HWND hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_X_640);
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Y_640);
			EnableWindow(hButton, FALSE);
			hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Z_640);
			EnableWindow(hButton, FALSE);
			if (Is64BitApp() == FALSE)
			{
				// Also disable 512 voxel resolution in one arbitrary axis on 32bit machines
				hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Y_512);
				EnableWindow(hButton, FALSE);
			}
		}
		else if (pFrame->m_deviceMemory <= 2 * Mebi)  // 2GB
		{
			if (Is64BitApp() == FALSE)
			{
				// Disable 640 voxel resolution in one arbitrary axis on 32bit machines
				HWND hButton = GetDlgItem(m_hWnd, IDC_SDEBUG_CHECK_VOXELS_Y_640);
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
HRESULT KinectFusion::SaveMeshFile(INuiFusionColorMesh* pMesh, KinectFusionMeshTypes saveMeshType)
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
void KinectFusion::InitializeUIControls()
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
		IDC_SDEBUG_SLIDER_DEPTH_MIN,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

	SendDlgItemMessage(fusionDebugHandle,
		IDC_SDEBUG_SLIDER_DEPTH_MAX,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SDEBUG_SLIDER_INTEGRATION_WEIGHT,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_INTEGRATION_WEIGHT, MAX_INTEGRATION_WEIGHT));

	// Set slider positions
	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SDEBUG_SLIDER_DEPTH_MAX,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fMaxDepthThreshold * 1000);

	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SDEBUG_SLIDER_DEPTH_MIN,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_fMinDepthThreshold * 1000);

	SendDlgItemMessage(
		fusionDebugHandle,
		IDC_SDEBUG_SLIDER_INTEGRATION_WEIGHT,
		TBM_SETPOS,
		TRUE,
		(UINT)m_params.m_cMaxIntegrationWeight);

	// Set intermediate slider tics at meter intervals
	for (int i = 1; i<(MAX_DEPTH_DISTANCE_MM / 1000); i++)
	{
		SendDlgItemMessage(fusionDebugHandle, IDC_SDEBUG_SLIDER_DEPTH_MAX, TBM_SETTIC, 0, i * 1000);
		SendDlgItemMessage(fusionDebugHandle, IDC_SDEBUG_SLIDER_DEPTH_MIN, TBM_SETTIC, 0, i * 1000);
	}

	// Update slider text
	WCHAR str[MAX_PATH];
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_SDEBUG_TEXT_MIN_DIST, str);
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_SDEBUG_TEXT_MAX_DIST, str);

	swprintf_s(str, ARRAYSIZE(str), L"%d", m_params.m_cMaxIntegrationWeight);
	SetDlgItemText(fusionDebugHandle, IDC_SDEBUG_TEXT_INTEGRATION_WEIGHT, str);

	// Set the radio buttons for Volume Parameters
	switch ((int)m_params.m_reconstructionParams.voxelsPerMeter)
	{
	case 768:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_768, BST_CHECKED);
		break;
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VPM_256, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountX)
	{
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_X_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_X_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_X_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_X_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_X_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_X_384, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountY)
	{
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_384, BST_CHECKED);
		break;
	}

	switch ((int)m_params.m_reconstructionParams.voxelCountZ)
	{
	case 640:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_640, BST_CHECKED);
		break;
	case 512:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_512, BST_CHECKED);
		break;
	case 384:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_384, BST_CHECKED);
		break;
	case 256:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_256, BST_CHECKED);
		break;
	case 128:
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_128, BST_CHECKED);
		break;
	default:
		m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_384, BST_CHECKED);
		break;
	}

	if (Stl == m_saveMeshFormat)
	{
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_RADIO_MESH_FORMAT_STL, BST_CHECKED);
	}
	else if (Obj == m_saveMeshFormat)
	{
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_RADIO_MESH_FORMAT_OBJ, BST_CHECKED);
	}
	else if (Ply == m_saveMeshFormat)
	{
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_RADIO_MESH_FORMAT_PLY, BST_CHECKED);
	}

	if (m_params.m_bCaptureColor)
	{
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_CAPTURE_COLOR, BST_CHECKED);
	}

	if (m_params.m_bAutoFindCameraPoseWhenLost)
	{
		CheckDlgButton(fusionDebugHandle, IDC_SDEBUG_CHECK_CAMERA_POSE_FINDER, BST_CHECKED);
	}
}


int WINAPI CountdownThread()
{

	int secondTimer = 3;
	KinectFusion* pThis = reinterpret_cast<KinectFusion*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA));
	pThis->SetWindowState(IF_FUSION_STATE_COUNTDOWN);
	//pThis->HideAllUIElements();
	ShowWindow(hCountdownText, SW_SHOW);
	//SetDlgItemText(hWndApp, IDC_FUSION_STATIC_COUNTDOWN, L"3");
	//KinectFusion* explorer = reinterpret_cast<KinectFusion*>(exp);
	while (secondTimer > 0)
	{
		SetDlgItemText(hWndApp, IDC_PREPARE_COUNTDOWN, std::to_wstring(secondTimer).c_str());
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		secondTimer--;
	}
	//OutputDebugString(L"wel")
	pThis->SetWindowState(IF_FUSION_STATE_SCAN);

	return 0;
}

/// <summary>
/// Process the UI inputs
/// </summary>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
void KinectFusion::ProcessUI(WPARAM wParam, LPARAM lParam)
{
	// If it was for the near mode control and a clicked event, change near mode
	if (IDC_SDEBUG_CHECK_NEARMODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		m_params.m_bNearMode = !m_params.m_bNearMode;
	}
	// If it was for the display surface normals toggle this variable
	if (IDC_SDEBUG_CHECK_CAPTURE_COLOR == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle capture color
		m_params.m_bCaptureColor = !m_params.m_bCaptureColor;
		/*if (m_params.m_reconstructionParams.voxelsPerMeter != 64)
		m_params.m_reconstructionParams.voxelsPerMeter = 64;
		else
		m_params.m_reconstructionParams.voxelsPerMeter = 256;*/
	}
	// If it was for the display surface normals toggle this variable
	if (IDC_SDEBUG_CHECK_MIRROR_DEPTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle depth mirroring
		m_params.m_bMirrorDepthFrame = !m_params.m_bMirrorDepthFrame;

		m_processor.ResetReconstruction();
	}
	if (IDC_SDEBUG_CHECK_CAMERA_POSE_FINDER == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_bAutoFindCameraPoseWhenLost = !m_params.m_bAutoFindCameraPoseWhenLost;

		if (!m_params.m_bAutoFindCameraPoseWhenLost)
		{
			// Force pause integration off when unchecking use of camera pose finder
			m_params.m_bPauseIntegration = false;
		}
	}
	// If it was the reset button clicked, clear the volume
	if (IDC_SCAN_BUTTON_RESET == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_processor.ResetReconstruction();
	}
	if (IDC_DEBUG_BUTTON_TEST_INTERACTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		FinishScan(1);
		/*interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(reinterpret_cast<KinectFusion*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA)), 1);*/
		//ptrStartOpenGL(&m_processor, 1);

		//openGLWin.testMode = 1;
		//openGLWin.StartOpenGLThread(m_hWnd, GetModuleHandle(NULL));

	}
	if (IDC_DEBUG_BUTTON_TEST_OPENGL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		FinishScan(2);
	//	interactionMode = true;
		m_params.m_bPauseIntegration = true;
		/*interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(reinterpret_cast<KinectFusion*>(::GetWindowLongPtr(hWndApp, GWLP_USERDATA)), 2);*/
		//openGLWin.testMode = 2;
		//openGLWin.StartOpenGLThread(m_hWnd, GetModuleHandle(NULL));
	}
	// If it was the mesh button clicked, mesh the volume and save
	if (IDC_SDEBUG_BUTTON_MESH_RECONSTRUCTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
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
	if (IDC_PREPARE_BUTTON_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		StartScan();

		//SetWindowState(SCAN);
	}
	if (IDC_SCAN_BUTTON_DONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		FinishScan(0);
	}
	if (IDC_SDEBUG_CHECK_PAUSE_INTEGRATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle the pause state of the reconstruction
		m_params.m_bPauseIntegration = !m_params.m_bPauseIntegration;
	}
	if (IDC_SDEBUG_CHECK_VPM_768 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 768.0f;
	}
	if (IDC_SDEBUG_CHECK_VPM_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 640.0f;
	}
	if (IDC_SDEBUG_CHECK_VPM_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 512.0f;
	}
	if (IDC_SDEBUG_CHECK_VPM_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 384.0f;
	}
	if (IDC_SDEBUG_CHECK_VPM_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;
	}
	if (IDC_SDEBUG_CHECK_VPM_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelsPerMeter = 128.0f;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_X_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 640;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_X_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 512;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_X_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 384;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_X_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 256;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_X_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountX = 128;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Y_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 640;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Y_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 512;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Y_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 384;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Y_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 256;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Y_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountY = 128;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Z_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 640;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Z_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 512;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Z_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 384;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Z_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 256;
	}
	if (IDC_SDEBUG_CHECK_VOXELS_Z_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_params.m_reconstructionParams.voxelCountZ = 128;
	}
	if (IDC_SDEBUG_RADIO_MESH_FORMAT_STL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Stl;
	}
	if (IDC_SDEBUG_RADIO_MESH_FORMAT_OBJ == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Obj;
	}
	if (IDC_SDEBUG_RADIO_MESH_FORMAT_PLY == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		m_saveMeshFormat = Ply;
	}

	m_processor.SetParams(m_params);
}

void KinectFusion::StartScan()
{
	openGLWin.SetWindowMode(IF_MODE_SCAN);
	m_params.m_bPauseIntegration = false;
	//interactionMode = false;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&CountdownThread, 0, 0, 0);
}

void KinectFusion::FinishScan(int testMode)
{
	if (testMode == 0)
	{
		//SetFocus(hWndApp);
		SetForegroundWindow(hWndApp);
		//openGLWin.testMode = 0;
		//SetStatusMessage(L"Creating mesh and entering interaction mode. Please wait...");
		m_bSavingMesh = true;
		//interactionMode = true;
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
			//interactionMode = true;
			ShowWindow(hWndApp, SW_HIDE);
			StartOpenGLThread(0);
			hr = S_OK;
			// Release the mesh
			SafeRelease(mesh);
		}
		else
		{
			SetStatusMessage(L"Failed to enter interaction mode.");
		}

		// Restore pause state of integration
		m_processor.SetParams(m_params);


		m_bSavingMesh = false;
	}
	else if (testMode == 1)
	{
		//interactionMode = true;
		m_params.m_bPauseIntegration = true;
		ShowWindow(hWndApp, SW_HIDE);
		StartOpenGLThread(1);
	}
	else if (testMode == 2)
	{
		//interactionMode = true;
		m_params.m_bPauseIntegration = true;
		ShowWindow(hWndApp, SW_HIDE);
		StartOpenGLThread(2);
	}

}



/// <summary>
/// Update the internal variable values from the UI Horizontal sliders.
/// </summary>
void KinectFusion::UpdateHSliders()
{
	int mmMinPos = (int)SendDlgItemMessage(fusionDebugHandle, IDC_SDEBUG_SLIDER_DEPTH_MIN, TBM_GETPOS, 0, 0);

	if (mmMinPos >= MIN_DEPTH_DISTANCE_MM && mmMinPos <= MAX_DEPTH_DISTANCE_MM)
	{
		m_params.m_fMinDepthThreshold = (float)mmMinPos * 0.001f;
	}

	int mmMaxPos = (int)SendDlgItemMessage(fusionDebugHandle, IDC_SDEBUG_SLIDER_DEPTH_MAX, TBM_GETPOS, 0, 0);

	if (mmMaxPos >= MIN_DEPTH_DISTANCE_MM && mmMaxPos <= MAX_DEPTH_DISTANCE_MM)
	{
		m_params.m_fMaxDepthThreshold = (float)mmMaxPos * 0.001f;
	}

	int maxWeight = (int)SendDlgItemMessage(fusionDebugHandle, IDC_SDEBUG_SLIDER_INTEGRATION_WEIGHT, TBM_GETPOS, 0, 0);
	m_params.m_cMaxIntegrationWeight = maxWeight % (MAX_INTEGRATION_WEIGHT + 1);


	// update text
	WCHAR str[MAX_PATH];
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_SDEBUG_TEXT_MIN_DIST, str);
	swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
	SetDlgItemText(fusionDebugHandle, IDC_SDEBUG_TEXT_MAX_DIST, str);

	swprintf_s(str, ARRAYSIZE(str), L"%d", m_params.m_cMaxIntegrationWeight);
	SetDlgItemText(fusionDebugHandle, IDC_SDEBUG_TEXT_INTEGRATION_WEIGHT, str);

	m_processor.SetParams(m_params);
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void KinectFusion::SetStatusMessage(const WCHAR * szMessage)
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
		SendDlgItemMessageW(m_hWnd, IDC_SCAN_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
		//ptrShowStatusMsg(szMessage);
		m_tickLastStatus = GetTickCount();
	}
	else
	{
		// Clear the status message after a timeout (as long as frames are flowing)
		if (GetTickCount() - m_tickLastStatus > cStatusTimeoutInMilliseconds &&
			m_fFramesPerSecond > 0)
		{
			//ptrShowStatusMsg(L"");
			SendDlgItemMessageW(m_hWnd, IDC_SCAN_STATUS, WM_SETTEXT, 0, 0);
			m_tickLastStatus = GetTickCount();
		}
	}
}

/// <summary>
/// Set the frames-per-second message
/// </summary>
/// <param name="fFramesPerSecond">current frame rate</param>
void KinectFusion::SetFramesPerSecond(float fFramesPerSecond)
{
	if (fFramesPerSecond != m_fFramesPerSecond)
	{
		m_fFramesPerSecond = fFramesPerSecond;
		WCHAR str[MAX_PATH] = { 0 };
		if (fFramesPerSecond > 0)
		{
			swprintf_s(str, ARRAYSIZE(str), L"%5.2f FPS", fFramesPerSecond);
		}

		SendDlgItemMessageW(m_hWnd, IDC_SCAN_FRAMES_PER_SECOND, WM_SETTEXT, 0, (LPARAM)str);
	}
}

void KinectFusion::HandleKeyInput()
{
	if (Keys::GetKeyStateOnce(VK_F10))
	{
		//cDebug::DbgOut(L"Keydown indeed");
		if (IsWindowVisible(fusionDebugHandle))
		{
			ShowWindow(hButtonScanDone, SW_SHOW);
			ShowWindow(hButtonScanReset, SW_SHOW);
			//ShowWindow(hButtonTestOne, SW_SHOW);
			//ShowWindow(hButtonTestTwo, SW_SHOW);
			ShowWindow(fusionDebugHandle, SW_HIDE);
		}
		else
		{
			RECT rRect;
			GetClientRect(m_hWnd, &rRect);
			ShowWindow(fusionDebugHandle, SW_SHOW);
			ShowWindow(hButtonScanDone, SW_HIDE);
			ShowWindow(hButtonScanReset, SW_HIDE);
			//ShowWindow(hButtonTestOne, SW_HIDE);
			//ShowWindow(hButtonTestTwo, SW_HIDE);
			MoveWindow(fusionDebugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
			InitializeUIControls();
		}

	}
}

void KinectFusion::MoveUIOnResize()
{
	RECT rRect;
	GetClientRect(GetParent(m_hWnd), &rRect);

	int recoHeight = (int)((rRect.right - 400)*fusionRatio);
	int recoWidth = rRect.right - 400;
	if (recoHeight >= rRect.bottom - 30)
	{
		recoHeight = rRect.bottom - 30;
		recoWidth = (int)(recoHeight / fusionRatio);
	}
	MoveWindow(hReconstructionView, (rRect.right - (recoWidth - (int)(55 / fusionRatio))) / 2, 55, recoWidth - (int)(55 / fusionRatio), recoHeight - 55, true);
	MoveWindow(hResidualsView, 30, 55, 200, 200, true);
	MoveWindow(hDepthView, 30, 255, 200, 200, true);
	MoveWindow(hButtonScanDone, rRect.right - 200, rRect.bottom / 2 + 50, 150, 150, true);
	MoveWindow(hButtonTestTwo, rRect.right - 350, 10, 300, 50, true);
	MoveWindow(hButtonTestOne, rRect.right - 350, 400, 300, 50, true);
	MoveWindow(hButtonScanReset, rRect.right - 200, rRect.bottom / 2 - 200, 150, 150, true);

	if (GetWindowState() == IF_FUSION_STATE_START || GetWindowState() == IF_FUSION_STATE_COUNTDOWN)
	{
		MoveWindow(hStartTextBig, rRect.right / 2 - 250, rRect.bottom / 2 - 210, 500, 40, true);

		MoveWindow(hStartTextSmall, rRect.right / 2 - 250, rRect.bottom / 2 - 170, 500, 30, true);

		MoveWindow(hButtonStart, rRect.right / 2 - 100, rRect.bottom - 220, 200, 50, true);

		MoveWindow(hHelpText, rRect.right / 2 - 450, rRect.bottom - 150, 900, 40, true);

		MoveWindow(hCountdownText, rRect.right / 2 - 250, rRect.bottom / 2 - 150, 500, 300, true);

		MoveWindow(hStartMeshQuality, rRect.right / 2 - 70, rRect.bottom/ 2 + 50, 200, 30, true);
	}
	//MoveWindow(GetDlgItem(m_hWnd, IDC_FRAMES_PER_SECOND), 0, rRect.bottom - 30, 100, 30, true);
	MoveWindow(fusionStatusHandle, 0, rRect.bottom - 30, rRect.right, 30, true);
	MoveWindow(fusionDebugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);

	if (sliderPos == -1)
	{
		MoveButtonSlider(0);
	}
	else
	{

		MoveButtonSlider(sliderPos);
	}

}

void KinectFusion::MoveButtonSlider(int pos)
{
	RECT rRect;
	GetClientRect(GetParent(m_hWnd), &rRect);
	//MoveWindow(hButtonSlider, (rRect.right / 2 - 100) + pos, rRect.bottom / 2 + 300, 50, 50, true);
	//MoveWindow(hSliderBackground, rRect.right / 2 - 100, rRect.bottom / 2 + 300, 250, 50, true);
	//MoveWindow(hSliderText, rRect.right / 2 + 310, rRect.bottom / 2 + 325, 300, 25, true);

	MoveWindow(hButtonSlider, rRect.right / 2 - 165 + pos, rRect.bottom / 2 - 30, 50, 50, true);
	MoveWindow(hSliderBackground, rRect.right / 2 - 165, rRect.bottom / 2 - 30, 400, 50, true);
	MoveWindow(hSliderText, rRect.right / 2 - 70, rRect.bottom / 2 + 30, 200, 25, true);
	MoveWindow(hSliderDescription, rRect.right / 2 - 235, rRect.bottom / 2 - 25, 60, 50, true);
}

void KinectFusion::InitializeFusionUI()
{

}

FusionState KinectFusion::GetWindowState()
{
	return state;
}

void KinectFusion::SetWindowState(FusionState fState)
{
	state = fState;

	openGLWin.HideUI(prepareUi);
	openGLWin.HideUI(scanUi);
	openGLWin.HideUI(countdownUi);
	MoveUIOnResize();
	if (state == IF_FUSION_STATE_START)
	{
		openGLWin.ShowUI(prepareUi);
		EnableWindow(hButtonScanDone, false);
		EnableWindow(hButtonScanReset, false);
	}
	else if (state == IF_FUSION_STATE_COUNTDOWN)
	{
		
		openGLWin.ShowUI(countdownUi);

		EnableWindow(hButtonScanDone, false);
		EnableWindow(hButtonScanReset, false);
	}
	else if (state == IF_FUSION_STATE_SCAN)
	{
		openGLWin.ShowUI(scanUi);
		EnableWindow(hButtonScanDone, true);
		EnableWindow(hButtonScanReset, true);

		if (!windowsInitialized)
		{
			DlgProc(m_hWnd, WM_INITDIALOG, 0, 0);
			windowsInitialized = true;
		}

	}

}

int KinectFusion::GetVoxelsPerMeter()
{
	return (int)m_params.m_reconstructionParams.voxelsPerMeter;
}

bool KinectFusion::IsMouseInHandle(HWND handle)
{
	POINT pCur;
	GetCursorPos(&pCur);

	RECT rRect; GetWindowRect(handle, &rRect);
	if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
		pCur.y >= rRect.top && pCur.y <= rRect.bottom)
		return true;

	return false;
}


Matrix4 KinectFusion::GetWorldToCameraTransform()
{
	return m_processor.GetWorldToCameraTransform();
}

void KinectFusion::ResumeScan()
{
	ShowWindow(hWndApp, SW_SHOW);
}

void KinectFusion::Hide()
{
	ShowWindow(hWndApp, SW_HIDE);
}