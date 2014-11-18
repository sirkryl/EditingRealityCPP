//------------------------------------------------------------------------------
// <copyright file="KinectFusionExplorer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// System includes
#include "stdafx.h"

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
bool interactionMode = false;
HWND hWndApp;
HWND hButtonTestOne, hButtonTestTwo, hButtonInteractionMode;
HWND hButtonResetReconstruction;
HWND fusionDebugHandle;
HBRUSH bDefaultBrush, bPressedBrush;
HBRUSH bBackground = CreateSolidBrush(RGB(0, 0, 0));
HPEN bDefaultPen, bPressedPen;
HFONT guiFont;
void(*ptrStartOpenGL)(KinectFusionProcessor*,int);
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
void StartKinectFusion(HWND parent, HINSTANCE hInstance, void(*a_ptrStartOpenGL)(KinectFusionProcessor*, int), HWND &fusionHandle)
{
	CKinectFusionExplorer application;
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
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
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
	guiFont = CreateFont(40, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"FreeSans");
	bDefaultBrush = CreateSolidBrush(RGB(20, 20, 20));
	bPressedBrush = CreateSolidBrush(RGB(40, 40, 40));
	bDefaultPen = CreatePen(PS_SOLID, 2, RGB(240, 240, 240));
	bPressedPen = CreatePen(PS_SOLID, 2, RGB(220, 220, 220));
	hButtonInteractionMode = CreateWindowEx(0, L"Button", L"Interaction Mode", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWndApp, (HMENU)IDC_BUTTON_INTERACTION_MODE, hInstance, 0);
	hButtonTestOne = CreateWindowEx(0, L"Button", L"Load Test Interaction", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 250, 150, 50, hWndApp, (HMENU)IDC_BUTTON_TEST_INTERACTION, hInstance, 0);
	hButtonTestTwo = CreateWindowEx(0, L"Button", L"Test Open GL", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_BUTTON_TEST_OPENGL, hInstance, 0);
	hButtonResetReconstruction = CreateWindowEx(0, L"Button", L"Reset", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, hWndApp, (HMENU)IDC_BUTTON_RESET_RECONSTRUCTION, hInstance, 0);
	ShowWindow(parent, SW_SHOWMAXIMIZED);
	ShowWindow(hWndApp, SW_SHOWMAXIMIZED);
	//HANDLE thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&ThreadMain, (LPVOID)hWndApp, 0, 0);
	
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
	if (interactionMode)
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
			return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
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
	if (KFKeys::GetKeyStateOnce(VK_F10))
	{
		//cDebug::DbgOut(L"Keydown indeed");
		if (IsWindowVisible(fusionDebugHandle))
		{
			ShowWindow(hButtonInteractionMode, SW_SHOW);
			ShowWindow(hButtonResetReconstruction, SW_SHOW);
			ShowWindow(hButtonTestOne, SW_SHOW);
			ShowWindow(hButtonTestTwo, SW_SHOW);
			ShowWindow(fusionDebugHandle, SW_HIDE);
		}
		else
		{
			RECT rRect;
			GetClientRect(hWnd, &rRect);
			ShowWindow(fusionDebugHandle, SW_SHOW);
			ShowWindow(hButtonInteractionMode, SW_HIDE);
			ShowWindow(hButtonResetReconstruction, SW_HIDE);
			ShowWindow(hButtonTestOne, SW_HIDE);
			ShowWindow(hButtonTestTwo, SW_HIDE);
			MoveWindow(fusionDebugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
			InitializeUIControls();
		}

	}

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
		DeleteObject(bDefaultPen);
		DeleteObject(bPressedBrush);
		DeleteObject(bPressedPen);
		DeleteObject(guiFont);
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
					OutputDebugString(L"WM_SIZE");
		RECT rRect;
		GetClientRect(GetParent(hWnd), &rRect);
		int recoHeight = (rRect.right - 400)*ratio;
		int recoWidth = rRect.right-400;
		if (recoHeight >= rRect.bottom - 30)
		{
			recoHeight = rRect.bottom - 30;
			recoWidth = recoHeight/ratio;
		}
		MoveWindow(GetDlgItem(hWnd, IDC_RECONSTRUCTION_VIEW), 0, 0, recoWidth, recoHeight, true);
		MoveWindow(GetDlgItem(hWnd, IDC_BUTTON_INTERACTION_MODE), rRect.right - 400, 50, 300, 50, true);
		MoveWindow(GetDlgItem(hWnd, IDC_BUTTON_TEST_INTERACTION), rRect.right - 400, 150, 300, 50, true);
		MoveWindow(GetDlgItem(hWnd, IDC_BUTTON_TEST_OPENGL), rRect.right - 400, 250, 300, 50, true);
		MoveWindow(GetDlgItem(hWnd, IDC_BUTTON_RESET_RECONSTRUCTION), rRect.right - 400, rRect.bottom - 250, 300, 50, true);
		MoveWindow(GetDlgItem(hWnd, IDC_FRAMES_PER_SECOND), 0, rRect.bottom - 30, 100, 30, true);
		MoveWindow(GetDlgItem(hWnd, IDC_STATUS), 100, rRect.bottom - 30, rRect.right - 100, 30, true);
		MoveWindow(fusionDebugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
	}
		break;
	case WM_DRAWITEM:
	{

		if (IDC_BUTTON_INTERACTION_MODE == LOWORD(wParam)
			|| IDC_BUTTON_TEST_INTERACTION == LOWORD(wParam)
			|| IDC_BUTTON_TEST_OPENGL == LOWORD(wParam)
			|| IDC_BUTTON_RESET_RECONSTRUCTION == LOWORD(wParam))
		{
			LPDRAWITEMSTRUCT Item;
			Item = (LPDRAWITEMSTRUCT)lParam;
			SelectObject(Item->hDC, guiFont);
			FillRect(Item->hDC, &Item->rcItem, bBackground);
			SelectObject(Item->hDC, bDefaultBrush);
			if (Item->itemState & ODS_SELECTED)
			{
				SetTextColor(Item->hDC, RGB(245, 245, 245));
				SelectObject(Item->hDC, bPressedBrush);
				SelectObject(Item->hDC, bPressedPen);
			}
			else
			{
				SetTextColor(Item->hDC, RGB(240, 240, 240));
				SelectObject(Item->hDC, bDefaultPen);

			}
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
		interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(&m_processor, 1);

		//openGLWin.testMode = 1;
		//openGLWin.StartOpenGLThread(m_hWnd, GetModuleHandle(NULL));

	}
	if (IDC_BUTTON_TEST_OPENGL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		interactionMode = true;
		ShowWindow(hWndApp, SW_HIDE);
		ptrStartOpenGL(&m_processor, 2);
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
	if (IDC_BUTTON_INTERACTION_MODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//openGLWin.testMode = 0;
		SetStatusMessage(L"Creating mesh and entering interaction mode. Please wait...");
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
			ptrStartOpenGL(&m_processor, 0);
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
