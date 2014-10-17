#include "common.h"
#include "openGLWin.h"
#include "resource.h"
#include "stdafx.h"

#pragma region

WNDPROC oldEditProc;

HWND editKSearchHandle, editMinClustersHandle, editMaxClustersHandle, editNonHandle, editSmoothnessHandle, editCurvatureHandle, editFillHoleHandle;
HWND statusHandle;

TCHAR Keys::kp[256] = { 0 };

const char className[] = "OpenGLWindow";

bool winDestroyed = false;

#pragma endregion variables

#pragma region

LPCWSTR OpenGLWin::GetLastErrorStdStr()
{
	DWORD error = GetLastError();
	if (error)
	{
		LPVOID lpMsgBuf;
		DWORD bufLen = FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER |
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			error,
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR)&lpMsgBuf,
			0, NULL);
		if (bufLen)
		{
			LPCWSTR lpMsgStr = (LPCWSTR)lpMsgBuf;
			//std::string result(lpMsgStr, lpMsgStr + bufLen);

			//LocalFree(lpMsgBuf);

			return lpMsgStr;
		}
	}
	return NULL;
}

HINSTANCE OpenGLWin::GetInstance()
{
	return appInstance;
}

#pragma endregion Getter and Setter

#pragma region

int Keys::GetKeyState(int key)
{
	return (GetAsyncKeyState(key) >> 15) & 1;
}

int Keys::GetKeyStateOnce(int key)
{
	if (GetKeyState(key) && !kp[key])
	{ 
		kp[key] = 1; 
		return 1;
	}
	else if (!GetKeyState(key))
	{
		kp[key] = 0;
	}
	return 0;
}

#pragma endregion class 'Keys'

#pragma region

void OpenGLWin::ResetTimer()
{
	tLastFrame = clock();
	fFrameInterval = 0.0f;
}

void OpenGLWin::UpdateTimer()
{
	clock_t tCur = clock();
	fFrameInterval = float(tCur - tLastFrame) / float(CLOCKS_PER_SEC);
	tLastFrame = tCur;
}

float OpenGLWin::SpeedOptimizedFloat(float fVal)
{
	return fVal*fFrameInterval;
}

#pragma endregion FPS stuff

#pragma region

bool OpenGLWin::CreateOpenGLWindow()
{
	HWND hWndApp = CreateDialogParamW(
		appInstance,
		MAKEINTRESOURCE(IDD_OPENGL),
		nullptr,
		(DLGPROC)GLDlgProc,
		reinterpret_cast<LPARAM>(this));

	// Show window
	//ShowWindow(parent, SW_HIDE);
	ShowWindow(hWndApp, SW_SHOW);
	WNDCLASSEX wc = { 0 };
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = GLViewportProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = appInstance;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = (LPCWSTR)className;
	wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	ATOM ClassAtom = RegisterClassExW(&wc);

	//RECT r = { 0, 0, 1024, 768 };
	//GetClientRect(parent, &r);

	HWND hWnd = CreateWindowExW(WS_EX_APPWINDOW, (LPCTSTR)MAKELONG(ClassAtom, 0), L"", WS_CHILD | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_VISIBLE,
		0, 0, width, height, hWndApp,
		NULL, appInstance, NULL);

	if (hWnd == NULL)
	{
		MessageBox(0, GetLastErrorStdStr(),
			L"ERROR!", MB_OK);
		return false;
	}
	glWindowHandle = hWnd;
	return true;
}

void OpenGLWin::ShutdownWindow()
{
	//ShowWindow(parent, SW_RESTORE);
	glControl.ReleaseOpenGLControl(&glControl);
	DestroyWindow(glWindowHandle);
	UnregisterClass((LPCWSTR)className, appInstance);
}

#pragma endregion Initialize / Shutdown Window

#pragma region

bool OpenGLWin::StartOpenGLThread(HWND parentWin, HINSTANCE currHInstance, KinectFusionProcessor* proc)
{
	parent = parentWin;
	appInstance = currHInstance;
	processor = proc;
	interactionThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&GLViewportThreadMain, 0, 0, &threadId);
	if (interactionThread == NULL)
		return false;
	return true;

}

DWORD OpenGLWin::GetThreadID()
{
	return threadId;
}

LRESULT CALLBACK GLViewportProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	//openGLWin.SetWheelDelta(0);
	PAINTSTRUCT ps;
	switch (msg)
	{
	case WM_PAINT:
		BeginPaint(hWnd, &ps);
		EndPaint(hWnd, &ps);
		break;

	case WM_CLOSE:
		PostQuitMessage(0);
		break;

	case WM_ACTIVATE:
	{
						switch (LOWORD(wParam))
						{
						case WA_ACTIVE:
						case WA_CLICKACTIVE:
							//appMain.bAppActive = true;
							openGLWin.ResetTimer();
							break;
						case WA_INACTIVE:
							//appMain.bAppActive = false;
							break;

						}
						break;
	}
	case WM_MOUSEWHEEL:
		openGLWin.wheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		break;
	case WM_SIZE:
		openGLWin.glControl.ResizeOpenGLViewportFull(openGLWin.width, openGLWin.height);
		openGLWin.glControl.SetProjection3D(45.0f, float(LOWORD(lParam)) / float(HIWORD(lParam)), 0.1f, 1000.0f);
		openGLWin.glControl.SetOrtho2D(LOWORD(lParam), HIWORD(lParam));
		break;

	default:
		return DefWindowProc(hWnd, msg, wParam, lParam);
	}
	return 0;
}

int WINAPI GLViewportThreadMain()
{
	if (!openGLWin.CreateOpenGLWindow())
	{
		MessageBox(0, L"Can't create OpenGL Window",
			L"ERROR!", MB_OK);
		return -1;
	}
	/*HWND glWindowHandle = CreateWindowExW(WS_EX_NOPARENTNOTIFY, (LPCTSTR)MAKELONG(ClassAtom, 0), L"", WS_CHILD,
	0, 0, 640, 480, parent,
	NULL, hInstance, NULL);*/

	if (!openGLWin.glControl.InitOpenGL(openGLWin.GetInstance(), &openGLWin.glWindowHandle, 3, 3, Initialize, Render, Release, openGLWin.processor, &openGLWin.glControl))
	{
		MessageBox(0, L"Error with initOpenGL",
			L"ERROR!", MB_OK);
	}
	openGLWin.glControl.ResizeOpenGLViewportFull(openGLWin.width, openGLWin.height);
	//appMain.bAppActive = true;
	ShowWindow(openGLWin.glWindowHandle, SW_SHOW);
	UpdateWindow(openGLWin.glWindowHandle);
	MSG msg;
	bool done = false;
	while (!done)
	{
		while (PeekMessage(&msg, 0, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			if (msg.message == WM_QUIT || msg.message == WM_DESTROY || winDestroyed)
				done = true;
		}

		openGLWin.UpdateTimer();

		//OutputDebugStringW(L"Timer updated\n");
		openGLWin.glControl.Render(&openGLWin.glControl);
		//OutputDebugStringW(L"OpenGL rendered\n");
		Sleep(10);

	}
	openGLWin.ShutdownWindow();
	winDestroyed = false;
	return 0;
}

#pragma endregion OpenGL Viewport Thread

#pragma region

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK GLDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_INITDIALOG:
		if (openGLWin.glWindowParent == NULL)
			openGLWin.glWindowParent = hWnd;
		InitializeGLUIControls();
		break;
	case WM_CLOSE:
		winDestroyed = true;
		DestroyWindow(hWnd);
		break;
	case WM_DESTROY:
		break;

		// Handle button press
	case WM_COMMAND:
		GLProcessUI(wParam, lParam);
		break;

		// Handle sliders
	case  WM_HSCROLL:
		UpdateGLHSliders();
		break;
	case WM_MOUSEWHEEL:
		openGLWin.wheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		break;
	case WM_SIZE:
		openGLWin.glControl.ResizeOpenGLViewportFull(openGLWin.width, openGLWin.height);
		openGLWin.glControl.SetProjection3D(45.0f, float(LOWORD(lParam)) / float(HIWORD(lParam)), 0.1f, 1000.0f);
		openGLWin.glControl.SetOrtho2D(LOWORD(lParam), HIWORD(lParam));
		break;
	case WM_NOTIFY:
		break;
	}

	return FALSE;
}
#pragma endregion Window Thread

#pragma region

void InitializeGLUIControls()
{
	editKSearchHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_RG_KSEARCHVALUE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editKSearchHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editMinClustersHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_RG_MINCLUSTERSIZE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editMinClustersHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editMaxClustersHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_RG_MAXCLUSTERSIZE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editMaxClustersHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editNonHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_RG_NON);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editNonHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editSmoothnessHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_RG_SMOOTHNESS);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editSmoothnessHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editCurvatureHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_RG_CURVATURE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editCurvatureHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editFillHoleHandle = GetDlgItem(openGLWin.glWindowParent, IDC_EDIT_FILLHOLES);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editFillHoleHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);

	statusHandle = GetDlgItem(openGLWin.glWindowParent, IDC_IM_STATUS);
	HFONT hFont = CreateFont(22, 10, 0, 0, 700, 0, 0, 0, 0, 0, 0, 0, 0, TEXT("Courier New"));
	SendMessage(statusHandle, WM_SETFONT, (WPARAM)hFont, 0);
	ResetEditControls();

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_KSEARCH_VALUE,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_RG_KSEARCH_VALUE, MAX_RG_KSEARCH_VALUE));



	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_MINCLUSTERSIZE,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_RG_MINCLUSTER, MAX_RG_MINCLUSTER));




	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_MAXCLUSTERSIZE,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_RG_MAXCLUSTER, MAX_RG_MAXCLUSTER));



	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_NON,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_RG_NEIGHBORS, MAX_RG_NEIGHBORS));



	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_SMOOTHNESS,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_RG_SMOOTHNESS, MAX_RG_SMOOTHNESS));



	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_CURVATURE,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_RG_CURVATURE, MAX_RG_CURVATURE));

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_FILLHOLES,
		TBM_SETRANGE,
		TRUE,
		MAKELPARAM(MIN_FILLHOLES, MAX_FILLHOLES));

	ResetSliders();
	//UpdateSliderText();
}

void GLProcessUI(WPARAM wParam, LPARAM lParam)
{
	if (IDC_BUTTON_FILLHOLES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Filling holes..");
		//RemoveSmallComponents();
		FillHoles();
	}
	if (IDC_BUTTON_RG_RESETVALUES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.segmentValuesChanged = true;
		openGLWin.kSearchValue = 50;
		openGLWin.minClusterSize = 100;
		openGLWin.maxClusterSize = 1000;
		openGLWin.numberOfNeighbors = 30;
		openGLWin.smoothnessThreshold = 100;
		openGLWin.curvatureThreshold = 10;
		ResetEditControls();
		ResetSliders();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Region Growth values back to default.");

	}
	if (IDC_CHECK_WIREFRAME == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		ToggleWireFrame();
	}
	if (IDC_CHECK_FREECAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		openGLWin.freeCameraControls = !openGLWin.freeCameraControls;
	}
	if (IDC_CHECK_COLORSELECTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		ToggleColorSelectedObject();
	}
	if (IDC_CHECK_SHOWBB == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		ToggleBoundingBoxes();
	}
	if (IDC_CHECK_RG_ESTIMATENORMALS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.segmentValuesChanged = true;
		openGLWin.estimateNormals = !openGLWin.estimateNormals;
	}
	if (IDC_CHECK_HELPINGVISUALS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.helpingVisuals = !openGLWin.helpingVisuals;
	}
	if (IDC_BUTTON_RG_EXTERNALPREVIEW == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		ShowPCLViewer();
	}
	if (IDC_BUTTON_WALL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		cDebug::DbgOut(L"select wall: ");
		SelectWallObject();
	}
	if (IDC_BUTTON_RESETWALL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		ResetWallObject();
	}
	if (IDC_BUTTON_SETBACKGROUND == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//GetDlgItemInt
		int redValue = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_BACKGROUND_RED, NULL, FALSE);
		int greenValue = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_BACKGROUND_GREEN, NULL, FALSE);
		int blueValue = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_BACKGROUND_BLUE, NULL, FALSE);
		if (redValue >= 0 && redValue <= 255
			&& greenValue >= 0 && greenValue <= 255
			&& blueValue >= 0 && blueValue <= 255)
		{
			SetBackgroundColor(redValue, greenValue, blueValue);
			SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Changed background color.");
		}
		//int redValue = (int)itemBuff;

		//m_processor.ResetReconstruction();
	}
	if (IDC_BUTTON_REGION_GROWTH_SEGMENTATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		cDebug::DbgOut(L"Region growth", 0);
		openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
		openGLWin.previewMode = false;
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Starting region growth segmentation..");
		StartSegmentation();
		//m_processor.ResetReconstruction();
	}


	if (IDC_BUTTON_RG_PREVIEW == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Preparing region growth segmentation preview...");
		openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
		openGLWin.previewMode = true;
		StartSegmentation();
		//m_processor.ResetReconstruction();
	}
	if (IDC_BUTTON_RESETCAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		ResetCameraPosition();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Camera Position reset.");
	}
	if (IDC_BUTTON_EXPORT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Combing and exporting all meshes..");
		CombineAndExport();
	}
}

void ResetEditControls()
{
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_RG_KSEARCHVALUE,
		openGLWin.kSearchValue,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_FILLHOLES,
		openGLWin.holeSize * 100,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_RG_MINCLUSTERSIZE,
		openGLWin.minClusterSize,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_RG_MAXCLUSTERSIZE,
		openGLWin.maxClusterSize * 1000,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_RG_NON,
		openGLWin.numberOfNeighbors,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_RG_SMOOTHNESS,
		(UINT)openGLWin.smoothnessThreshold,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_RG_CURVATURE,
		(UINT)openGLWin.curvatureThreshold,
		FALSE);

	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_BACKGROUND_RED,
		211,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_BACKGROUND_GREEN,
		211,
		FALSE);
	SetDlgItemInt(openGLWin.glWindowParent,
		IDC_EDIT_BACKGROUND_BLUE,
		211,
		FALSE);
}

void ResetSliders()
{
	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_KSEARCH_VALUE,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.kSearchValue);

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_MINCLUSTERSIZE,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.minClusterSize);

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_MAXCLUSTERSIZE,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.maxClusterSize);

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_NON,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.numberOfNeighbors);

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_SMOOTHNESS,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.smoothnessThreshold);

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_RG_CURVATURE,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.curvatureThreshold);

	SendDlgItemMessage(
		openGLWin.glWindowParent,
		IDC_SLIDER_FILLHOLES,
		TBM_SETPOS,
		TRUE,
		(UINT)openGLWin.holeSize);
}

void UpdateSliderText()
{
	wstringstream strs;
	strs << openGLWin.kSearchValue;
	wstring concLabel;
	concLabel.append(strs.str());
	//WCHAR str[MAX_PATH];
	//swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", openGLWin.kSearchValue);
	SetDlgItemText(openGLWin.glWindowParent, IDC_TEXT_RG_KSEARCH_VALUE, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.minClusterSize;
	concLabel.append(strs.str());
	//WCHAR str[MAX_PATH];
	//swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", openGLWin.kSearchValue);
	SetDlgItemText(openGLWin.glWindowParent, IDC_TEXT_RG_MINCLUSTERSIZE, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.maxClusterSize;
	concLabel.append(strs.str());
	//WCHAR str[MAX_PATH];
	//swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", openGLWin.kSearchValue);
	SetDlgItemText(openGLWin.glWindowParent, IDC_TEXT_RG_MAXCLUSTERSIZE, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.numberOfNeighbors;
	concLabel.append(strs.str());
	//WCHAR str[MAX_PATH];
	//swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", openGLWin.kSearchValue);
	SetDlgItemText(openGLWin.glWindowParent, IDC_TEXT_RG_NON, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.smoothnessThreshold;
	concLabel.append(strs.str());
	//WCHAR str[MAX_PATH];
	//swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", openGLWin.kSearchValue);
	SetDlgItemText(openGLWin.glWindowParent, IDC_TEXT_RG_SMOOTHNESS, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.curvatureThreshold;
	concLabel.append(strs.str());
	//WCHAR str[MAX_PATH];
	//swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", openGLWin.kSearchValue);
	SetDlgItemText(openGLWin.glWindowParent, IDC_TEXT_RG_CURVATURE, concLabel.c_str());
}

void UpdateGLHSliders()
{
	openGLWin.segmentValuesChanged = true;
	int kSearchValuePos = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_RG_KSEARCH_VALUE, TBM_GETPOS, 0, 0);

	if (kSearchValuePos >= MIN_RG_KSEARCH_VALUE && kSearchValuePos <= MAX_RG_KSEARCH_VALUE)
	{
		openGLWin.kSearchValue = kSearchValuePos;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_RG_KSEARCHVALUE,
			openGLWin.kSearchValue,
			FALSE);
	}

	int minClusterPos = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_GETPOS, 0, 0);

	if (minClusterPos >= MIN_RG_MINCLUSTER && minClusterPos <= MAX_RG_MINCLUSTER)
	{
		openGLWin.minClusterSize = minClusterPos;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_RG_MINCLUSTERSIZE,
			openGLWin.minClusterSize,
			FALSE);
	}

	int maxClusterPos = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_GETPOS, 0, 0);

	if (maxClusterPos >= MIN_RG_MAXCLUSTER && maxClusterPos <= MAX_RG_MAXCLUSTER)
	{
		openGLWin.maxClusterSize = maxClusterPos;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_RG_MAXCLUSTERSIZE,
			openGLWin.maxClusterSize * 1000,
			FALSE);
	}

	int nonPos = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_RG_NON, TBM_GETPOS, 0, 0);

	if (nonPos >= MIN_RG_NEIGHBORS && nonPos <= MAX_RG_NEIGHBORS)
	{
		openGLWin.numberOfNeighbors = nonPos;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_RG_NON,
			openGLWin.numberOfNeighbors,
			FALSE);
	}

	int smoothnessPos = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_RG_SMOOTHNESS, TBM_GETPOS, 0, 0);

	if (smoothnessPos >= MIN_RG_SMOOTHNESS && smoothnessPos <= MAX_RG_SMOOTHNESS)
	{
		openGLWin.smoothnessThreshold = smoothnessPos;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_RG_SMOOTHNESS,
			(UINT)openGLWin.smoothnessThreshold,
			FALSE);
	}

	int curvaturePos = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_RG_CURVATURE, TBM_GETPOS, 0, 0);

	if (curvaturePos >= MIN_RG_CURVATURE && curvaturePos <= MAX_RG_CURVATURE)
	{
		openGLWin.curvatureThreshold = curvaturePos;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_RG_CURVATURE,
			(UINT)openGLWin.curvatureThreshold,
			FALSE);
	}

	int holeSize = (int)SendDlgItemMessage(openGLWin.glWindowParent, IDC_SLIDER_FILLHOLES, TBM_GETPOS, 0, 0);

	if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
	{
		openGLWin.holeSize = holeSize;
		SetDlgItemInt(openGLWin.glWindowParent,
			IDC_EDIT_FILLHOLES,
			openGLWin.holeSize * 100,
			FALSE);
	}

	//UpdateSliderText();
}

LRESULT CALLBACK SubEditProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch (msg)
	{
	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_RETURN:
			openGLWin.segmentValuesChanged = true;
			if (wnd == editKSearchHandle)
			{
				int kSearchValue = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_RG_KSEARCHVALUE, NULL, FALSE);
				if (kSearchValue >= MIN_RG_KSEARCH_VALUE && kSearchValue <= MAX_RG_KSEARCH_VALUE)
				{
					openGLWin.kSearchValue = kSearchValue;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_RG_KSEARCH_VALUE,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.kSearchValue);
				}
			}
			else if (wnd == editMinClustersHandle)
			{
				int minClusterSize = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_RG_MINCLUSTERSIZE, NULL, FALSE);
				if (minClusterSize >= MIN_RG_MINCLUSTER && minClusterSize <= MAX_RG_MINCLUSTER)
				{
					openGLWin.minClusterSize = minClusterSize;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_RG_MINCLUSTERSIZE,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.minClusterSize);
				}
			}
			else if (wnd == editMaxClustersHandle)
			{
				int maxClusterSize = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_RG_MAXCLUSTERSIZE, NULL, FALSE) / 1000;
				if (maxClusterSize >= MIN_RG_MAXCLUSTER && maxClusterSize <= MAX_RG_MAXCLUSTER)
				{
					openGLWin.maxClusterSize = maxClusterSize;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_RG_MAXCLUSTERSIZE,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.maxClusterSize);
				}
			}
			else if (wnd == editNonHandle)
			{
				int numberOfNeighbors = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_RG_NON, NULL, FALSE);
				if (numberOfNeighbors >= MIN_RG_NEIGHBORS && numberOfNeighbors <= MAX_RG_NEIGHBORS)
				{
					openGLWin.numberOfNeighbors = numberOfNeighbors;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_RG_NON,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.numberOfNeighbors);
				}
			}
			else if (wnd == editSmoothnessHandle)
			{
				int smoothnessThreshold = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_RG_SMOOTHNESS, NULL, FALSE);
				if (smoothnessThreshold >= MIN_RG_SMOOTHNESS && smoothnessThreshold <= MAX_RG_SMOOTHNESS)
				{
					openGLWin.smoothnessThreshold = smoothnessThreshold;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_RG_SMOOTHNESS,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.smoothnessThreshold);
				}
			}
			else if (wnd == editCurvatureHandle)
			{
				int curvatureThreshold = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_RG_CURVATURE, NULL, FALSE);
				if (curvatureThreshold >= MIN_RG_CURVATURE && curvatureThreshold <= MAX_RG_CURVATURE)
				{
					openGLWin.curvatureThreshold = curvatureThreshold;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_RG_CURVATURE,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.curvatureThreshold);
				}
			}
			else if (wnd == editFillHoleHandle)
			{
				int holeSize = GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_FILLHOLES, NULL, FALSE) / 100;
				if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
				{
					openGLWin.holeSize = holeSize;
					SendDlgItemMessage(
						openGLWin.glWindowParent,
						IDC_SLIDER_FILLHOLES,
						TBM_SETPOS,
						TRUE,
						(UINT)openGLWin.holeSize);
				}
			}
			//GetDlgItemInt(openGLWin.glWindowParent, IDC_EDIT_BACKGROUND_RED, NULL, FALSE)
			//Do your stuff
			break;  //or return 0; if you don't want to pass it further to def proc
			//If not your key, skip to default:
		}
	default:
		return CallWindowProc(oldEditProc, wnd, msg, wParam, lParam);
	}
	return 0;
}

bool OpenGLWin::IsMouseInOpenGLWindow()
{
	POINT pCur;
	GetCursorPos(&pCur);
	RECT rRect; GetWindowRect(openGLWin.glWindowHandle, &rRect);

	if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
		pCur.y >= rRect.top && pCur.y <= rRect.bottom &&
		GetActiveWindow() == openGLWin.glWindowParent)
		return true;
	else
		return false;
}

#pragma endregion GUI