#include "common.h"
#include "openGLWin.h"
#include "resource.h"
#include "stdafx.h"
#include <KinectFusionExplorer.h>
#pragma region

OpenGLWin openGLWin;

HWND hButtonYes, hButtonNo, hButtonExport;
HWND hCheckBoxDuplicate;
HWND hTextWalls;
std::vector<HWND> uiElements;
WNDPROC oldEditProc;
HBRUSH hBackground = CreateSolidBrush(RGB(0, 0, 0));
HBRUSH buttonDefaultBrush, buttonPressedBrush;
HPEN buttonDefaultPen, buttonPressedPen;

HWND editKSearchHandle, editMinClustersHandle, editCarryDistanceHandle, editMaxClustersHandle, editNonHandle, editSmoothnessHandle, editCurvatureHandle, editFillHoleHandle, editRemoveComponentHandle;
HWND statusHandle;
HWND debugHandle;
int debugWidth = 0;
TCHAR Keys::kp[256] = { 0 };
HFONT uiFont;
HFONT statusFont;
const char className[] = "OpenGLWindow";

bool winDestroyed = false;
bool winResized = false;
#pragma endregion variables


int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
	openGLWin.appInstance = hInstance;
	openGLWin.parent = CreateDialogParamW(
		openGLWin.appInstance,
		MAKEINTRESOURCE(IDD_INTERACTION),
		nullptr,
		(DLGPROC)GLDlgProc,
		0);

	// Show window
	//ShowWindow(parent, SW_HIDE);
	ShowWindow(openGLWin.parent, SW_SHOW);
	openGLWin.mode = openGLWin.Scanning;
	StartKinectFusion(openGLWin.parent, hInstance, StartOpenGLThread, openGLWin.fusionHandle);
}

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

HANDLE hbitmap;

bool OpenGLWin::CreateOpenGLWindow()
{
	/*char *argv[] = { "arg" };
	int argc = 1; // must/should match the number of strings in argv

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(300, 300);
	glutCreateWindow("here");
	glutDisplayFunc(&display);
	glutMainLoop();*/
	
	WNDCLASSEX wc = { 0 };
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = GLViewportProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = openGLWin.appInstance;
	wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = (LPCWSTR)className;
	wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	ATOM ClassAtom = RegisterClassExW(&wc);

	RECT r = { 0, 0, 1024, 768 };
	//GetClientRect(openGLWin, &r);

	HWND hWnd = CreateWindowExW(WS_EX_APPWINDOW, (LPCTSTR)MAKELONG(ClassAtom, 0), L"", WS_CHILD | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_VISIBLE,
		0, 0, r.right, r.bottom, parent,
		NULL, openGLWin.appInstance, NULL);

	if (hWnd == NULL)
	{
		MessageBox(0, openGLWin.GetLastErrorStdStr(),
			L"ERROR!", MB_OK);
		return false;
	}
	openGLWin.glWindowHandle = hWnd;


	uiFont = CreateFont(40, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"FreeSans");
	statusFont = CreateFont(22, 10, 0, 0, 700, 0, 0, 0, 0, 0, 0, 0, 0, TEXT("Courier New"));
	
	hbitmap = LoadImage(GetModuleHandle(NULL), MAKEINTRESOURCE(IDB_BUTTON), IMAGE_BITMAP, 84, 36, LR_DEFAULTCOLOR);
	hButtonYes = CreateWindowEx(0, L"Button", L"Yes", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_YES, openGLWin.appInstance, 0);
	hButtonNo = CreateWindowEx(0, L"Button", L"No", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_NO, openGLWin.appInstance, 0);
	//hTextWalls = CreateWindowEx(0, L"Text", L"Is this (part of) a floor/wall?", WS_CHILD | WS_VISIBLE | BS_BITMAP, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_NO, appInstance, 0);
	hButtonExport = CreateWindowEx(0, L"BUTTON", L"Export", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_EXPORT, NULL, 0);

	buttonDefaultBrush = CreateSolidBrush(RGB(20, 20, 20));
	buttonPressedBrush = CreateSolidBrush(RGB(40, 40, 40));
	buttonDefaultPen = CreatePen(PS_SOLID, 2, RGB(240, 240, 240));
	buttonPressedPen = CreatePen(PS_SOLID, 2, RGB(220, 220, 220));
	/*HFONT hf;
	HDC hdc;
	long lfHeight;

	hdc = GetDC(NULL);
	lfHeight = -MulDiv(20, GetDeviceCaps(hdc, LOGPIXELSY), 72);
	ReleaseDC(NULL, hdc);

	hf = CreateFont(lfHeight, 0, 0, 0, 600, 0, 0, 0, 0, 0, 0, 0, 0, L"Segoe UI");*/
	//SendMessage(hButtonYes, WM_SETFONT, (WPARAM)hf, TRUE);
	//SendMessage(hButtonNo, WM_SETFONT, (WPARAM)hf, TRUE);
	//SendMessage(hButtonExport, WM_SETFONT, (WPARAM)hf, TRUE);


	ShowWindow(hButtonYes, SW_HIDE);
	ShowWindow(hButtonNo, SW_HIDE);
	
	uiElements.push_back(hButtonYes);
	uiElements.push_back(hButtonNo);
	uiElements.push_back(hButtonExport);
	uiElements.push_back(hTextWalls);


	
//debugHandle = CreateWindowEx(WS_EX_OVERLAPPEDWINDOW, MAKEINTRESOURCE(IDD_DEBUG), L"Debug", WS_CHILD | WS_VISIBLE | DS_CONTROL, 50, 50, 300, 300, hWnd, 0, appInstance, 0);
	debugHandle = CreateDialog(openGLWin.appInstance, MAKEINTRESOURCE(IDD_DEBUG), hWnd, (DLGPROC)DebugDlgProc);
	ShowWindow(debugHandle, SW_HIDE);

	hTextWalls = GetDlgItem(openGLWin.glWindowParent, IDC_STATIC_WALL);
	SetDlgItemText(openGLWin.glWindowParent, IDC_STATIC_WALL, L"Is this (part of) a floor/wall?");
	SendMessage(hTextWalls, WM_SETFONT, (WPARAM)uiFont, TRUE);
	ShowWindow(hTextWalls, SW_HIDE);
	ShowWindow(parent, SW_SHOWMAXIMIZED);


	//SendMessage(d, BM_SETIMAGE, IMAGE_BITMAP, (LPARAM)hbitmap);
	
	//hbit = LoadBitmap(hInstance, L"Bit");
	//SendMessage(d, BM_SETIMAGE, (WPARAM)IMAGE_BITMAP, (LPARAM)hbit);


	return true;
}

void OpenGLWin::ReleaseOpenGL()
{
	glControl.ReleaseOpenGLControl(&glControl);
}

void OpenGLWin::ShutdownWindow()
{
	//ShowWindow(parent, SW_RESTORE);
	cDebug::DbgOut(L"CleanUp");

	winDestroyed = true;
	DestroyWindow(debugHandle);
	DestroyWindow(openGLWin.fusionHandle);
	//PostMessage(openGLWin.fusionHandle, WM_CLOSE, 0, 0);
	DestroyWindow(openGLWin.glWindowHandle);
	DestroyWindow(openGLWin.parent);
	DeleteObject(hBackground);
	DeleteObject(buttonDefaultBrush);
	DeleteObject(buttonDefaultPen);
	DeleteObject(buttonPressedBrush);
	DeleteObject(buttonPressedPen);
	DeleteObject(statusFont);
	DeleteObject(uiFont);

	
	//if (openGLWin.mode != openGLWin.Scanning)
	//	openGLWin.ReleaseOpenGL();
	UnregisterClass((LPCWSTR)className, appInstance);
	
	
}

#pragma endregion Initialize / Shutdown Window

#pragma region

//bool OpenGLWin::StartOpenGLThread(HWND parentWin, HINSTANCE currHInstance, KinectFusionProcessor* proc)
void StartOpenGLThread(KinectFusionProcessor* proc, int testMode)
{
	openGLWin.testMode = testMode;
	openGLWin.mode = openGLWin.Interaction;
	//parent = parentWin;
	//appInstance = currHInstance;
	openGLWin.processor = proc;
	openGLWin.interactionThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&GLViewportThreadMain, 0, 0, &openGLWin.threadId);
	if (openGLWin.interactionThread == NULL)
		return;
	return;

}

DWORD OpenGLWin::GetThreadID()
{
	return threadId;
}

int WINAPI LoadMeshThread()
{
	LoadInput();
	return 0;
}

LRESULT CALLBACK GLViewportProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	//openGLWin.SetWheelDelta(0);
	PAINTSTRUCT ps;
	LPDRAWITEMSTRUCT Item;
	switch (msg)
	{
	case WM_PAINT:
		BeginPaint(hWnd, &ps);
		EndPaint(hWnd, &ps);
		break;
	case WM_CLOSE:
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
		GLProcessUI(wParam, lParam);
		break;
	case WM_DRAWITEM:
	{
		
		if (IDC_BUTTON_EXPORT == LOWORD(wParam)
			|| IDC_BUTTON_YES == LOWORD(wParam)
			|| IDC_BUTTON_NO == LOWORD(wParam))
		{
		
		Item = (LPDRAWITEMSTRUCT)lParam;
		SelectObject(Item->hDC, uiFont);
		FillRect(Item->hDC, &Item->rcItem, hBackground);
		SelectObject(Item->hDC, buttonDefaultBrush);
		if (Item->itemState & ODS_SELECTED)
		{
			SetTextColor(Item->hDC, RGB(245, 245, 245));
			SelectObject(Item->hDC, buttonPressedBrush);
			SelectObject(Item->hDC, buttonPressedPen);
		}
		else
		{
			SetTextColor(Item->hDC, RGB(240,240,240));
			SelectObject(Item->hDC, buttonDefaultPen);

		}
		SetBkMode(Item->hDC, TRANSPARENT);
		RoundRect(Item->hDC, Item->rcItem.left, Item->rcItem.top, Item->rcItem.right, Item->rcItem.bottom, 20, 20);
		int len;
		len = GetWindowTextLength(Item->hwndItem);
		LPSTR lpBuff;
		lpBuff = new char[len + 1];
		GetWindowTextA(Item->hwndItem, lpBuff, len + 1);
		
		DrawTextA(Item->hDC, lpBuff, len, &Item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
		DeleteObject(lpBuff);
		return TRUE;
		}
	}
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
		openGLWin.glControl.ResizeOpenGLViewportFull(LOWORD(lParam), HIWORD(lParam));
		openGLWin.glControl.SetProjection3D(45.0f, float(LOWORD(lParam)) / float(HIWORD(lParam)), 0.1f, 1000.0f);
		openGLWin.glControl.SetOrtho2D(LOWORD(lParam), HIWORD(lParam));
		MoveButtonsOnResize();
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

	
	//if (!openGLWin.glControl.InitOpenGL(openGLWin.GetInstance(), &openGLWin.glWindowHandle, 3, 3, Initialize, Render, Release,  //&openGLWin.glControl))
	if (!openGLWin.glControl.InitOpenGL(openGLWin.appInstance, &openGLWin.glWindowHandle, 3, 3, Initialize, Render, Release, openGLWin.processor, &openGLWin.glControl))
	{
		MessageBox(0, L"Error with initOpenGL",
			L"ERROR!", MB_OK);
	}
	DWORD threads;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&LoadMeshThread, 0, 0, &threads);

	RECT rRect; GetClientRect(GetParent(openGLWin.glWindowHandle), &rRect);
	openGLWin.glControl.ResizeOpenGLViewportFull(rRect.right, rRect.bottom);
	MoveButtonsOnResize();
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

		openGLWin.glControl.Render(&openGLWin.glControl);
		//Sleep(10);
	}
	openGLWin.glControl.ReleaseOpenGLControl(&openGLWin.glControl);
	//delete openGLWin.processor;
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
		//InitializeGLUIControls();
		break;
	case WM_CLOSE:
		openGLWin.ShutdownWindow();
		PostQuitMessage(0);
		break;
	case WM_DESTROY:
		break;
	
		// Handle button press
	case WM_COMMAND:
		//GLProcessUI(wParam, lParam);
		break;
	case WM_CTLCOLORSTATIC:
		if ((HWND)lParam == hTextWalls)
		{
			HDC hdc = reinterpret_cast<HDC>(wParam);
			SetBkMode((HDC)wParam, TRANSPARENT);
			SetTextColor(hdc, RGB(255,255,255));
			
			return (LRESULT)hBackground;
		}
		break;
	case  WM_HSCROLL:
		//UpdateGLHSliders();
		break;
	case WM_MOUSEWHEEL:
		openGLWin.wheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		break;
	case WM_SIZE:
		if (openGLWin.mode == openGLWin.Scanning)
			MoveWindow(openGLWin.fusionHandle, 0, 0, LOWORD(lParam), HIWORD(lParam), true);
		else if (openGLWin.mode == openGLWin.Interaction)
		{
			openGLWin.glControl.ResizeOpenGLViewportFull(LOWORD(lParam), HIWORD(lParam));
			openGLWin.glControl.SetProjection3D(45.0f, float(LOWORD(lParam)) / float(HIWORD(lParam)), 0.1f, 1000.0f);
			openGLWin.glControl.SetOrtho2D(LOWORD(lParam), HIWORD(lParam));
			MoveButtonsOnResize();
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
LRESULT CALLBACK DebugDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_INITDIALOG:
		InitializeGLUIControls();
		break;
	case WM_CLOSE:
		DestroyWindow(hWnd);
		break;
	case WM_DESTROY:
		break;
		case WM_COMMAND:
		GLProcessUI(wParam, lParam);
		break;
	case WM_HSCROLL:
		UpdateGLHSliders();
		break;
	case WM_MOUSEWHEEL:
		break;
	case WM_SIZE:
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
	editKSearchHandle = GetDlgItem(debugHandle, IDC_EDIT_RG_KSEARCHVALUE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editKSearchHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editMinClustersHandle = GetDlgItem(debugHandle, IDC_EDIT_RG_MINCLUSTERSIZE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editMinClustersHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editMaxClustersHandle = GetDlgItem(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editMaxClustersHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editNonHandle = GetDlgItem(debugHandle, IDC_EDIT_RG_NON);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editNonHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editSmoothnessHandle = GetDlgItem(debugHandle, IDC_EDIT_RG_SMOOTHNESS);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editSmoothnessHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editCurvatureHandle = GetDlgItem(debugHandle, IDC_EDIT_RG_CURVATURE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editCurvatureHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);
	editFillHoleHandle = GetDlgItem(debugHandle, IDC_EDIT_FILLHOLES);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editFillHoleHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);

	editRemoveComponentHandle = GetDlgItem(debugHandle, IDC_EDIT_REMOVESEGMENTS);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editRemoveComponentHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);

	editCarryDistanceHandle = GetDlgItem(debugHandle, IDC_EDIT_SELECTION_DISTANCE);
	oldEditProc = (WNDPROC)SetWindowLongPtr(editCarryDistanceHandle, GWLP_WNDPROC, (LONG_PTR)SubEditProc);

	//HWND hCheck = GetDlgItem(openGLWin.glWindowParent, IDC_CHECK_PLACING_SNAPTOVERTEX);

	//PostMessage(hCheck, BM_SETCHECK, BST_CHECKED, 0);

	statusHandle = GetDlgItem(openGLWin.glWindowParent, IDC_IM_STATUS);
	
	SendMessage(statusHandle, WM_SETFONT, (WPARAM)statusFont, 0);
	ShowWindow(statusHandle, SW_SHOW);
	ResetEditControls();

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_KSEARCH_VALUE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_KSEARCH_VALUE, MAX_RG_KSEARCH_VALUE));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_REMOVESEGMENTS,TBM_SETRANGE, TRUE,	MAKELPARAM(MIN_REMOVESEGMENTS_VALUE, MAX_REMOVESEGMENTS_VALUE));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE,TBM_SETRANGE, TRUE, MAKELPARAM(MIN_CARRYDISTANCE, MAX_CARRYDISTANCE));

	SendDlgItemMessage(debugHandle,	IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_SETRANGE,	TRUE, MAKELPARAM(MIN_RG_MINCLUSTER, MAX_RG_MINCLUSTER));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_MAXCLUSTER, MAX_RG_MAXCLUSTER));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_NEIGHBORS, MAX_RG_NEIGHBORS));

	SendDlgItemMessage(debugHandle,	IDC_SLIDER_RG_SMOOTHNESS, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_SMOOTHNESS, MAX_RG_SMOOTHNESS));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_CURVATURE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_CURVATURE, MAX_RG_CURVATURE));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_FILLHOLES, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_FILLHOLES, MAX_FILLHOLES));

	ResetSliders();
}

void GLProcessUI(WPARAM wParam, LPARAM lParam)
{
	if (IDC_BUTTON_YES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.wallSelection = false;
		openGLWin.isWall = true;
		openGLWin.ShowConfirmationButtons(false);
		ShowWindow(hTextWalls, SW_HIDE);
		openGLWin.glControl.SetOffSetHeight(0);
		openGLWin.glControl.SetOffSetWidth(0);
		openGLWin.glControl.ResizeOpenGLViewportFull(openGLWin.glControl.GetViewportWidth(), openGLWin.glControl.GetViewportHeight());

	}
	if (IDC_BUTTON_NO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.wallSelection = false;
		openGLWin.isWall = false;
		openGLWin.ShowConfirmationButtons(false);
		ShowWindow(hTextWalls, SW_HIDE);
		openGLWin.glControl.SetOffSetHeight(0);
		openGLWin.glControl.SetOffSetWidth(0);
		openGLWin.glControl.ResizeOpenGLViewportFull(openGLWin.glControl.GetViewportWidth(), openGLWin.glControl.GetViewportHeight());
	}
	if (IDC_BUTTON_EXPORT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Combing and exporting all meshes..");
		CombineAndExport();
	}
	if (IDC_BUTTON_FILLHOLES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Filling holes..");
		//RemoveSmallComponents();
		FillHoles(openGLWin.holeSize);
	}
	if (IDC_BUTTON_RG_RESETVALUES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.segmentValuesChanged = true;
		openGLWin.kSearchValue = 20;
		openGLWin.minClusterSize = 100;
		openGLWin.maxClusterSize = 1000;
		openGLWin.numberOfNeighbors = 20;
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
	if (IDC_CHECK_PLACING_RAYCAST == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		ToggleRaycastPlacing();
	}
	if (IDC_CHECK_PLACING_SNAPTOVERTEX == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		ToggleSnapToVertex();
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
		//ShowPCLViewer();
	}
	if (IDC_BUTTON_WALL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		cDebug::DbgOut(L"select wall: ");
		SelectWallObject();
	}
	if (IDC_BUTTON_MLS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		cDebug::DbgOut(L"select MLS: ");
		//MLS();
	}
	if (IDC_BUTTON_REMOVESEGMENTS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		cDebug::DbgOut(L"select Remove Segments: ");
		int size = GetDlgItemInt(debugHandle, IDC_EDIT_REMOVESEGMENTS, NULL, FALSE);
		RemoveSmallComponents(size);
	}
	if (IDC_BUTTON_RESETWALL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		ResetWallObject();
	}
	if (IDC_BUTTON_SETBACKGROUND == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//GetDlgItemInt
		int redValue = GetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_RED, NULL, FALSE);
		int greenValue = GetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_GREEN, NULL, FALSE);
		int blueValue = GetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_BLUE, NULL, FALSE);
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
		openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
		openGLWin.previewMode = false;
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Starting region growth segmentation..");
		SetFocus(openGLWin.glWindowHandle);
		StartSegmentation();
		//m_processor.ResetReconstruction();
	}

	if (IDC_BUTTON_CLEANMESH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		cDebug::DbgOut(L"clean mesh", 0);
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, L"Cleaning mesh..");
		CleanMesh();
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
}

void ResetEditControls()
{
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_KSEARCHVALUE, openGLWin.kSearchValue, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_REMOVESEGMENTS, openGLWin.maxComponentSize, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_SELECTION_DISTANCE,	openGLWin.carryDistance, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_FILLHOLES, openGLWin.holeSize * 100, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_MINCLUSTERSIZE, openGLWin.minClusterSize, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE, openGLWin.maxClusterSize * 1000,	FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_NON,	openGLWin.numberOfNeighbors, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_SMOOTHNESS, (UINT)openGLWin.smoothnessThreshold,	FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_CURVATURE, (UINT)openGLWin.curvatureThreshold, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_RED, 211, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_GREEN, 211, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_BLUE, 211, FALSE);
}

void ResetSliders()
{
	SendDlgItemMessage(debugHandle,	IDC_SLIDER_RG_KSEARCH_VALUE, TBM_SETPOS, TRUE, (UINT)openGLWin.kSearchValue);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_REMOVESEGMENTS, TBM_SETPOS, TRUE, (UINT)openGLWin.maxComponentSize);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE, TBM_SETPOS, TRUE, (UINT)openGLWin.carryDistance);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)openGLWin.minClusterSize);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)openGLWin.maxClusterSize);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_SETPOS, TRUE, (UINT)openGLWin.numberOfNeighbors);
	SendDlgItemMessage(debugHandle,	IDC_SLIDER_RG_SMOOTHNESS, TBM_SETPOS, TRUE,	(UINT)openGLWin.smoothnessThreshold);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_CURVATURE, TBM_SETPOS, TRUE,	(UINT)openGLWin.curvatureThreshold);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_FILLHOLES, TBM_SETPOS, TRUE, (UINT)openGLWin.holeSize);
}

void UpdateSliderText()
{
	wstringstream strs;
	strs << openGLWin.kSearchValue;
	wstring concLabel;
	concLabel.append(strs.str());
	SetDlgItemText(debugHandle, IDC_TEXT_RG_KSEARCH_VALUE, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.minClusterSize;
	concLabel.append(strs.str());
	SetDlgItemText(debugHandle, IDC_TEXT_RG_MINCLUSTERSIZE, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.maxComponentSize;
	concLabel.append(strs.str());
	SetDlgItemText(debugHandle, IDC_TEXT_RG_MINCLUSTERSIZE, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.numberOfNeighbors;
	concLabel.append(strs.str());
	SetDlgItemText(debugHandle, IDC_TEXT_RG_NON, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.smoothnessThreshold;
	concLabel.append(strs.str());
	SetDlgItemText(debugHandle, IDC_TEXT_RG_SMOOTHNESS, concLabel.c_str());

	strs.str(L"");
	concLabel = L"";
	strs << openGLWin.curvatureThreshold;
	concLabel.append(strs.str());
	SetDlgItemText(debugHandle, IDC_TEXT_RG_CURVATURE, concLabel.c_str());
}

void UpdateGLHSliders()
{
	openGLWin.segmentValuesChanged = true;
	int kSearchValuePos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_KSEARCH_VALUE, TBM_GETPOS, 0, 0);

	if (kSearchValuePos >= MIN_RG_KSEARCH_VALUE && kSearchValuePos <= MAX_RG_KSEARCH_VALUE)
	{
		openGLWin.kSearchValue = kSearchValuePos;
        SetDlgItemInt(debugHandle, IDC_EDIT_RG_KSEARCHVALUE, openGLWin.kSearchValue, FALSE);
	}

	int maxComponetSizePos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_REMOVESEGMENTS, TBM_GETPOS, 0, 0);

	if (maxComponetSizePos >= MIN_REMOVESEGMENTS_VALUE && maxComponetSizePos <= MAX_REMOVESEGMENTS_VALUE)
	{
		openGLWin.maxComponentSize = maxComponetSizePos;
		SetDlgItemInt(debugHandle, IDC_EDIT_REMOVESEGMENTS,	openGLWin.maxComponentSize,	FALSE);
	}

	int carryDistancePos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE, TBM_GETPOS, 0, 0);

	if (carryDistancePos >= MIN_CARRYDISTANCE && carryDistancePos <= MAX_CARRYDISTANCE)
	{
		openGLWin.carryDistance = carryDistancePos;
		SetDlgItemInt(debugHandle, IDC_EDIT_SELECTION_DISTANCE,	openGLWin.carryDistance, FALSE);
	}


	int minClusterPos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_GETPOS, 0, 0);

	if (minClusterPos >= MIN_RG_MINCLUSTER && minClusterPos <= MAX_RG_MINCLUSTER)
	{
		openGLWin.minClusterSize = minClusterPos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_MINCLUSTERSIZE, openGLWin.minClusterSize, FALSE);
	}

	int maxClusterPos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_GETPOS, 0, 0);

	if (maxClusterPos >= MIN_RG_MAXCLUSTER && maxClusterPos <= MAX_RG_MAXCLUSTER)
	{
		openGLWin.maxClusterSize = maxClusterPos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE, openGLWin.maxClusterSize * 1000,	FALSE);
	}

	int nonPos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_GETPOS, 0, 0);

	if (nonPos >= MIN_RG_NEIGHBORS && nonPos <= MAX_RG_NEIGHBORS)
	{
		openGLWin.numberOfNeighbors = nonPos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_NON,	openGLWin.numberOfNeighbors, FALSE);
	}

	int smoothnessPos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_SMOOTHNESS, TBM_GETPOS, 0, 0);

	if (smoothnessPos >= MIN_RG_SMOOTHNESS && smoothnessPos <= MAX_RG_SMOOTHNESS)
	{
		openGLWin.smoothnessThreshold = smoothnessPos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_SMOOTHNESS, (UINT)openGLWin.smoothnessThreshold,	FALSE);
	}

	int curvaturePos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_CURVATURE, TBM_GETPOS, 0, 0);

	if (curvaturePos >= MIN_RG_CURVATURE && curvaturePos <= MAX_RG_CURVATURE)
	{
		openGLWin.curvatureThreshold = curvaturePos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_CURVATURE, (UINT)openGLWin.curvatureThreshold, FALSE);
	}

	int holeSize = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_FILLHOLES, TBM_GETPOS, 0, 0);

	if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
	{
		openGLWin.holeSize = holeSize;
		SetDlgItemInt(debugHandle, IDC_EDIT_FILLHOLES, openGLWin.holeSize * 100, FALSE);
	}
}

void MoveButtonsOnResize()
{
	RECT rRect;
	GetClientRect(debugHandle, &rRect);
	MoveWindow(debugHandle, openGLWin.glControl.GetViewportWidth() - rRect.right, 0, rRect.right, rRect.bottom, true);
	MoveWindow(hButtonYes, openGLWin.glControl.GetViewportWidth() / 2 - 175, openGLWin.glControl.GetViewportHeight() - 150, 150, 50, true);
	MoveWindow(hButtonNo, openGLWin.glControl.GetViewportWidth() / 2 + 25, openGLWin.glControl.GetViewportHeight() - 150, 150, 50, true);
	MoveWindow(hButtonExport, 50, openGLWin.glControl.GetViewportHeight() - 150, 150, 50, true);

	RECT sRect;
	GetWindowRect(statusHandle, &sRect);
	MoveWindow(statusHandle, 0, openGLWin.glControl.GetViewportHeight() - 30, openGLWin.glControl.GetViewportWidth(), 30, true);

	RECT rect;
	GetClientRect(hTextWalls, &rect);
	MoveWindow(hTextWalls, openGLWin.glControl.GetViewportWidth() / 2 - 250, openGLWin.glControl.GetViewportHeight() - 200, 500, 40, true);
	SetDlgItemText(openGLWin.glWindowParent, IDC_STATIC_WALL, L"Is this (part of) a floor/wall?");
	
	InvalidateRect(hTextWalls, &rect, TRUE);
	MapWindowPoints(hTextWalls, openGLWin.glWindowHandle, (POINT *)&rect, 2);
	RedrawWindow(hTextWalls, &rect, NULL, RDW_ERASE | RDW_INVALIDATE);
}

void ToggleDebugControls()
{
	if (IsWindowVisible(debugHandle))
	{
		debugWidth = 0;
		ShowWindow(debugHandle, SW_HIDE);
	}
	else
	{
		ResetEditControls(); 
		ResetSliders();
		RECT dRect;
		GetClientRect(debugHandle, &dRect);
		debugWidth = dRect.right;
		MoveWindow(debugHandle, openGLWin.glControl.GetViewportWidth() - dRect.right, 0, dRect.right, dRect.bottom, false);
		ShowWindow(debugHandle, SW_SHOW);
	}
	RECT rRect;
	GetClientRect(GetParent(openGLWin.glWindowHandle), &rRect);
	openGLWin.glControl.ResizeOpenGLViewportFull(rRect.right, rRect.bottom);
	openGLWin.glControl.SetProjection3D(45.0f, (float)rRect.right / (float)rRect.bottom, 0.1f, 1000.0f);
	openGLWin.glControl.SetOrtho2D(rRect.right, rRect.bottom);
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
				int kSearchValue = GetDlgItemInt(debugHandle, IDC_EDIT_RG_KSEARCHVALUE, NULL, FALSE);
				if (kSearchValue >= MIN_RG_KSEARCH_VALUE && kSearchValue <= MAX_RG_KSEARCH_VALUE)
				{
					openGLWin.kSearchValue = kSearchValue;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_KSEARCH_VALUE,TBM_SETPOS, TRUE, (UINT)openGLWin.kSearchValue);
				}
			}
			else if (wnd == editRemoveComponentHandle)
			{
				int maxComponentSize = GetDlgItemInt(debugHandle, IDC_EDIT_REMOVESEGMENTS, NULL, FALSE);
				if (maxComponentSize >= MIN_REMOVESEGMENTS_VALUE && maxComponentSize <= MAX_REMOVESEGMENTS_VALUE)
				{
					openGLWin.maxComponentSize = maxComponentSize;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_REMOVESEGMENTS, TBM_SETPOS, TRUE, (UINT)openGLWin.maxComponentSize);
				}
			}
			else if (wnd == editCarryDistanceHandle)
			{
				int carryDistance = GetDlgItemInt(debugHandle, IDC_EDIT_SELECTION_DISTANCE, NULL, FALSE);
				if (carryDistance >= MIN_CARRYDISTANCE && carryDistance <= MAX_CARRYDISTANCE)
				{
					openGLWin.carryDistance = carryDistance;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_SELECTION_DISTANCE,TBM_SETPOS,TRUE,(UINT)openGLWin.carryDistance);
				}
			}
			else if (wnd == editMinClustersHandle)
			{
				int minClusterSize = GetDlgItemInt(debugHandle, IDC_EDIT_RG_MINCLUSTERSIZE, NULL, FALSE);
				if (minClusterSize >= MIN_RG_MINCLUSTER && minClusterSize <= MAX_RG_MINCLUSTER)
				{
					openGLWin.minClusterSize = minClusterSize;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_RG_MINCLUSTERSIZE,TBM_SETPOS,TRUE,(UINT)openGLWin.minClusterSize);
				}
			}
			else if (wnd == editMaxClustersHandle)
			{
				int maxClusterSize = GetDlgItemInt(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE, NULL, FALSE) / 1000;
				if (maxClusterSize >= MIN_RG_MAXCLUSTER && maxClusterSize <= MAX_RG_MAXCLUSTER)
				{
					openGLWin.maxClusterSize = maxClusterSize;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_RG_MAXCLUSTERSIZE,TBM_SETPOS,TRUE,(UINT)openGLWin.maxClusterSize);
				}
			}
			else if (wnd == editNonHandle)
			{
				int numberOfNeighbors = GetDlgItemInt(debugHandle, IDC_EDIT_RG_NON, NULL, FALSE);
				if (numberOfNeighbors >= MIN_RG_NEIGHBORS && numberOfNeighbors <= MAX_RG_NEIGHBORS)
				{
					openGLWin.numberOfNeighbors = numberOfNeighbors;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_RG_NON,TBM_SETPOS,TRUE,(UINT)openGLWin.numberOfNeighbors);
				}
			}
			else if (wnd == editSmoothnessHandle)
			{
				int smoothnessThreshold = GetDlgItemInt(debugHandle, IDC_EDIT_RG_SMOOTHNESS, NULL, FALSE);
				if (smoothnessThreshold >= MIN_RG_SMOOTHNESS && smoothnessThreshold <= MAX_RG_SMOOTHNESS)
				{
					openGLWin.smoothnessThreshold = smoothnessThreshold;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_RG_SMOOTHNESS,TBM_SETPOS,TRUE,(UINT)openGLWin.smoothnessThreshold);
				}
			}
			else if (wnd == editCurvatureHandle)
			{
				int curvatureThreshold = GetDlgItemInt(debugHandle, IDC_EDIT_RG_CURVATURE, NULL, FALSE);
				if (curvatureThreshold >= MIN_RG_CURVATURE && curvatureThreshold <= MAX_RG_CURVATURE)
				{
					openGLWin.curvatureThreshold = curvatureThreshold;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_RG_CURVATURE,TBM_SETPOS,TRUE,(UINT)openGLWin.curvatureThreshold);
				}
			}
			else if (wnd == editFillHoleHandle)
			{
				int holeSize = GetDlgItemInt(debugHandle, IDC_EDIT_FILLHOLES, NULL, FALSE) / 100;
				if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
				{
					openGLWin.holeSize = holeSize;
					SendDlgItemMessage(debugHandle,IDC_SLIDER_FILLHOLES,TBM_SETPOS,TRUE,(UINT)openGLWin.holeSize);
				}
			}
			break;
		}
	default:
		return CallWindowProc(oldEditProc, wnd, msg, wParam, lParam);
	}
	return 0;
}

bool OpenGLWin::IsMouseInHandle()
{
	POINT pCur;
	GetCursorPos(&pCur);
	for (int i = 0; i < uiElements.size(); i++)
	{
		if (IsWindowVisible(uiElements[i]))
		{
			RECT rRect; GetWindowRect(uiElements[i], &rRect);
			if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
				pCur.y >= rRect.top && pCur.y <= rRect.bottom)
				return true;
		}
	}
	return false;
}

bool OpenGLWin::IsMouseInOpenGLWindow()
{
	POINT pCur;
	GetCursorPos(&pCur);
	RECT rRect; GetWindowRect(openGLWin.glWindowHandle, &rRect);

	if (pCur.x >= rRect.left && pCur.x <= rRect.right-debugWidth &&
		pCur.y >= rRect.top && pCur.y <= rRect.bottom &&
		GetActiveWindow() == openGLWin.glWindowParent && !IsMouseInHandle())
		return true;
	else
		return false;
}

void OpenGLWin::InitWallConfirmation()
{
	
	ShowWindow(hTextWalls, SW_SHOW);
	openGLWin.wallSelection = true;
	openGLWin.glControl.SetOffSetHeight(100);
	openGLWin.glControl.SetOffSetWidth(100);
	RECT rRect; GetClientRect(GetParent(openGLWin.glWindowHandle), &rRect);
	openGLWin.glControl.ResizeOpenGLViewportFull(rRect.right, rRect.bottom);
	openGLWin.ShowConfirmationButtons(true);
	
}

void OpenGLWin::ShowConfirmationButtons(bool flag)
{
	if (flag)
	{
		MoveButtonsOnResize();
		ShowWindow(hButtonYes, SW_SHOW);
		ShowWindow(hButtonNo, SW_SHOW);
	}
	else
	{
		ShowWindow(hButtonYes, SW_HIDE);
		ShowWindow(hButtonNo, SW_HIDE);
	}
}

#pragma endregion GUI


void CleanUp()
{
	cDebug::DbgOut(L"CleanUp");
	winDestroyed = true;
	DestroyWindow(debugHandle);
	DestroyWindow(openGLWin.fusionHandle);
	DestroyWindow(openGLWin.glWindowHandle);
	DestroyWindow(openGLWin.parent);
	DeleteObject(hBackground);
	DeleteObject(buttonDefaultBrush);
	DeleteObject(buttonDefaultPen);
	DeleteObject(buttonPressedBrush);
	DeleteObject(buttonPressedPen);
	DeleteObject(uiFont);
}