#include "InteractiveFusion.h"
#include "MeshHelper.h"
#include "SelectionHelper.h"
#include "OpenGLCamera.h"
#include "Keys.h"
#include "SegmentationHelper.h"
//#include <gdiplus.h>

#pragma region

#define SECOND_TIMER 1000

InteractiveFusion openGLWin;
MeshHelper meshHelper;

const char className[] = "OpenGLWindow";
TCHAR Keys::kp[256] = { 0 };

//STATUS BAR
HWND statusHandle;

std::vector<HWND> progressionUI;
HWND statusText;
HWND statusPercentText;

//HELP DIALOG
HWND hButtonHelp;

//INTERACTION
std::vector<HWND> interactionUi;
HWND hButtonExport, hButtonReset;
HWND hButtonDuplicate;
HWND hButtonScale, hButtonRotateVertical, hButtonRotateHorizontal;

//WINDOW MODE TABS
HWND hPrepareText, hScanText, hSegmentationText, hInteractionText;

//WALL SELECTION
std::vector<HWND> wallSelectionUi;
HWND hButtonWallSizePlus, hButtonWallSizeMinus;
HWND hButtonWallSmoothnessPlus, hButtonWallSmoothnessMinus;
HWND hTextWallSizeLabel, hTextWallSize;
HWND hTextWallSmoothnessLabel, hTextWallSmoothness;
HWND hTextWalls;
HWND hHelpText1, hHelpText2;
HWND hButtonYes, hButtonNo;

//SEGMENTATION PREVIEW
std::vector<HWND> segmentationPreviewUi;
HWND hButtonSegmentationFinish;
HWND hButtonSegmentationBegin;

std::vector<HWND> euclideanUi;
HWND hButtonClusterTolerancePlus, hButtonClusterToleranceMinus;
HWND hTextClusterToleranceLabel, hTextClusterTolerance;

std::vector<HWND> regionGrowthUi;
HWND hButtonRegionGrowthSegmentation, hButtonEuclideanSegmentation;
HWND hButtonRGSmoothnessPlus, hButtonRGSmoothnessMinus;
HWND hTextRGSmoothnessLabel, hTextRGSmoothness;
HWND hButtonRGCurvaturePlus, hButtonRGCurvatureMinus;
HWND hTextRGCurvatureLabel, hTextRGCurvature;
HWND hButtonRGNeighborsPlus, hButtonRGNeighborsMinus;
HWND hTextRGNeighborsLabel, hTextRGNeighbors;
HWND hButtonRGKSearchPlus, hButtonRGKSearchMinus;
HWND hTextRGKSearchLabel, hTextRGKSearch;

//FONTS
HFONT uiFont, mediumUiFont, smallUiFont;
HFONT statusFont;

//BITMAPS
HBITMAP trashBmp;
HBITMAP trashBmp_mask;

//ICONS
std::vector<HWND> helpUi;
HICON hDeleteIcon;

//BRUSHES
HBRUSH hBackground = CreateSolidBrush(RGB(30, 30, 30));
HBRUSH buttonDefaultBrush, buttonPressedBrush, buttonActiveBrush;
HBRUSH buttonGreenBrush, buttonGreenPressedBrush, buttonGreenInactiveBrush, buttonRedBrush, buttonRedPressedBrush, buttonRedInactiveBrush;
HBRUSH buttonBlueBrush;

//PENS
HPEN buttonDefaultPen, buttonPressedPen, buttonInactivePen, buttonModePen;

//EDIT CONTROLS
WNDPROC oldEditProc;
HWND editKSearchHandle, editMinClustersHandle, editCarryDistanceHandle, editMaxClustersHandle, editNonHandle, editSmoothnessHandle, editCurvatureHandle, editFillHoleHandle, editRemoveComponentHandle;

//DEBUG UI
HWND debugHandle;
int debugWidth = 0;


//OTHER STUFF (DO I EVEN NEED THESSE THINGS?)
HWND hModeArrow;
HWND hCheckBoxDuplicate;


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

	ShowWindow(openGLWin.parent, SW_SHOW);

	uiFont = CreateFont(40, 0, 0, 0, FW_REGULAR, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");
	mediumUiFont = CreateFont(30, 0, 0, 0, FW_REGULAR, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");
	smallUiFont = CreateFont(20, 0, 0, 0, FW_REGULAR, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, L"Open Sans");


	hPrepareText = CreateWindowEx(0, L"BUTTON", L"PREPARE", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, openGLWin.parent, (HMENU)IDC_BUTTON_MODE_PREPARE, NULL, 0);
	hSegmentationText = CreateWindowEx(0, L"BUTTON", L"SEGMENTATION", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, openGLWin.parent, (HMENU)IDC_BUTTON_MODE_SEGMENTATION, NULL, 0);
	hScanText = CreateWindowEx(0, L"BUTTON", L"SCAN", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, openGLWin.parent, (HMENU)IDC_BUTTON_MODE_SCAN, NULL, 0);

	hInteractionText = CreateWindowEx(0, L"BUTTON", L"INTERACTION", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 250, 50, 150, 50, openGLWin.parent, (HMENU)IDC_BUTTON_MODE_INTERACTION, NULL, 0);

	trashBmp = LoadBitmap(openGLWin.appInstance, MAKEINTRESOURCE(IDB_MODE_ARROW));
	trashBmp_mask = LoadBitmap(openGLWin.appInstance, MAKEINTRESOURCE(IDB_MODE_ARROW_MASK));

	//hModeArrow = CreateWindowEx(0, L"STATIC", L"a", WS_CHILD | WS_VISIBLE | SS_OWNERDRAW | WS_EX_TOPMOST, 250, 50, 150, 50, openGLWin.parent, (HMENU)IDC_BUTTON_DELETE, NULL, 0);

	buttonDefaultBrush = CreateSolidBrush(RGB(20, 20, 20));
	buttonPressedBrush = CreateSolidBrush(RGB(40, 40, 40));
	buttonDefaultPen = CreatePen(PS_SOLID, 2, RGB(100, 100, 100));
	buttonInactivePen = CreatePen(PS_SOLID, 1, RGB(50, 50, 50));
	buttonPressedPen = CreatePen(PS_SOLID, 2, RGB(160, 160, 160));
	buttonActiveBrush = CreateSolidBrush(RGB(0, 0, 255));
	buttonModePen = CreatePen(PS_SOLID, 2, RGB(50, 50, 50));
	buttonGreenBrush = CreateSolidBrush(RGB(0, 170, 0));
	buttonGreenInactiveBrush = CreateSolidBrush(RGB(0, 50, 0));
	buttonBlueBrush = CreateSolidBrush(RGB(0, 0, 170));
	buttonGreenPressedBrush = CreateSolidBrush(RGB(0, 100, 0));
	buttonRedBrush = CreateSolidBrush(RGB(170, 0, 0));
	buttonRedPressedBrush = CreateSolidBrush(RGB(100, 0, 0));
	buttonRedInactiveBrush = CreateSolidBrush(RGB(50, 0, 0));

	openGLWin.SetWindowMode(MODE_PREPARE_SCANNING);
	//openGLWin.SetWindowState(START);
	StartKinectFusion(openGLWin.parent, hInstance, StartOpenGLThread, SetWindowMode,openGLWin.fusionExplorer, openGLWin.fusionHandle);
}

#pragma region

WindowMode InteractiveFusion::GetWindowMode()
{
	return mode;
}

void InteractiveFusion::SetWindowMode(WindowMode wMode)
{
	mode = wMode;

	if (mode == MODE_INTERACTION)
	{
		DetermineMeshQuality();
		ShowWindow(glWindowHandle, SW_SHOW);
		EnableWindow(hPrepareText, true);
		EnableWindow(hScanText, true);
		EnableWindow(hInteractionText, false);
		EnableWindow(hSegmentationText, true);
	}
	else if (mode == MODE_SCANNING)
	{
		cDebug::DbgOut(L"activate scanning");
		EnableWindow(hPrepareText, true);
		EnableWindow(hScanText, false);
		EnableWindow(hInteractionText, true);
		EnableWindow(hSegmentationText, true);
	}
	else if (mode == MODE_PREPARE_SCANNING)
	{
		EnableWindow(hPrepareText, false);
		EnableWindow(hScanText, true);
		EnableWindow(hInteractionText, true);
		if (openGLWin.fusionExplorer)
		{ 
			if (openGLWin.fusionExplorer->GetWindowState() != START)
				openGLWin.fusionExplorer->SetWindowState(START);
		}
		EnableWindow(hSegmentationText, true);
	}
	else if (mode == MODE_SEGMENTATION)
	{
		DetermineMeshQuality();
		ShowWindow(glWindowHandle, SW_SHOW);
		EnableWindow(hPrepareText, true);
		EnableWindow(hScanText, true);
		EnableWindow(hInteractionText, true);
		EnableWindow(hSegmentationText, false);
	}

	MoveButtonsOnResize();
}

WindowState InteractiveFusion::GetWindowState()
{
	return state;
}

bool InteractiveFusion::IsHandleInUI(HWND handle, std::vector<HWND> handles)
{
	for (int i = 0; i < handles.size(); i++)
	{
		if (handle == handles[i])
			return true;
	}
	return false;
}

void InteractiveFusion::HideUI(std::vector<HWND> handles)
{
	for (int i = 0; i < handles.size(); i++)
	{
		ShowWindow(handles[i], SW_HIDE);
	}
}

void InteractiveFusion::ShowUI(std::vector<HWND> handles)
{
	for (int i = 0; i < handles.size(); i++)
	{
		ShowWindow(handles[i], SW_SHOW);
	}
}

void InteractiveFusion::HideWholeUI()
{
	for (int i = 0; i < segmentationPreviewUi.size(); i++)
	{
		ShowWindow(segmentationPreviewUi[i], SW_HIDE);
	}
	for (int i = 0; i < euclideanUi.size(); i++)
	{
		ShowWindow(euclideanUi[i], SW_HIDE);
	}
	for (int i = 0; i < regionGrowthUi.size(); i++)
	{
		ShowWindow(regionGrowthUi[i], SW_HIDE);
	}
	for (int i = 0; i < wallSelectionUi.size(); i++)
	{
		ShowWindow(wallSelectionUi[i], SW_HIDE);
	}
	for (int i = 0; i < interactionUi.size(); i++)
	{
		ShowWindow(interactionUi[i], SW_HIDE);
	}
	for (int i = 0; i < progressionUI.size(); i++)
	{
		ShowWindow(progressionUI[i], SW_HIDE);
	}
}

void InteractiveFusion::DeactivateWholeUI()
{
	for (int i = 0; i < segmentationPreviewUi.size(); i++)
	{
		EnableWindow(segmentationPreviewUi[i], false);
	}
	for (int i = 0; i < euclideanUi.size(); i++)
	{
		EnableWindow(euclideanUi[i], false);
	}
	for (int i = 0; i < regionGrowthUi.size(); i++)
	{
		EnableWindow(regionGrowthUi[i], false);
	}
	for (int i = 0; i < wallSelectionUi.size(); i++)
	{
		EnableWindow(wallSelectionUi[i], false);
	}
	for (int i = 0; i < interactionUi.size(); i++)
	{
		EnableWindow(interactionUi[i], false);
	}
	for (int i = 0; i < progressionUI.size(); i++)
	{
		EnableWindow(progressionUI[i], false);
	}
}

void InteractiveFusion::ActivateWholeUI()
{
	for (int i = 0; i < segmentationPreviewUi.size(); i++)
	{
		EnableWindow(segmentationPreviewUi[i], true);
	}
	for (int i = 0; i < euclideanUi.size(); i++)
	{
		EnableWindow(euclideanUi[i], true);
	}
	for (int i = 0; i < regionGrowthUi.size(); i++)
	{
		EnableWindow(regionGrowthUi[i], true);
	}
	for (int i = 0; i < wallSelectionUi.size(); i++)
	{
		EnableWindow(wallSelectionUi[i], true);
	}
	for (int i = 0; i < interactionUi.size(); i++)
	{
		EnableWindow(interactionUi[i], true);
	}
	for (int i = 0; i < progressionUI.size(); i++)
	{
		EnableWindow(progressionUI[i], true);
	}
}

void InteractiveFusion::SetWindowState(WindowState wState)
{
	state = wState;
	HideWholeUI();
	openGLWin.glControl.SetOffSetBottom(0);
	openGLWin.glControl.SetOffSetRight(0);
	ShowWindow(hPrepareText, SW_SHOW);
	ShowWindow(hSegmentationText, SW_SHOW);
	ShowWindow(hScanText, SW_SHOW);
	ShowWindow(hInteractionText, SW_SHOW);

	if (state == WALL_SELECTION)
	{
		glCamera.mode = CAMERA_FREE;
		openGLWin.glControl.SetOffSetBottom(250);

		ShowUI(wallSelectionUi);
	}
	if (state == SEGMENTATION)
	{
		glCamera.mode = CAMERA_FREE;
		openGLWin.glControl.SetOffSetRight(250);
		//ShowUI(progressionUI);
		/*ShowUI(segmentationPreviewUi);
		if (glSegmentation.GetSegmentationMode() == SEGMENTATION_REGIONGROWTH)
		ShowUI(regionGrowthUi);
		else if (glSegmentation.GetSegmentationMode() == SEGMENTATION_EUCLIDEAN)
		ShowUI(euclideanUi);*/
	}
	if (state == SEGMENTATION_PREVIEW)
	{
		glCamera.mode = CAMERA_FREE;
		openGLWin.glControl.SetOffSetRight(250);

		ShowUI(segmentationPreviewUi);
		if (glSegmentation.GetSegmentationMode() == SEGMENTATION_REGIONGROWTH)
			ShowUI(regionGrowthUi);
		else if (glSegmentation.GetSegmentationMode() == SEGMENTATION_EUCLIDEAN)
			ShowUI(euclideanUi);
	}
	if (state == DEFAULT)
	{
		glCamera.mode = CAMERA_SENSOR;
		openGLWin.glControl.SetOffSetBottom(0);
		openGLWin.glControl.SetOffSetRight(250);

		ShowUI(interactionUi);
	}
	
	RECT rRect; GetClientRect(GetParent(openGLWin.glWindowHandle), &rRect);
	openGLWin.glControl.ResizeOpenGLViewportFull(rRect.right, rRect.bottom);
	MoveButtonsOnResize();
	
}

void InteractiveFusion::SetWindowBusyState(WindowBusyState bState)
{
	busyState = bState;

	if (busyState == IF_BUSYSTATE_BUSY)
	{ 
		//ShowUI(helpUi);
		//DeactivateWholeUI();
	}
	else if (busyState == IF_BUSYSTATE_DEFAULT)
	{ 
		//HideUI(helpUi);
		//ActivateWholeUI();
	}

	MoveButtonsOnResize();

}

WindowBusyState InteractiveFusion::GetWindowBusyState()
{
	return busyState;
}

void InteractiveFusion::SetAnswer(Answer ans)
{
	answer = ans;
}

Answer InteractiveFusion::GetAnswer()
{
	return answer;
}
LPCWSTR InteractiveFusion::GetLastErrorStdStr()
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

HINSTANCE InteractiveFusion::GetInstance()
{
	return appInstance;
}

#pragma endregion Getter and Setter

#pragma region

//HANDLE hbitmap;

bool InteractiveFusion::CreateOpenGLWindow()
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
	wc.hbrBackground = hBackground;
	wc.lpszMenuName = NULL;
	wc.lpszClassName = (LPCWSTR)className;
	wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	ATOM ClassAtom = RegisterClassExW(&wc);

	RECT r = { 0, 0, 1024, 768 };
	GetClientRect(openGLWin.parent, &r);

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


	hButtonHelp = CreateWindowEx(0, L"Button", L"Alright", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_HELP_OK, openGLWin.appInstance, 0);
	
	helpUi.push_back(hButtonHelp);

	HideUI(helpUi);

	statusFont = CreateFont(22, 10, 0, 0, 700, 0, 0, 0, 0, 0, 0, 0, 0, TEXT("Courier New"));
	
	statusText = CreateWindowEx(0, L"STATIC", L"Is the highlighted area (part of) a wall, floor or ceiling?", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_STATUS, openGLWin.appInstance, 0);
	SendMessage(statusText, WM_SETFONT, (WPARAM)uiFont, TRUE);

	statusPercentText = CreateWindowEx(0, L"STATIC", L"10 %", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_STATUS_PERCENT, openGLWin.appInstance, 0);
	SendMessage(statusPercentText, WM_SETFONT, (WPARAM)uiFont, TRUE);

	progressionUI.push_back(statusText);
	progressionUI.push_back(statusPercentText);

	//hbitmap = LoadImage(GetModuleHandle(NULL), MAKEINTRESOURCE(IDB_BUTTON), IMAGE_BITMAP, 84, 36, LR_DEFAULTCOLOR);
	hButtonYes = CreateWindowEx(0, L"Button", L"Wall", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_YES, openGLWin.appInstance, 0);
	hButtonNo = CreateWindowEx(0, L"Button", L"No Wall", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_NO, openGLWin.appInstance, 0);

	hButtonSegmentationFinish = CreateWindowEx(0, L"Button", L"Done", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_SEGMENTATION_FINISH, openGLWin.appInstance, 0);

	hButtonSegmentationBegin = CreateWindowEx(0, L"Button", L"Segment", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_SEGMENTATION_BEGIN, openGLWin.appInstance, 0);

	hTextClusterTolerance = CreateWindowEx(0, L"STATIC", L"2cm", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_CLUSTERTOLERANCE, openGLWin.appInstance, 0);

	SendMessage(hTextClusterTolerance, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);
	hTextClusterToleranceLabel = CreateWindowEx(0, L"STATIC", L"Tolerance", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_CLUSTERTOLERANCE_LABEL, openGLWin.appInstance, 0);
	
	SendMessage(hTextClusterToleranceLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	hButtonClusterTolerancePlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_CLUSTERTOLERANCE_PLUS, openGLWin.appInstance, 0);
	hButtonClusterToleranceMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_CLUSTERTOLERANCE_MINUS, openGLWin.appInstance, 0);

	hButtonRegionGrowthSegmentation = CreateWindowEx(0, L"Button", L"RG", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_SEGMENTATION_REGIONGROWTH, openGLWin.appInstance, 0);
	hButtonEuclideanSegmentation = CreateWindowEx(0, L"Button", L"Euclidean", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_SEGMENTATION_EUCLIDEAN, openGLWin.appInstance, 0);
	
	hButtonRGSmoothnessPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_SMOOTHNESS_PLUS, openGLWin.appInstance, 0);
	hButtonRGSmoothnessMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_SMOOTHNESS_MINUS, openGLWin.appInstance, 0);

	hTextRGSmoothness = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_SMOOTHNESS, openGLWin.appInstance, 0);

	SendMessage(hTextRGSmoothness, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);
	hTextRGSmoothnessLabel = CreateWindowEx(0, L"STATIC", L"Smoothness", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_SMOOTHNESS_LABEL, openGLWin.appInstance, 0);

	SendMessage(hTextRGSmoothnessLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	hButtonRGCurvaturePlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_CURVATURE_PLUS, openGLWin.appInstance, 0);
	hButtonRGCurvatureMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_CURVATURE_MINUS, openGLWin.appInstance, 0);

	hButtonRGNeighborsPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_NEIGHBORS_PLUS, openGLWin.appInstance, 0);
	hButtonRGNeighborsMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_NEIGHBORS_MINUS, openGLWin.appInstance, 0);

	hButtonRGKSearchPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_KSEARCH_PLUS, openGLWin.appInstance, 0);
	hButtonRGKSearchMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RG_KSEARCH_MINUS, openGLWin.appInstance, 0);

	hTextRGCurvature = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_CURVATURE, openGLWin.appInstance, 0);

	SendMessage(hTextRGCurvature, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);
	hTextRGCurvatureLabel = CreateWindowEx(0, L"STATIC", L"Curvature", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_CURVATURE_LABEL, openGLWin.appInstance, 0);

	SendMessage(hTextRGCurvatureLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	hTextRGNeighbors = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_NEIGHBORS, openGLWin.appInstance, 0);

	SendMessage(hTextRGNeighbors, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);
	hTextRGNeighborsLabel = CreateWindowEx(0, L"STATIC", L"Neighbors", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_NEIGHBORS_LABEL, openGLWin.appInstance, 0);

	SendMessage(hTextRGNeighborsLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	hTextRGKSearch = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_KSEARCH, openGLWin.appInstance, 0);

	SendMessage(hTextRGKSearch, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);
	hTextRGKSearchLabel = CreateWindowEx(0, L"STATIC", L"KSearch", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_RG_KSEARCH_LABEL, openGLWin.appInstance, 0);

	SendMessage(hTextRGKSearchLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);


	segmentationPreviewUi.push_back(hButtonSegmentationFinish);
	segmentationPreviewUi.push_back(hButtonSegmentationBegin);
	segmentationPreviewUi.push_back(hButtonRegionGrowthSegmentation);
	segmentationPreviewUi.push_back(hButtonEuclideanSegmentation);

	regionGrowthUi.push_back(hButtonRGSmoothnessPlus);
	regionGrowthUi.push_back(hButtonRGSmoothnessMinus);
	regionGrowthUi.push_back(hTextRGSmoothness);
	regionGrowthUi.push_back(hTextRGSmoothnessLabel);

	regionGrowthUi.push_back(hButtonRGCurvaturePlus);
	regionGrowthUi.push_back(hButtonRGCurvatureMinus);
	regionGrowthUi.push_back(hButtonRGNeighborsPlus);
	regionGrowthUi.push_back(hButtonRGNeighborsMinus);
	regionGrowthUi.push_back(hButtonRGKSearchPlus);
	regionGrowthUi.push_back(hButtonRGKSearchMinus);
	regionGrowthUi.push_back(hTextRGCurvature);
	regionGrowthUi.push_back(hTextRGCurvatureLabel);
	regionGrowthUi.push_back(hTextRGNeighbors);
	regionGrowthUi.push_back(hTextRGNeighborsLabel);
	regionGrowthUi.push_back(hTextRGKSearch);
	regionGrowthUi.push_back(hTextRGKSearchLabel);

	euclideanUi.push_back(hTextClusterTolerance);
	euclideanUi.push_back(hTextClusterToleranceLabel);
	euclideanUi.push_back(hButtonClusterTolerancePlus);
	euclideanUi.push_back(hButtonClusterToleranceMinus);
	
	openGLWin.UpdateSegmentationPreviewValues();

	hButtonWallSizePlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_WALLSIZE_PLUS, openGLWin.appInstance, 0);
	hButtonWallSizeMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_WALLSIZE_MINUS, openGLWin.appInstance, 0);
	hButtonWallSmoothnessPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_WALLSMOOTHNESS_PLUS, openGLWin.appInstance, 0);
	hButtonWallSmoothnessMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_WALLSMOOTHNESS_MINUS, openGLWin.appInstance, 0);
	

	hTextWalls = CreateWindowEx(0, L"STATIC", L"Is the highlighted area (part of) a wall, floor or ceiling?", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_WALL, openGLWin.appInstance, 0);
	SendMessage(hTextWalls, WM_SETFONT, (WPARAM)uiFont, TRUE);


	hTextWallSize = CreateWindowEx(0, L"STATIC", L"2cm", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_WALLSIZE, openGLWin.appInstance, 0);

	//SetDlgItemText(openGLWin.glWindowParent, IDC_STATIC_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
	SendMessage(hTextWallSize, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);
	hTextWallSizeLabel = CreateWindowEx(0, L"STATIC", L"Thickness", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_WALLSIZE_LABEL, openGLWin.appInstance, 0);
	//SetDlgItemText(openGLWin.glWindowParent, IDC_STATIC_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
	SendMessage(hTextWallSizeLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	hTextWallSmoothness = CreateWindowEx(0, L"STATIC", L"2cm", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_WALLSMOOTHNESS, openGLWin.appInstance, 0);
	//SetDlgItemText(openGLWin.glWindowParent, IDC_STATIC_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
	SendMessage(hTextWallSmoothness, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	hTextWallSmoothnessLabel = CreateWindowEx(0, L"STATIC", L"Smoothness", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_WALLSMOOTHNESS_LABEL, openGLWin.appInstance, 0);
	//SetDlgItemText(openGLWin.glWindowParent, IDC_STATIC_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
	SendMessage(hTextWallSmoothnessLabel, WM_SETFONT, (WPARAM)mediumUiFont, TRUE);

	openGLWin.UpdateWallSelectionValues();
	

	hHelpText1 = CreateWindowEx(0, L"STATIC", L"If the highlighted area is too thick or too thin, change the wall thickness.", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_HELP1, openGLWin.appInstance, 0);
	SendMessage(hHelpText1, WM_SETFONT, (WPARAM)smallUiFont, TRUE);

	hHelpText2 = CreateWindowEx(0, L"STATIC", L"If the highlighted area has gaps, decrease smoothness. If it is not flat enough (contains other objects), increase it.", WS_CHILD | WS_VISIBLE | SS_CENTER | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, hWnd, (HMENU)IDC_STATIC_TEXT_HELP2, openGLWin.appInstance, 0);
	SendMessage(hHelpText2, WM_SETFONT, (WPARAM)smallUiFont, TRUE);
	
	wallSelectionUi.push_back(hTextWalls);
	wallSelectionUi.push_back(hTextWallSize);
	wallSelectionUi.push_back(hTextWallSizeLabel);
	wallSelectionUi.push_back(hTextWallSmoothness);
	wallSelectionUi.push_back(hTextWallSmoothnessLabel);
	wallSelectionUi.push_back(hHelpText1);
	wallSelectionUi.push_back(hHelpText2);
	wallSelectionUi.push_back(hButtonYes);
	wallSelectionUi.push_back(hButtonNo);
	wallSelectionUi.push_back(hButtonWallSizePlus);
	wallSelectionUi.push_back(hButtonWallSizeMinus);
	wallSelectionUi.push_back(hButtonWallSmoothnessPlus);
	wallSelectionUi.push_back(hButtonWallSmoothnessMinus);

	//hTextWalls = CreateWindowEx(0, L"Text", L"Is this (part of) a floor/wall?", WS_CHILD | WS_VISIBLE | BS_BITMAP, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_NO, appInstance, 0);
	hButtonExport = CreateWindowEx(0, L"BUTTON", L"Export", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_EXPORT, NULL, 0);
	hButtonDuplicate = CreateWindowEx(0, L"BUTTON", L"Duplicate", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_DUPLICATE, NULL, 0);
	hButtonReset = CreateWindowEx(0, L"BUTTON", L"Reset", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_RESET, NULL, 0);
	hDeleteIcon = (HICON)LoadImage(openGLWin.appInstance, MAKEINTRESOURCE(IDI_TRASH), IMAGE_ICON, 100, 100, NULL);


	hButtonScale = CreateWindowEx(0, L"BUTTON", L"Scale", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_SCALE, NULL, 0);
	hButtonRotateVertical = CreateWindowEx(0, L"BUTTON", L"Rotate ^v", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_ROTATE_VERTICAL, NULL, 0);
	hButtonRotateHorizontal = CreateWindowEx(0, L"BUTTON", L"Rotate <>", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, hWnd, (HMENU)IDC_BUTTON_ROTATE_HORIZONTAL, NULL, 0);

	interactionUi.push_back(hButtonExport);
	interactionUi.push_back(hButtonDuplicate);
	interactionUi.push_back(hButtonScale);
	interactionUi.push_back(hButtonRotateVertical);
	interactionUi.push_back(hButtonRotateHorizontal);
	interactionUi.push_back(hButtonReset);
	//hDeleteIcon = (HICON)LoadImage(openGLWin.appInstance, MAKEINTRESOURCE(IDI_TRASH), IMAGE_ICON, 128, 128, NULL);
	//SendMessage(hButtonDelete, STM_SETIMAGE, IMAGE_ICON, (LPARAM)hDeleteIcon);
	HideWholeUI();


	
//debugHandle = CreateWindowEx(WS_EX_OVERLAPPEDWINDOW, MAKEINTRESOURCE(IDD_DEBUG), L"Debug", WS_CHILD | WS_VISIBLE | DS_CONTROL, 50, 50, 300, 300, hWnd, 0, appInstance, 0);
	debugHandle = CreateDialog(openGLWin.appInstance, MAKEINTRESOURCE(IDD_DEBUG), hWnd, (DLGPROC)DebugDlgProc);
	ShowWindow(debugHandle, SW_HIDE);

	
	ShowWindow(parent, SW_SHOWMAXIMIZED);


	//SendMessage(d, BM_SETIMAGE, IMAGE_BITMAP, (LPARAM)hbitmap);
	
	//hbit = LoadBitmap(hInstance, L"Bit");
	//SendMessage(d, BM_SETIMAGE, (WPARAM)IMAGE_BITMAP, (LPARAM)hbit);


	return true;
}

void InteractiveFusion::ReleaseOpenGL()
{
	glControl.ReleaseOpenGLControl(&glControl);
}

void InteractiveFusion::ShutdownWindow()
{
	winDestroyed = true;
	
	if (debugHandle)
		DestroyWindow(debugHandle);
	if (openGLWin.fusionHandle)
		DestroyWindow(openGLWin.fusionHandle);
	if (openGLWin.glWindowHandle)
		DestroyWindow(openGLWin.glWindowHandle);
	if (openGLWin.parent)
		DestroyWindow(openGLWin.parent);

	UnregisterClass((LPCWSTR)className, appInstance);
	

	
	
	
}

#pragma endregion Initialize / Shutdown Window

void InteractiveFusion::ResumeScanning()
{
	HideWholeUI();
	//HideAllButtons();
	ShowWindow(openGLWin.glWindowHandle, SW_HIDE);
	openGLWin.SetWindowMode(MODE_SCANNING);
	ResumeKinectFusion();
}

#pragma region

int WINAPI LoadMeshThread()
{
	InitialLoading();
	return 0;
}

void StartOpenGLThread(int testMode)
{
	
		
	openGLWin.testMode = testMode;
	openGLWin.SetWindowMode(MODE_INTERACTION);
	openGLWin.SetWindowState(INITIALIZING);

	if (openGLWin.glWindowHandle)
	{
		ResetForResume();
		CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&LoadMeshThread, 0, 0, NULL);
	}
	else
	{
		//openGLWin.fusionExplorer = expl;
		openGLWin.interactionThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&GLViewportThreadMain, 0, 0, &openGLWin.threadId);
	}
	if (openGLWin.interactionThread == NULL)
		return;
	return;

}

void SetWindowMode(int wMode)
{
	openGLWin.SetWindowMode((WindowMode)wMode);
}

DWORD InteractiveFusion::GetThreadID()
{
	return threadId;
}

bool DrawButton(WPARAM wParam, LPARAM lParam)
{
	LPDRAWITEMSTRUCT item = (LPDRAWITEMSTRUCT)lParam;
	if (IDC_BUTTON_MODE_INTERACTION == LOWORD(wParam) ||
		IDC_BUTTON_MODE_SCAN == LOWORD(wParam) ||
		IDC_BUTTON_MODE_PREPARE == LOWORD(wParam) ||
		IDC_BUTTON_MODE_SEGMENTATION == LOWORD(wParam) ||
		IDC_BUTTON_SEGMENTATION_REGIONGROWTH == LOWORD(wParam) ||
		IDC_BUTTON_SEGMENTATION_EUCLIDEAN == LOWORD(wParam) ||
		IDC_BUTTON_HELP_OK == LOWORD(wParam))
	{
		if (IDC_BUTTON_SEGMENTATION_REGIONGROWTH == LOWORD(wParam) ||
			IDC_BUTTON_SEGMENTATION_EUCLIDEAN == LOWORD(wParam))
			SelectObject(item->hDC, mediumUiFont);
		else
			SelectObject(item->hDC, uiFont);
		FillRect(item->hDC, &item->rcItem, hBackground);
		//SelectObject(item->hDC, hBackground);
		HBRUSH bgBrush = CreateSolidBrush(RGB(10, 10, 10));
		HBRUSH inactiveBrush = CreateSolidBrush(RGB(60, 60, 60));
		SelectObject(item->hDC, bgBrush);
		if (IsWindowEnabled(item->hwndItem))
		{
			SetTextColor(item->hDC, RGB(245, 245, 245));
			SelectObject(item->hDC, buttonModePen);
		}
		else if (item->itemState & ODS_SELECTED)
		{
			SetTextColor(item->hDC, RGB(245, 245, 245));
			SelectObject(item->hDC, buttonModePen);
			//SelectObject(item->hDC, buttonPressedBrush);
			//SelectObject(item->hDC, buttonPressedPen);
		}
		else
		{
			SelectObject(item->hDC, inactiveBrush);
		//	FillRect(item->hDC, &item->rcItem, CreateSolidBrush(RGB(50,50,50)));
			SelectObject(item->hDC, CreatePen(PS_SOLID, 2, RGB(100,100,100)));
			SetTextColor(item->hDC, RGB(245, 245, 245));
			//SelectObject(item->hDC, buttonDefaultPen);
		}

		SetBkMode(item->hDC, TRANSPARENT);
		
		RoundRect(item->hDC, item->rcItem.left, item->rcItem.top, item->rcItem.right, item->rcItem.bottom, 0, 0);
		int len;
		len = GetWindowTextLength(item->hwndItem);
		LPSTR lpBuff = new char[len + 1];
		GetWindowTextA(item->hwndItem, lpBuff, len + 1);

		if (IDC_BUTTON_MODE_SCAN == LOWORD(wParam))
			DrawTextA(item->hDC, lpBuff, len, &item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
		else
			DrawTextA(item->hDC, lpBuff, len, &item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
		DeleteObject(lpBuff);
		DeleteObject(bgBrush);
		DeleteObject(inactiveBrush);
	}

	if (IDC_BUTTON_DELETE == LOWORD(wParam))
	{
		/*LPDRAWITEMSTRUCT item = (LPDRAWITEMSTRUCT)lParam;
		DrawIconEx(item->hDC, 0, 0, hDeleteIcon, 100, 100, 0, NULL, DI_NORMAL);*/
		HDC hMemDC = CreateCompatibleDC(item->hDC);
		COLORREF crTransColor = RGB(0, 0, 0);
		// Select the bitmap into the device context
		HBITMAP hOldBitmap = (HBITMAP)SelectObject(hMemDC, trashBmp);

		// Draw the bitmap to the destination device context

		TransparentBlt(item->hDC, 0, 0, 50, 50, hMemDC, 0, 0, 50, 50, crTransColor);

		// Restore and delete the memory device context
		SelectObject(hMemDC, hOldBitmap);
		DeleteDC(hMemDC);
		
		return TRUE;
	}
	if (IDC_BUTTON_EXPORT == LOWORD(wParam)
		|| IDC_BUTTON_YES == LOWORD(wParam)
		|| IDC_BUTTON_NO == LOWORD(wParam)
		|| IDC_BUTTON_DUPLICATE == LOWORD(wParam)
		|| IDC_BUTTON_RESET == LOWORD(wParam)
		|| IDC_BUTTON_SCALE == LOWORD(wParam)
		|| IDC_BUTTON_ROTATE_VERTICAL == LOWORD(wParam)
		|| IDC_BUTTON_ROTATE_HORIZONTAL == LOWORD(wParam)
		|| IDC_BUTTON_SEGMENTATION_FINISH == LOWORD(wParam)
		|| IDC_BUTTON_SEGMENTATION_BEGIN == LOWORD(wParam)
		|| openGLWin.IsHandleInUI(item->hwndItem, wallSelectionUi)
		|| openGLWin.IsHandleInUI(item->hwndItem, euclideanUi)
		|| openGLWin.IsHandleInUI(item->hwndItem, regionGrowthUi))
	{

		if (IDC_BUTTON_SCALE == LOWORD(wParam) || IDC_BUTTON_ROTATE_VERTICAL == LOWORD(wParam) || IDC_BUTTON_ROTATE_HORIZONTAL == LOWORD(wParam))
			SelectObject(item->hDC, smallUiFont);
		else
			SelectObject(item->hDC, uiFont);
		FillRect(item->hDC, &item->rcItem, hBackground);
		SelectObject(item->hDC, buttonDefaultBrush);
		if (!IsWindowEnabled(item->hwndItem))
		{
			SetTextColor(item->hDC, RGB(50, 50, 50));

			if (IDC_BUTTON_EXPORT == LOWORD(wParam) || IDC_BUTTON_YES == LOWORD(wParam) || IDC_BUTTON_SEGMENTATION_FINISH == LOWORD(wParam))
			{
				SelectObject(item->hDC, buttonDefaultPen);
				SelectObject(item->hDC, buttonGreenInactiveBrush);
			}
			else if (IDC_BUTTON_RESET == LOWORD(wParam) || IDC_BUTTON_NO == LOWORD(wParam))
			{
				SelectObject(item->hDC, buttonDefaultPen);
				SelectObject(item->hDC, buttonRedInactiveBrush);
			}
			else
			{
				SelectObject(item->hDC, buttonInactivePen);
			}
		}
		else if (item->itemState & ODS_SELECTED)
		{
			SetTextColor(item->hDC, RGB(245, 245, 245));
			if (IDC_BUTTON_EXPORT == LOWORD(wParam) || IDC_BUTTON_YES == LOWORD(wParam) || IDC_BUTTON_SEGMENTATION_FINISH == LOWORD(wParam))
			{
				SelectObject(item->hDC, buttonDefaultPen);
				SelectObject(item->hDC, buttonGreenPressedBrush);
			}
			else if (IDC_BUTTON_RESET == LOWORD(wParam) || IDC_BUTTON_NO == LOWORD(wParam))
			{
				SelectObject(item->hDC, buttonDefaultPen);
				SelectObject(item->hDC, buttonRedPressedBrush);
			}
			else
			{
				SelectObject(item->hDC, buttonPressedBrush);
				SelectObject(item->hDC, buttonPressedPen);
			}
		}
		else
		{
			SetTextColor(item->hDC, RGB(240, 240, 240));
			if (IDC_BUTTON_EXPORT == LOWORD(wParam) || IDC_BUTTON_YES == LOWORD(wParam) || IDC_BUTTON_SEGMENTATION_FINISH == LOWORD(wParam))
			{
				SelectObject(item->hDC, buttonDefaultPen);
				SelectObject(item->hDC, buttonGreenBrush);
			}
			else if (IDC_BUTTON_RESET == LOWORD(wParam) || IDC_BUTTON_NO == LOWORD(wParam))
			{
				SelectObject(item->hDC, buttonDefaultPen);
				SelectObject(item->hDC, buttonRedBrush);
			}
			else
			{
				SelectObject(item->hDC, buttonDefaultPen);
			}
		}
		if (IDC_BUTTON_DUPLICATE == LOWORD(wParam) && openGLWin.duplicationMode)
			SelectObject(item->hDC, buttonActiveBrush);
		if (IDC_BUTTON_SCALE == LOWORD(wParam) && glSelector.GetManipulationMode() == MANIPULATION_SCALE)
			SelectObject(item->hDC, buttonActiveBrush);
		else if (IDC_BUTTON_ROTATE_HORIZONTAL == LOWORD(wParam) && glSelector.GetManipulationMode() == MANIPULATION_ROTATE_Y)
			SelectObject(item->hDC, buttonActiveBrush);
		else if (IDC_BUTTON_ROTATE_VERTICAL == LOWORD(wParam) && glSelector.GetManipulationMode() == MANIPULATION_ROTATE_X)
			SelectObject(item->hDC, buttonActiveBrush);
			
		
		SetBkMode(item->hDC, TRANSPARENT);
		
		if (IDC_BUTTON_YES == LOWORD(wParam) || IDC_BUTTON_NO == LOWORD(wParam) || IDC_BUTTON_WALLSIZE_MINUS == LOWORD(wParam)
			|| IDC_BUTTON_WALLSIZE_PLUS == LOWORD(wParam) || IDC_BUTTON_WALLSMOOTHNESS_MINUS == LOWORD(wParam) ||
			IDC_BUTTON_WALLSMOOTHNESS_PLUS == LOWORD(wParam))
			RoundRect(item->hDC, item->rcItem.left, item->rcItem.top, item->rcItem.right, item->rcItem.bottom, 20, 20);
		else
			RoundRect(item->hDC, item->rcItem.left, item->rcItem.top, item->rcItem.right, item->rcItem.bottom, 500, 500);
		int len;
		len = GetWindowTextLength(item->hwndItem);
		LPSTR lpBuff = new char[len + 1];
		GetWindowTextA(item->hwndItem, lpBuff, len + 1);

		DrawTextA(item->hDC, lpBuff, len, &item->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
		DeleteObject(lpBuff);
	}
	return TRUE;
}

LRESULT CALLBACK GLViewportProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;
	LPDRAWITEMSTRUCT Item;
	switch (msg)
	{
	case WM_CREATE:
		break;
	case WM_PAINT:
		BeginPaint(hWnd, &ps);
		EndPaint(hWnd, &ps);
		break;
	case WM_CLOSE:
		DestroyWindow(hWnd);
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
		GLProcessUI(wParam, lParam);
		break;
	case WM_DRAWITEM:

		return DrawButton(wParam, lParam);
		break;
	case WM_CTLCOLORSTATIC:
	{
			HDC hdc = reinterpret_cast<HDC>(wParam);
			SetBkMode((HDC)wParam, TRANSPARENT);
			SetTextColor(hdc, RGB(255, 255, 255));

			return (LRESULT)hBackground;
	}
		break;
	case WM_SIZE:	
		break;
	case WM_DESTROY:
		DeleteObject(hBackground);
		DeleteObject(buttonDefaultBrush);
		DeleteObject(buttonDefaultPen);
		DeleteObject(buttonInactivePen);
		DeleteObject(buttonModePen);
		DeleteObject(buttonPressedBrush);
		DeleteObject(buttonPressedPen);
		DeleteObject(buttonActiveBrush);
		DeleteObject(buttonGreenBrush);
		DeleteObject(buttonGreenPressedBrush);
		DeleteObject(buttonGreenInactiveBrush);
		DeleteObject(buttonRedBrush);
		DeleteObject(buttonRedPressedBrush);
		DeleteObject(buttonRedInactiveBrush);
		DeleteObject(buttonBlueBrush);
		DeleteObject(statusFont);
		DeleteObject(uiFont);
		DeleteObject(smallUiFont);
		DeleteObject(mediumUiFont);
		
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
	if (!openGLWin.glControl.InitOpenGL(openGLWin.appInstance, openGLWin.glWindowHandle, 3, 3, Initialize, Render, Release, &openGLWin.fusionExplorer->m_processor, &openGLWin.glControl))
	{
		MessageBox(0, L"Error with initOpenGL",
			L"ERROR!", MB_OK);
	}
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&LoadMeshThread, 0, 0, NULL);

	RECT rRect; GetClientRect(GetParent(openGLWin.glWindowHandle), &rRect);
	openGLWin.glControl.ResizeOpenGLViewportFull(rRect.right, rRect.bottom);
	openGLWin.glControl.SetProjection3D(45.0f, (float)openGLWin.glControl.GetViewportWidth() / (float)openGLWin.glControl.GetViewportHeight(), 0.1f, 1000.0f);
	openGLWin.glControl.SetOrtho2D(openGLWin.glControl.GetViewportWidth(), openGLWin.glControl.GetViewportHeight());
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
			{
				done = true;
			}
		}

		openGLWin.UpdateTimer();

		openGLWin.glControl.Render(&openGLWin.glControl);
		//cDebug::DbgOut(L"hello?");
		
		//RedrawWindow(hButtonDelete, &ddRect, NULL, RDW_ERASE | RDW_INVALIDATE);
		/*RECT ddRect;
		GetClientRect(hButtonDelete, &ddRect);
		InvalidateRect(hButtonDelete, &ddRect, TRUE);
		MapWindowPoints(hButtonDelete, openGLWin.glWindowHandle, (POINT *)&ddRect, 2);
		RedrawWindow(hButtonDelete, &ddRect, NULL, RDW_ERASE | RDW_INVALIDATE);*/
		
		//InvalidateRect(hButtonDelete, &ddRect, TRUE);
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
		break;
	case WM_CLOSE:
		DestroyWindow(hWnd);
		openGLWin.ShutdownWindow();
		PostQuitMessage(0);
		break;
	case WM_DESTROY:
		break;
	case WM_ACTIVATE:
	{
		switch (LOWORD(wParam))
		{
		case WA_ACTIVE:
		case WA_CLICKACTIVE:
			openGLWin.ResetTimer();
			return DefWindowProc(hWnd, message, wParam, lParam);
			break;
		case WA_INACTIVE:
			break;
		}
		break;
	}
	case WM_NCACTIVATE:
			cDebug::DbgOut(L"WM_NCACTIVATE");
			break;
		// Handle button press
	case WM_COMMAND:
		cDebug::DbgOut(L"WM_COMMAND");
		GLProcessUI(wParam, lParam);
		break;
	case WM_DRAWITEM:
		return DrawButton(wParam, lParam);
		break;
	case WM_CTLCOLORSTATIC:
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
	case WM_NCLBUTTONDBLCLK:
	case WM_SIZE:
		if (openGLWin.GetWindowMode() == MODE_SCANNING || openGLWin.GetWindowMode() == MODE_PREPARE_SCANNING)
		{
			MoveModeButtonsOnResize();
			MoveWindow(openGLWin.fusionHandle, 0, 0, LOWORD(lParam), HIWORD(lParam), true);
		}
		else
		{
			openGLWin.glControl.ResizeOpenGLViewportFull(LOWORD(lParam), HIWORD(lParam));
			//openGLWin.glControl.SetProjection3D(45.0f, float(LOWORD(lParam)) / float(HIWORD(lParam)), 0.1f, 1000.0f);
			openGLWin.glControl.SetProjection3D(45.0f, float(openGLWin.glControl.GetViewportWidth()) / float(openGLWin.glControl.GetViewportHeight()), 0.1f, 1000.0f);
			openGLWin.glControl.SetOrtho2D(openGLWin.glControl.GetViewportWidth(), openGLWin.glControl.GetViewportHeight());
			MoveButtonsOnResize();
		}
		break;
	}
	return FALSE;
}

#pragma endregion Window Thread

#pragma region Debug Dialog

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
	case WM_SIZE:
		//cDebug::DbgOut(L"DebugDlgProc WM_SIZE");
		break;
	case WM_NOTIFY:
		break;
	}
	return FALSE;
}

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

	SendDlgItemMessage(debugHandle, IDC_SLIDER_REMOVESEGMENTS, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_REMOVESEGMENTS_VALUE, MAX_REMOVESEGMENTS_VALUE));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_CARRYDISTANCE, MAX_CARRYDISTANCE));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_MINCLUSTER, MAX_RG_MINCLUSTER));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_MAXCLUSTER, MAX_RG_MAXCLUSTER));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_NEIGHBORS, MAX_RG_NEIGHBORS));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_SMOOTHNESS, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_SMOOTHNESS, MAX_RG_SMOOTHNESS));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_CURVATURE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_CURVATURE, MAX_RG_CURVATURE));

	SendDlgItemMessage(debugHandle, IDC_SLIDER_FILLHOLES, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_FILLHOLES, MAX_FILLHOLES));

	ResetSliders();
}

void InteractiveFusion::SetBackgroundColor(int redValue, int greenValue, int blueValue)
{
	openGLWin.bgRed = redValue / 255.0f;
	openGLWin.bgGreen = greenValue / 255.0f;
	openGLWin.bgBlue = blueValue / 255.0f;
}

void ResetEditControls()
{
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_KSEARCHVALUE, openGLWin.kSearchValue, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_REMOVESEGMENTS, openGLWin.maxComponentSize, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_SELECTION_DISTANCE, openGLWin.carryDistance, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_FILLHOLES, openGLWin.holeSize * 100, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_MINCLUSTERSIZE, openGLWin.minClusterSize, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE, openGLWin.maxClusterSize * 1000, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_NON, openGLWin.numberOfNeighbors, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_SMOOTHNESS, (UINT)openGLWin.smoothnessThreshold, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_RG_CURVATURE, (UINT)openGLWin.curvatureThreshold, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_RED, 211, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_GREEN, 211, FALSE);
	SetDlgItemInt(debugHandle, IDC_EDIT_BACKGROUND_BLUE, 211, FALSE);
}

void ResetSliders()
{
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_KSEARCH_VALUE, TBM_SETPOS, TRUE, (UINT)openGLWin.kSearchValue);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_REMOVESEGMENTS, TBM_SETPOS, TRUE, (UINT)openGLWin.maxComponentSize);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE, TBM_SETPOS, TRUE, (UINT)openGLWin.carryDistance);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)openGLWin.minClusterSize);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)openGLWin.maxClusterSize);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_SETPOS, TRUE, (UINT)openGLWin.numberOfNeighbors);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_SMOOTHNESS, TBM_SETPOS, TRUE, (UINT)openGLWin.smoothnessThreshold);
	SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_CURVATURE, TBM_SETPOS, TRUE, (UINT)openGLWin.curvatureThreshold);
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
		SetDlgItemInt(debugHandle, IDC_EDIT_REMOVESEGMENTS, openGLWin.maxComponentSize, FALSE);
	}

	int carryDistancePos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE, TBM_GETPOS, 0, 0);

	if (carryDistancePos >= MIN_CARRYDISTANCE && carryDistancePos <= MAX_CARRYDISTANCE)
	{
		openGLWin.carryDistance = carryDistancePos;
		SetDlgItemInt(debugHandle, IDC_EDIT_SELECTION_DISTANCE, openGLWin.carryDistance, FALSE);
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
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE, openGLWin.maxClusterSize * 1000, FALSE);
	}

	int nonPos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_GETPOS, 0, 0);

	if (nonPos >= MIN_RG_NEIGHBORS && nonPos <= MAX_RG_NEIGHBORS)
	{
		openGLWin.numberOfNeighbors = nonPos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_NON, openGLWin.numberOfNeighbors, FALSE);
	}

	int smoothnessPos = (int)SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_SMOOTHNESS, TBM_GETPOS, 0, 0);

	if (smoothnessPos >= MIN_RG_SMOOTHNESS && smoothnessPos <= MAX_RG_SMOOTHNESS)
	{
		openGLWin.smoothnessThreshold = smoothnessPos;
		SetDlgItemInt(debugHandle, IDC_EDIT_RG_SMOOTHNESS, (UINT)openGLWin.smoothnessThreshold, FALSE);
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

void InteractiveFusion::ToggleDebugControls()
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
		MoveWindow(debugHandle, openGLWin.glControl.GetViewportWidth() - dRect.right, 0, dRect.right, openGLWin.glControl.GetViewportHeight(), false);
		ShowWindow(debugHandle, SW_SHOW);
	}
	RECT rRect;
	GetClientRect(GetParent(openGLWin.glWindowHandle), &rRect);
	openGLWin.glControl.ResizeOpenGLViewportFull();
	//openGLWin.glControl.SetProjection3D(45.0f, (float)rRect.right / (float)rRect.bottom, 0.1f, 1000.0f);
	//openGLWin.glControl.SetOrtho2D(rRect.right, rRect.bottom);
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
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_KSEARCH_VALUE, TBM_SETPOS, TRUE, (UINT)openGLWin.kSearchValue);
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
					SendDlgItemMessage(debugHandle, IDC_SLIDER_SELECTION_DISTANCE, TBM_SETPOS, TRUE, (UINT)openGLWin.carryDistance);
				}
			}
			else if (wnd == editMinClustersHandle)
			{
				int minClusterSize = GetDlgItemInt(debugHandle, IDC_EDIT_RG_MINCLUSTERSIZE, NULL, FALSE);
				if (minClusterSize >= MIN_RG_MINCLUSTER && minClusterSize <= MAX_RG_MINCLUSTER)
				{
					openGLWin.minClusterSize = minClusterSize;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MINCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)openGLWin.minClusterSize);
				}
			}
			else if (wnd == editMaxClustersHandle)
			{
				int maxClusterSize = GetDlgItemInt(debugHandle, IDC_EDIT_RG_MAXCLUSTERSIZE, NULL, FALSE) / 1000;
				if (maxClusterSize >= MIN_RG_MAXCLUSTER && maxClusterSize <= MAX_RG_MAXCLUSTER)
				{
					openGLWin.maxClusterSize = maxClusterSize;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)openGLWin.maxClusterSize);
				}
			}
			else if (wnd == editNonHandle)
			{
				int numberOfNeighbors = GetDlgItemInt(debugHandle, IDC_EDIT_RG_NON, NULL, FALSE);
				if (numberOfNeighbors >= MIN_RG_NEIGHBORS && numberOfNeighbors <= MAX_RG_NEIGHBORS)
				{
					openGLWin.numberOfNeighbors = numberOfNeighbors;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_NON, TBM_SETPOS, TRUE, (UINT)openGLWin.numberOfNeighbors);
				}
			}
			else if (wnd == editSmoothnessHandle)
			{
				int smoothnessThreshold = GetDlgItemInt(debugHandle, IDC_EDIT_RG_SMOOTHNESS, NULL, FALSE);
				if (smoothnessThreshold >= MIN_RG_SMOOTHNESS && smoothnessThreshold <= MAX_RG_SMOOTHNESS)
				{
					openGLWin.smoothnessThreshold = smoothnessThreshold;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_SMOOTHNESS, TBM_SETPOS, TRUE, (UINT)openGLWin.smoothnessThreshold);
				}
			}
			else if (wnd == editCurvatureHandle)
			{
				int curvatureThreshold = GetDlgItemInt(debugHandle, IDC_EDIT_RG_CURVATURE, NULL, FALSE);
				if (curvatureThreshold >= MIN_RG_CURVATURE && curvatureThreshold <= MAX_RG_CURVATURE)
				{
					openGLWin.curvatureThreshold = curvatureThreshold;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_RG_CURVATURE, TBM_SETPOS, TRUE, (UINT)openGLWin.curvatureThreshold);
				}
			}
			else if (wnd == editFillHoleHandle)
			{
				int holeSize = GetDlgItemInt(debugHandle, IDC_EDIT_FILLHOLES, NULL, FALSE) / 100;
				if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
				{
					openGLWin.holeSize = holeSize;
					SendDlgItemMessage(debugHandle, IDC_SLIDER_FILLHOLES, TBM_SETPOS, TRUE, (UINT)openGLWin.holeSize);
				}
			}
			break;
		}
	default:
		return CallWindowProc(oldEditProc, wnd, msg, wParam, lParam);
	}
	return 0;
}

#pragma endregion

#pragma region

void GLProcessUI(WPARAM wParam, LPARAM lParam)
{
	if (IDC_BUTTON_HELP_OK == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowBusyState() == IF_BUSYSTATE_BUSY)
		{
			openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
		}
	}
	if (IDC_BUTTON_MODE_PREPARE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowMode() != MODE_PREPARE_SCANNING)
		{
			if (openGLWin.GetWindowMode() != MODE_SCANNING)
				openGLWin.ResumeScanning();

			openGLWin.SetWindowMode(MODE_PREPARE_SCANNING);
		}
	}
	if (IDC_BUTTON_MODE_SCAN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowMode() == MODE_PREPARE_SCANNING)
			openGLWin.fusionExplorer->StartScan();
		else if (openGLWin.GetWindowMode() != MODE_SCANNING)
			openGLWin.ResumeScanning();
		
	}
	if (IDC_BUTTON_MODE_SEGMENTATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowMode() != MODE_SEGMENTATION)
		{
			openGLWin.SetWindowMode(MODE_SEGMENTATION);
		}
	}
	if (IDC_BUTTON_MODE_INTERACTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		SetFocus(hScanText);
		MoveModeButtonsOnResize();
		if (openGLWin.GetWindowMode() == MODE_SCANNING && openGLWin.fusionExplorer->GetWindowState() == SCAN)
		{
			openGLWin.fusionExplorer->FinishScan(0);
		}
	}
	if (IDC_BUTTON_SEGMENTATION_BEGIN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			openGLWin.previewMode = true;
			glSegmentation.StartSegmentation();
		}
	}
	if (IDC_BUTTON_SEGMENTATION_FINISH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			openGLWin.previewMode = false;
			glSegmentation.StartSegmentation();
		}
	}
	if (IDC_BUTTON_SEGMENTATION_EUCLIDEAN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			glSegmentation.SetSegmentationMode(SEGMENTATION_EUCLIDEAN);
			EnableWindow(hButtonRegionGrowthSegmentation, true);
			EnableWindow(hButtonEuclideanSegmentation, false);
			openGLWin.HideUI(regionGrowthUi);
			openGLWin.ShowUI(euclideanUi);
			MoveButtonsOnResize();
		}
	}
	if (IDC_BUTTON_SEGMENTATION_REGIONGROWTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			glSegmentation.SetSegmentationMode(SEGMENTATION_REGIONGROWTH);
			EnableWindow(hButtonRegionGrowthSegmentation, false);
			EnableWindow(hButtonEuclideanSegmentation, true);
			openGLWin.HideUI(euclideanUi);
			openGLWin.ShowUI(regionGrowthUi);
			MoveButtonsOnResize();
		}
	}
	if (IDC_BUTTON_RG_CURVATURE_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			float step = 0.0f;
			if (openGLWin.curvatureThreshold > 10)
				step = 1;
			else if (openGLWin.curvatureThreshold > 3)
				step = 0.5f;
			else
				step = 0.1f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.curvatureThreshold += step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_CURVATURE_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			if (openGLWin.curvatureThreshold <= 0.1f)
				return;
			float step = 0.0f;
			if (openGLWin.curvatureThreshold > 10)
				step = 1;
			else if (openGLWin.curvatureThreshold > 3)
				step = 0.5f;
			else
				step = 0.1f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.curvatureThreshold -= step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_NEIGHBORS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			float step = 1.0f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.numberOfNeighbors += step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_NEIGHBORS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			if (openGLWin.numberOfNeighbors <= 1)
				return;
			float step = 1.0f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.numberOfNeighbors -= step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_KSEARCH_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			float step = 1.0f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.kSearchValue += step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_KSEARCH_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			if (openGLWin.kSearchValue <= 1)
				return;
			float step = 1.0f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.kSearchValue -= step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_SMOOTHNESS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			float step = 0.0f;
			if (openGLWin.smoothnessThreshold > 100)
				step = 10;
			else if (openGLWin.smoothnessThreshold > 20)
				step = 5;
			else
				step = 1;

			openGLWin.segmentValuesChanged = true;
			openGLWin.smoothnessThreshold += step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_RG_SMOOTHNESS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			if (openGLWin.smoothnessThreshold <= 1)
				return;
			float step = 0.0f;
			if (openGLWin.smoothnessThreshold > 100)
				step = 10;
			else if (openGLWin.smoothnessThreshold > 20)
				step = 5;
			else
				step = 1;

			openGLWin.segmentValuesChanged = true;
			openGLWin.smoothnessThreshold -= step;
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_CLUSTERTOLERANCE_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			float step = 0.0f;
			if (openGLWin.clusterTolerance >= 0.05f)
				step = 0.01f;
			else if (openGLWin.clusterTolerance >= 0.01f)
				step = 0.005f;
			else
				step = 0.001f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.clusterTolerance += step;
			cDebug::DbgOut(L"PLUS: ", openGLWin.clusterTolerance);
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_CLUSTERTOLERANCE_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowState() == SEGMENTATION_PREVIEW)
		{
			if (openGLWin.clusterTolerance <= 0.001f)
				return;
			float step = 0.0f;
			if (openGLWin.clusterTolerance > 0.05f)
				step = 0.01f;
			else if (openGLWin.clusterTolerance > 0.01f)
				step = 0.005f;
			else
				step = 0.001f;

			openGLWin.segmentValuesChanged = true;
			openGLWin.clusterTolerance -= step;
			cDebug::DbgOut(L"MINUS: ", openGLWin.clusterTolerance);
			openGLWin.UpdateSegmentationPreviewValues();
		}
	}
	if (IDC_BUTTON_WALLSIZE_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		float step = 0.0f;
		if (openGLWin.wallThickness >= 0.30f)
			step = 0.1f;
		else if (openGLWin.wallThickness >= 0.10f)
			step = 0.05f;
		else
			step = 0.01f;
		openGLWin.wallThickness += step;
		cDebug::DbgOut(L"PLUS: ", openGLWin.wallThickness);
		//meshHelper.RemoveAllHighlights();

		openGLWin.UpdateWallSelectionValues();
	}
	if (IDC_BUTTON_WALLSIZE_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.wallThickness <= 0.01f)
			return;
		float step = 0.0f;
		if (openGLWin.wallThickness > 0.30f)
			step = 0.1f;
		else if (openGLWin.wallThickness > 0.10f)
			step = 0.05f;
		else
			step = 0.01f;

		openGLWin.wallThickness -= step;
		cDebug::DbgOut(L"MINUS: ", openGLWin.wallThickness);
		//meshHelper.RemoveAllHighlights();
		
		openGLWin.UpdateWallSelectionValues();
	}
	if (IDC_BUTTON_WALLSMOOTHNESS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.wallSmoothness >= 1.00f)
			return;
		float step = 0.0f;
		if (openGLWin.wallSmoothness >= 0.30f)
			step = 0.1f;
		else if (openGLWin.wallSmoothness >= 0.10f)
			step = 0.05f;
		else
			step = 0.01f;
		openGLWin.wallSmoothness += step;
		cDebug::DbgOut(L"PLUS: ", openGLWin.wallSmoothness);
		//meshHelper.RemoveAllHighlights();
		
		openGLWin.UpdateWallSelectionValues();
	}
	if (IDC_BUTTON_WALLSMOOTHNESS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.wallSmoothness <= 0.00f)
			return;
		float step = 0.0f;
		if (openGLWin.wallSmoothness > 0.30f)
			step = 0.1f;
		else if (openGLWin.wallSmoothness > 0.10f)
			step = 0.05f;
		else
			step = 0.01f;
		openGLWin.wallSmoothness -= step;
		cDebug::DbgOut(L"MINUS: ", openGLWin.wallSmoothness);
		//meshHelper.RemoveAllHighlights();
		
		openGLWin.UpdateWallSelectionValues();
	}
	if (IDC_BUTTON_RESET == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//openGLWin.ResumeScanning();
		meshHelper.ResetAll();
		glSelector.Unselect();
	}
	if (IDC_BUTTON_DUPLICATE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.duplicationMode = !openGLWin.duplicationMode;
	}
	if (IDC_BUTTON_SCALE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.RedrawManipulationButtons();
		glSelector.SetManipulationMode(MANIPULATION_SCALE);
	}
	if (IDC_BUTTON_ROTATE_HORIZONTAL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.RedrawManipulationButtons();
		glSelector.SetManipulationMode(MANIPULATION_ROTATE_Y);
	}
	if (IDC_BUTTON_ROTATE_VERTICAL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.RedrawManipulationButtons();
		glSelector.SetManipulationMode(MANIPULATION_ROTATE_X);
	}
	if (IDC_BUTTON_YES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//openGLWin.glControl.SetOffSetBottom(0);
		openGLWin.SetAnswer(ANSWER_YES);
		//openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
		openGLWin.isWall = true;
		openGLWin.wallThickness = 0.2f;
		openGLWin.wallSmoothness = 0.5f;
		meshHelper.RemoveAllHighlights();
		ShowWindow(hTextWalls, SW_HIDE);
	}
	if (IDC_BUTTON_NO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		//openGLWin.glControl.SetOffSetBottom(0);
		openGLWin.SetAnswer(ANSWER_NO);
		//openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
		openGLWin.isWall = false;
		openGLWin.wallThickness = 0.2f;
		openGLWin.wallSmoothness = 0.5f;
		meshHelper.RemoveAllHighlights();
		ShowWindow(hTextWalls, SW_HIDE);
		//openGLWin.glControl.ResizeOpenGLViewportFull();
	}
	if (IDC_BUTTON_EXPORT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.ShowStatusBarMessage(L"Combining and exporting all meshes...");
		meshHelper.CombineAndExport();
	}
	if (IDC_BUTTON_FILLHOLES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.ShowStatusBarMessage(L"Filling holes...");
		meshHelper.FillHoles(openGLWin.holeSize);
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
		openGLWin.ShowStatusBarMessage(L"Region growth values back to default.");
	}
	
	if (IDC_CHECK_WIREFRAME == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		openGLWin.wireFrameMode = !openGLWin.wireFrameMode;
	}
	if (IDC_CHECK_FREECAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		
		ToggleCameraMode();
	}
	if (IDC_CHECK_COLORSELECTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		openGLWin.colorSelection = !openGLWin.colorSelection;
		if (!openGLWin.colorSelection)
			RemoveSelectionColor();
	}
	if (IDC_CHECK_PLACING_RAYCAST == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.placeWithRaycast = !openGLWin.placeWithRaycast;
	}
	if (IDC_CHECK_PLACING_SNAPTOVERTEX == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		openGLWin.snapToVertex = !openGLWin.snapToVertex;
	}
	if (IDC_CHECK_SHOWBB == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		// Toggle our internal state for near mode
		openGLWin.showBB = !openGLWin.showBB;
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
		meshHelper.RemoveSmallComponents(size);
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
			openGLWin.SetBackgroundColor(redValue, greenValue, blueValue);
			openGLWin.ShowStatusBarMessage(L"Changed background color.");
		}
		//int redValue = (int)itemBuff;

		//m_processor.ResetReconstruction();
	}
	if (IDC_BUTTON_REGION_GROWTH_SEGMENTATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
		openGLWin.previewMode = false;
		openGLWin.ShowStatusBarMessage(L"Starting region growth segmentation...");
		SetFocus(openGLWin.glWindowHandle);
		StartSegmentation();
		//m_processor.ResetReconstruction();
	}

	if (IDC_BUTTON_CLEANMESH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		if (openGLWin.GetWindowBusyState() == IF_BUSYSTATE_DEFAULT)
			openGLWin.SetWindowBusyState(IF_BUSYSTATE_BUSY);
		else 
		openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
		/*cDebug::DbgOut(L"clean mesh", 0);
		openGLWin.ShowStatusBarMessage(L"Cleaning mesh...");
		meshHelper.CleanMesh();*/
		
	}
	if (IDC_BUTTON_RG_PREVIEW == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		openGLWin.ShowStatusBarMessage(L"Preparing region growth segmentation preview...");
		openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
		openGLWin.previewMode = true;
		StartSegmentation();
		//m_processor.ResetReconstruction();
	}
	if (IDC_BUTTON_RESETCAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
	{
		ResetCameraPosition();
		openGLWin.ShowStatusBarMessage(L"Camera position reset.");
	}
}

void InteractiveFusion::UpdateWallSelectionValues()
{
	int thicknessLabel = openGLWin.wallThickness * 100;
	wstring thicknessString = to_wstring(thicknessLabel) + L"cm";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_WALLSIZE, thicknessString.c_str());

	int smoothnessLabel = openGLWin.wallSmoothness * 100;
	wstring smoothnessString = to_wstring(smoothnessLabel) + L"%";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_WALLSMOOTHNESS, smoothnessString.c_str());
}

void InteractiveFusion::UpdateSegmentationPreviewValues()
{
	int clusterToleranceLabel = openGLWin.clusterTolerance * 1000;
	wstring clusterToleranceString = to_wstring(clusterToleranceLabel) + L"cm";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_CLUSTERTOLERANCE, clusterToleranceString.c_str());

	int smoothnessLabel = openGLWin.smoothnessThreshold;
	wstring smoothnessString = to_wstring(smoothnessLabel) + L"";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_RG_SMOOTHNESS, smoothnessString.c_str());

	int curvatureLabel = openGLWin.curvatureThreshold;
	wstring curvatureString = to_wstring(curvatureLabel) + L"";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_RG_CURVATURE, curvatureString.c_str());

	int neighborsLabel = openGLWin.numberOfNeighbors;
	wstring neighborsString = to_wstring(neighborsLabel) + L"";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_RG_NEIGHBORS, neighborsString.c_str());

	int kSearchLabel = openGLWin.kSearchValue;
	wstring kSearchString = to_wstring(kSearchLabel) + L"";
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_RG_KSEARCH, kSearchString.c_str());
}

void MoveModeButtonsOnResize()
{
	RECT rect; GetWindowRect(openGLWin.parent, &rect);
	//MoveWindow(hModeArrow, 113, 0, 50, 50, true);
	MoveWindow(hPrepareText, rect.right / 2 - 390, 8, 150, 40, true);
	MoveWindow(hScanText, rect.right/2 - 240, 8, 130, 40, true);
	MoveWindow(hSegmentationText, rect.right / 2 - 110, 8, 250, 40, true);
	MoveWindow(hInteractionText, rect.right/2 + 140 , 8, 250, 40, true);
	//MoveWindow(hScanText, 15, 8, 125, 40, true);
	//MoveWindow(hInteractionText, 140, 8, 250, 40, true);
}

void MoveButtonsOnResize()
{
	int width = openGLWin.glControl.GetViewportWidth() + openGLWin.glControl.GetOffSetRight();
	int height = openGLWin.glControl.GetViewportHeight() + openGLWin.glControl.GetOffSetBottom();

	

	if (GetWindowTextLength(statusPercentText) > 0)
	{ 
		MoveWindow(statusText, 0, height / 2 - 75, width, 100, true);
		MoveWindow(statusPercentText, 0, height/2 + 25, width, 50, true);
		
	}
	else
	{ 
		MoveWindow(statusText, 0, height / 2 - 75, width, 150, true);
		
	}

	if (openGLWin.GetWindowMode() == MODE_SEGMENTATION)
	{
		if (openGLWin.GetWindowState() == WALL_SELECTION)
		{
			MoveWindow(hButtonYes, width - 200, height - 200, 150, 150, true);
			MoveWindow(hButtonNo, 50, height - 200, 150, 150, true);

		//	MoveWindow(hTextWalls, 0, 48, width, 40, true);
			MoveWindow(hTextWalls, 0, 48, 0, 0, true);

			MoveWindow(hHelpText1, 0, height - 240, width, 40, true);
			MoveWindow(hTextWallSizeLabel, width / 2 - 200, height - 195, 150, 50, true);
			MoveWindow(hButtonWallSizeMinus, width / 2 - 25, height - 200, 50, 50, true);
			MoveWindow(hTextWallSize, width / 2 + 50, height - 195, 75, 50, true);
			MoveWindow(hButtonWallSizePlus, width / 2 + 150, height - 200, 50, 50, true);


			MoveWindow(hHelpText2, 0, height - 135, width, 40, true);
			MoveWindow(hTextWallSmoothnessLabel, width / 2 - 225, height - 80, 175, 50, true);
			MoveWindow(hButtonWallSmoothnessMinus, width / 2 - 25, height - 85, 50, 50, true);
			MoveWindow(hTextWallSmoothness, width / 2 + 50, height - 80, 75, 50, true);
			MoveWindow(hButtonWallSmoothnessPlus, width / 2 + 150, height - 85, 50, 50, true);
		}

		MoveWindow(hButtonSegmentationFinish, width - 200, height - 200, 150, 150, true);
		MoveWindow(hButtonSegmentationBegin, width - 200, height - 400, 150, 150, true);

		MoveWindow(hButtonEuclideanSegmentation, width - 300, 200, 150, 50, true);
		MoveWindow(hButtonRegionGrowthSegmentation, width - 150, 200, 150, 50, true);

		if (glSegmentation.GetSegmentationMode() == SEGMENTATION_EUCLIDEAN)
		{ 
			MoveWindow(hTextClusterToleranceLabel, width - 180, 290, 100, 30, true);
			MoveWindow(hButtonClusterToleranceMinus, width - 250, 300, 50, 50, true);
			MoveWindow(hTextClusterTolerance, width - 180, 320, 100, 30, true);
			MoveWindow(hButtonClusterTolerancePlus, width -60, 300, 50, 50, true);
		}
		else if (glSegmentation.GetSegmentationMode() == SEGMENTATION_REGIONGROWTH)
		{
			MoveWindow(hTextRGSmoothnessLabel, width - 180, 290, 100, 30, true);
			MoveWindow(hButtonRGSmoothnessMinus, width - 250, 300, 50, 50, true);
			MoveWindow(hTextRGSmoothness, width - 180, 320, 100, 30, true);
			MoveWindow(hButtonRGSmoothnessPlus, width - 60, 300, 50, 50, true);

			MoveWindow(hTextRGCurvatureLabel, width - 180, 360, 100, 30, true);
			MoveWindow(hButtonRGCurvatureMinus, width - 250, 370, 50, 50, true);
			MoveWindow(hTextRGCurvature, width - 180, 390, 100, 30, true);
			MoveWindow(hButtonRGCurvaturePlus, width - 60, 370, 50, 50, true);

			MoveWindow(hTextRGNeighborsLabel, width - 180, 430, 100, 30, true);
			MoveWindow(hButtonRGNeighborsMinus, width - 250, 440, 50, 50, true);
			MoveWindow(hTextRGNeighbors, width - 180, 460, 100, 30, true);
			MoveWindow(hButtonRGNeighborsPlus, width - 60, 440, 50, 50, true);

			MoveWindow(hTextRGKSearchLabel, width - 180, 500, 100, 30, true);
			MoveWindow(hButtonRGKSearchMinus, width - 250, 510, 50, 50, true);
			MoveWindow(hTextRGKSearch, width - 180, 530, 100, 30, true);
			MoveWindow(hButtonRGKSearchPlus, width - 60, 510, 50, 50, true);
		}
	}
	if (openGLWin.GetWindowMode() == MODE_INTERACTION)
	{
		if (openGLWin.GetWindowState() == DEFAULT)
		{
			MoveWindow(hButtonExport, width - 200, height-200, 150, 150, true);
			
			MoveWindow(hButtonDuplicate, width - 200, 300, 150, 150, true);
			MoveWindow(hButtonReset, width - 200, 100, 150, 150, true);

			MoveWindow(hButtonScale, width - 250, 500, 75, 75, true);
			MoveWindow(hButtonRotateHorizontal, width - 165, 500, 75, 75, true);
			MoveWindow(hButtonRotateVertical, width - 80, 500, 75, 75, true);
		}
	}

	if (openGLWin.GetWindowBusyState() == IF_BUSYSTATE_BUSY)
	{
		if (openGLWin.glControl.GetViewportHeight() != 0)
		{ 
			MoveWindow(hButtonHelp, openGLWin.glControl.GetViewportWidth() / 2 - 75, (openGLWin.glControl.GetViewportHeight() / 2), 150, 50, true);
		}
	}
	MoveModeButtonsOnResize();

	RECT sRect; GetWindowRect(statusHandle, &sRect);
	MoveWindow(statusHandle, 0, height - 30, width, 30, true);

	if (IsWindowVisible(debugHandle))
	{
		RECT rRect;
		GetClientRect(debugHandle, &rRect);
		MoveWindow(debugHandle, width - rRect.right, 0, rRect.right, rRect.bottom, true);
	}

	/*RECT ddRect;
	GetClientRect(hButtonDelete, &ddRect);
	InvalidateRect(hButtonDelete, &ddRect, TRUE);*/
}

bool InteractiveFusion::IsMouseInHandle()
{
	POINT pCur;
	GetCursorPos(&pCur);

	if (IsMouseInUI(wallSelectionUi))
		return true;
	if (IsMouseInUI(segmentationPreviewUi))
		return true;
	if (IsMouseInUI(interactionUi))
		return true;

	return false;
}

bool InteractiveFusion::IsMouseInUI(std::vector<HWND> handles)
{
	POINT pCur;
	GetCursorPos(&pCur);
	for (int i = 0; i < handles.size(); i++)
	{
		if (IsWindowVisible(handles[i]) && handles[i])
		{
			RECT rRect; GetWindowRect(handles[i], &rRect);
			if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
				pCur.y >= rRect.top && pCur.y <= rRect.bottom)
				return true;
		}
	}
	return false;
}

void InteractiveFusion::RedrawManipulationButtons()
{
	RECT rect;
	GetClientRect(hButtonScale, &rect);

	InvalidateRect(hButtonScale, &rect, TRUE);

	GetClientRect(hButtonRotateHorizontal, &rect);

	InvalidateRect(hButtonRotateHorizontal, &rect, TRUE);

	GetClientRect(hButtonRotateVertical, &rect);

	InvalidateRect(hButtonRotateVertical, &rect, TRUE);
	//MapWindowPoints(hTextWalls, openGLWin.glWindowHandle, (POINT *)&rect, 2);
	//RedrawWindow(hTextWalls, &rect, NULL, RDW_ERASE | RDW_INVALIDATE);
}

bool InteractiveFusion::IsMouseInOpenGLWindow()
{
	POINT pCur;
	GetCursorPos(&pCur);
	RECT rRect; GetWindowRect(openGLWin.glWindowHandle, &rRect);

	if (pCur.x >= rRect.left && pCur.x <= rRect.right-glControl.GetOffSetRight() &&
		pCur.y >= rRect.top && pCur.y <= rRect.bottom - glControl.GetOffSetBottom() &&
		GetActiveWindow() == openGLWin.glWindowParent && !IsMouseInHandle())
		return true;
	else
		return false;
}

void InteractiveFusion::ShowStatusBarMessage(string message)
{
	std::wstring ws = Util::StringToWString(message);
	LPCWSTR statusBarMessage = ws.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, statusBarMessage);
}

void InteractiveFusion::ShowStatusBarMessage(wstring message)
{
	LPCWSTR statusBarMessage = message.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, statusBarMessage);
}

#pragma endregion GUI

#pragma region

void InteractiveFusion::ResetTimer()
{
	tLastFrame = clock();
	fFrameInterval = 0.0f;
}

void InteractiveFusion::UpdateTimer()
{
	clock_t tCur = clock();
	fFrameInterval = float(tCur - tLastFrame) / float(CLOCKS_PER_SEC);
	tLastFrame = tCur;
}

float InteractiveFusion::SpeedOptimizedFloat(float fVal)
{
	return fVal*fFrameInterval;
}

void InteractiveFusion::SetProgressionText(wstring text)
{
	LPCWSTR statusMessage = text.c_str();
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_STATUS, statusMessage);
}
void InteractiveFusion::SetProgressionPercent(wstring percent)
{
	LPCWSTR statusMessage = percent.c_str();
	SetDlgItemText(openGLWin.glWindowHandle, IDC_STATIC_TEXT_STATUS_PERCENT, statusMessage);
}

void InteractiveFusion::DetermineMeshQuality()
{
	int vpm = fusionExplorer->GetVoxelsPerMeter();
	if (vpm > 192)
		meshQuality = QUALITY_VERYHIGH;
	else if (vpm > 150)
		meshQuality = QUALITY_HIGH;
	else if (vpm > 110)
		meshQuality = QUALITY_MEDIUM;
	else if (vpm > 64)
		meshQuality = QUALITY_LOW;
	else
		meshQuality = QUALITY_VERYLOW;
}

#pragma endregion FPS stuff