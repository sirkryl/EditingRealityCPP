#include "PlaneSelectionWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"
#include "SegmentationParams.h"

namespace InteractiveFusion {

	PlaneSegmentationParams planeParams;

	GUIContainer  wallUi;
	HWND buttonSizePlus, buttonSizeMinus;
	HWND buttonSmoothnessPlus, buttonSmoothnessMinus;
	HWND textSizeDescription, textSize;
	HWND textSmoothnessDescription, textSmoothness;
	HWND textWalls;
	HWND helpText1, helpText2;
	HWND buttonYes, buttonNo;

	HWND propertiesWindow;
	HWND propertiesButton;

	HWND yesWindow;
	HWND noWindow;
	HBRUSH propertyBackground;
	PlaneSelectionWindow::PlaneSelectionWindow()
	{
		DebugUtility::DbgOut(L"PlaneSelectionWindow::PlaneSelectionWindow()");
	}


	PlaneSelectionWindow::~PlaneSelectionWindow()
	{
	}

	void PlaneSelectionWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		SetWindowLong(windowHandle, GWL_EXSTYLE, GetWindowLong(windowHandle, GWL_EXSTYLE) | WS_EX_TRANSPARENT);

		propertyBackground = CreateSolidBrush(RGB(StyleSheet::GetInstance()->GetPropertyBackgroundColor().r, StyleSheet::GetInstance()->GetPropertyBackgroundColor().g, StyleSheet::GetInstance()->GetPropertyBackgroundColor().b));
		std::wstring propertyClassName = className + L"_sub";

		WNDCLASSEX wc = { 0 };
		wc.cbSize = sizeof(WNDCLASSEX);
		wc.lpfnWndProc = (WNDPROC)PlaneSelectionWindow::MessageRouter;
		wc.style = 0;
		wc.hInstance = hInstance;
		wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = propertyBackground;
		wc.lpszMenuName = NULL;
		wc.lpszClassName = (LPCWSTR)propertyClassName.c_str();
		//wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
		ATOM propertyClass = RegisterClassExW(&wc);


		yesWindow = CreateWindowExW(0, (LPCTSTR)MAKELONG(propertyClass, 0), 0, WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN,
			0, 0, 0, 0, parentHandle,
			NULL, hInstance, this);
		SetWindowPos(yesWindow, HWND_TOP, 0, 0, 0, 0, 0);

		noWindow = CreateWindowExW(0, (LPCTSTR)MAKELONG(propertyClass, 0), 0, WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN,
			0, 0, 0, 0, parentHandle,
			NULL, hInstance, this);
		SetWindowPos(noWindow, yesWindow, 0, 0, 0, 0, 0);

		propertiesWindow = CreateWindowExW(0, (LPCTSTR)MAKELONG(propertyClass, 0), 0, WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN,
			0, 0, 0, 0, parentHandle,
			NULL, hInstance, this);
		SetWindowPos(propertiesWindow, noWindow, 0, 0, 0, 0, 0);

		Hide();

		planeParams.planeThickness = 0.1f;
		planeParams.planeSmoothness = 0.05f;

		/*propertiesWindow = CreateWindowExW(0, L"Static", 0, WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_BORDER,
			0, 0, 0, 0, windowHandle,
			NULL, hInstance, this);*/

		propertiesButton = CreateWindowEx(0, L"Button", L"+", WS_CHILD | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANE_BUTTON_PROPERTIES, hInstance, 0);

		buttonSizePlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_BUTTON_WALLSIZE_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonSizePlus, ButtonLayout());
		buttonLayoutMap[buttonSizePlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonSizePlus].SetFontSize(60);
		buttonSizeMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_BUTTON_WALLSIZE_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonSizeMinus, ButtonLayout());
		buttonLayoutMap[buttonSizeMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonSizeMinus].SetFontSize(60);
		buttonSmoothnessPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW , 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_BUTTON_WALLSMOOTHNESS_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonSmoothnessPlus, ButtonLayout());
		buttonLayoutMap[buttonSmoothnessPlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonSmoothnessPlus].SetFontSize(60);
		buttonSmoothnessMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_BUTTON_WALLSMOOTHNESS_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonSmoothnessMinus, ButtonLayout());
		buttonLayoutMap[buttonSmoothnessMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonSmoothnessMinus].SetFontSize(60);

		textWalls = CreateWindowEx(0, L"STATIC", L"Is the highlighted area (part of) a wall, floor or ceiling?", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANE_TEXT_ISWALL, hInstance, 0);
		SendMessage(textWalls, WM_SETFONT, (WPARAM)uiFontBig, TRUE);


		textSize = CreateWindowEx(0, L"STATIC", L"2cm", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_TEXT_WALLSIZE, hInstance, 0);

		//SetDlgItemText(glWindowParent, IDC_PLANE_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
		SendMessage(textSize, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textSizeDescription = CreateWindowEx(0, L"STATIC", L"Thickness", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_TEXT_WALLSIZE_LABEL, hInstance, 0);
		//SetDlgItemText(glWindowParent, IDC_PLANE_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
		SendMessage(textSizeDescription, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textSmoothness = CreateWindowEx(0, L"STATIC", L"2cm", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_TEXT_WALLSMOOTHNESS, hInstance, 0);
		//SetDlgItemText(glWindowParent, IDC_PLANE_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
		SendMessage(textSmoothness, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textSmoothnessDescription = CreateWindowEx(0, L"STATIC", L"Smoothness", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_TEXT_WALLSMOOTHNESS_LABEL, hInstance, 0);
		//SetDlgItemText(glWindowParent, IDC_PLANE_TEXT_WALLSIZE, L"Is this (part of) a floor/wall?");
		SendMessage(textSmoothnessDescription, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		
		UpdatePlaneSelectionValues();
		

		helpText1 = CreateWindowEx(0, L"STATIC", L"If the highlighted area is too thick or too thin, change the wall thickness.", WS_CHILD | WS_CLIPSIBLINGS | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_TEXT_WALLSIZE_HELP, hInstance, 0);
		SendMessage(helpText1, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);

		helpText2 = CreateWindowEx(0, L"STATIC", L"If the highlighted area has gaps, decrease smoothness. If it is not flat enough (contains other objects), increase it.", WS_CHILD | SS_CENTER | WS_CLIPSIBLINGS | SS_CENTERIMAGE, 250, 50, 150, 50, propertiesWindow, (HMENU)IDC_PLANE_TEXT_WALLSMOOTHNESS_HELP, hInstance, 0);
		SendMessage(helpText2, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);

		buttonYes = CreateWindowEx(0, L"Button", L"YES", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, yesWindow, (HMENU)IDC_PLANE_BUTTON_YES, hInstance, 0);

		buttonLayoutMap[buttonYes] = ButtonLayout();
		buttonLayoutMap[buttonYes].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Green));
		buttonNo = CreateWindowEx(0, L"Button", L"NO", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, noWindow, (HMENU)IDC_PLANE_BUTTON_NO, hInstance, 0);

		buttonLayoutMap[buttonNo] = ButtonLayout();
		buttonLayoutMap[buttonNo].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Red));

		wallUi.Add(textWalls);
		wallUi.Add(textSize);
		wallUi.Add(textSizeDescription);
		wallUi.Add(textSmoothness);
		wallUi.Add(textSmoothnessDescription);
		//wallUi.Add(helpText1);
		//wallUi.Add(helpText2);
		wallUi.Add(buttonYes);
		wallUi.Add(buttonNo);
		wallUi.Add(buttonSizeMinus);
		wallUi.Add(buttonSizePlus);
		wallUi.Add(buttonSmoothnessPlus);
		wallUi.Add(buttonSmoothnessMinus);

	}




	LRESULT CALLBACK PlaneSelectionWindow::SubWindowProc(HWND _windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		
		switch (message)
		{
		case WM_LBUTTONDOWN:
			if (_windowHandle == windowHandle)
				DebugUtility::DbgOut(L"down it is");
			if (_windowHandle == propertiesWindow)
				DebugUtility::DbgOut(L"PROPERTIES DOWN");
			break;
		case WM_CTLCOLORSTATIC:
		{
			HDC hdc = reinterpret_cast<HDC>(wParam);
			SetBkMode((HDC)wParam, TRANSPARENT);
			SetTextColor(hdc, RGB(StyleSheet::GetInstance()->GetDefaultTextColor().r, StyleSheet::GetInstance()->GetDefaultTextColor().g, StyleSheet::GetInstance()->GetDefaultTextColor().b));

			return (LRESULT)propertyBackground;
		}
		break;
		}
		return SubWindow::SubWindowProc(_windowHandle, message, wParam, lParam);
	}

	void PlaneSelectionWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_PLANE_BUTTON_YES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneSelectionWindow::ProcessUI::YES");
			
			//parentWindow->ChangeState(Segmentation);
			
			planeParams.planeThickness = 0.1f;
			planeParams.planeSmoothness = 0.05f;

			ShowWindow(textWalls, SW_HIDE);
			eventQueue.push(PlaneSelectionWindowEvent::PlaneConfirmed);
		}
		if (IDC_PLANE_BUTTON_NO == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneSelectionWindow::ProcessUI::NO");
			
			planeParams.planeThickness = 0.1f;
			planeParams.planeSmoothness = 0.05f;

			ShowWindow(textWalls, SW_HIDE);
			eventQueue.push(PlaneSelectionWindowEvent::PlaneRejected);
		}
		if (IDC_PLANE_BUTTON_WALLSIZE_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			float step = 0.0f;
			if (planeParams.planeThickness >= 0.30f)
				step = 0.1f;
			else if (planeParams.planeThickness >= 0.10f)
				step = 0.05f;
			else
				step = 0.01f;
			planeParams.planeThickness += step;
			DebugUtility::DbgOut(L"PLUS: ", planeParams.planeThickness);
			//meshHelper.RemoveAllHighlights();

			UpdatePlaneSelectionValues();
			eventQueue.push(PlaneSelectionWindowEvent::UpdateSegmentation);
		}
		if (IDC_PLANE_BUTTON_WALLSIZE_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (planeParams.planeThickness <= 0.01f)
				return;
			float step = 0.0f;
			if (planeParams.planeThickness > 0.30f)
				step = 0.1f;
			else if (planeParams.planeThickness > 0.10f)
				step = 0.05f;
			else
				step = 0.01f;

			planeParams.planeThickness -= step;
			DebugUtility::DbgOut(L"MINUS: ", planeParams.planeThickness);
			//meshHelper.RemoveAllHighlights();

			UpdatePlaneSelectionValues();
			eventQueue.push(PlaneSelectionWindowEvent::UpdateSegmentation);
		}
		if (IDC_PLANE_BUTTON_WALLSMOOTHNESS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{

			DebugUtility::DbgOut(L"BEGINNING: ", planeParams.planeSmoothness);

			if (planeParams.planeSmoothness >= 1.00f)
				return;
			float step = 0.0f;
			if (planeParams.planeSmoothness >= 0.30f)
				step = 0.1f;
			else if (planeParams.planeSmoothness >= 0.10f)
				step = 0.05f;
			else
				step = 0.01f;
			planeParams.planeSmoothness += step;
			DebugUtility::DbgOut(L"PLUS: ", planeParams.planeSmoothness);
			//meshHelper.RemoveAllHighlights();
			UpdatePlaneSelectionValues();
			DebugUtility::DbgOut(L"UPDATED: ", planeParams.planeSmoothness);
			eventQueue.push(PlaneSelectionWindowEvent::UpdateSegmentation);
		}
		if (IDC_PLANE_BUTTON_WALLSMOOTHNESS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (planeParams.planeSmoothness <= 0.00f)
				return;
			float step = 0.0f;
			if (planeParams.planeSmoothness > 0.30f)
				step = 0.1f;
			else if (planeParams.planeSmoothness > 0.10f)
				step = 0.05f;
			else
				step = 0.01f;
			planeParams.planeSmoothness -= step;
			DebugUtility::DbgOut(L"MINUS: ", planeParams.planeSmoothness);
			//meshHelper.RemoveAllHighlights();

			UpdatePlaneSelectionValues();
			eventQueue.push(PlaneSelectionWindowEvent::UpdateSegmentation);
		}
		if (IDC_PLANE_BUTTON_PROPERTIES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (IsWindowVisible(propertiesWindow))
			{
				ShowWindow(propertiesWindow, false);
			}
			else
				ShowWindow(propertiesWindow, true);
		}
	}

	void PlaneSelectionWindow::HandleEvents(MainWindow* _parentWindow)
	{
		while (!eventQueue.empty()) 
		{
			int event = eventQueue.front();

			switch (event)
			{
			case PlaneSelectionWindowEvent::StateChange:
				_parentWindow->ChangeState(Segmentation);
				_parentWindow->SetAndShowHelpMessage(HelpMessage::SegmentationHelp);
				_parentWindow->UpdateObjectSegmentation(new EuclideanSegmentationParams());
				break;
			case PlaneSelectionWindowEvent::PlaneConfirmed:
				_parentWindow->ConfirmSegmentedPlane(&planeParams);
				break;
			case PlaneSelectionWindowEvent::PlaneRejected:
				_parentWindow->RejectSegmentedPlane(&planeParams);
				break;
			case PlaneSelectionWindowEvent::UpdateSegmentation:
				_parentWindow->UpdatePlaneSegmentation(&planeParams);
				break;
			}

			eventQueue.pop();
		}
	}

	void PlaneSelectionWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"PrepareWindow::RESIZE");
		//RECT rRect;
		//GetClientRect(parentHandle, &rRect);

		SubWindow::Resize(parentWidth, parentHeight);

		int noX = (int)(0.0f);
		
		int noWidth = (int)(0.125f*parentWidth);
		int noHeight = (int)(0.125f*parentWidth);
		int noY = (int)(parentHeight/2 - noHeight/2) - 28;
		MoveWindow(noWindow, noX, noY, noWidth, noHeight, true);

		MoveWindow(buttonNo, 0, 0, noWidth, noWidth, true);
		//MoveWindow(buttonNo, (int)(0.025*noWidth), (int)(0.025*noHeight), (int)(0.95*noWidth), (int)(0.95*noWidth), true);

		int yesX = (int)(parentWidth - 0.125f*parentWidth);
		
		int yesWidth = (int)(0.125f*parentWidth);
		int yesHeight = (int)(0.125f*parentWidth);
		int yesY = (int)(parentHeight/2 - yesHeight/2) - 28;
		
		MoveWindow(yesWindow, yesX, yesY, yesWidth, yesHeight, true);
		MoveWindow(buttonYes, 0, 0, yesWidth, yesWidth, true);
		//MoveWindow(buttonYes, (int)(0.025*yesWidth), (int)(0.025*yesHeight), (int)(0.95*yesWidth), (int)(0.95*yesWidth), true);
		

		//	MoveWindow(hTextWalls, 0, 48, width, 40, true);


		MoveWindow(textWalls, 0, 68, 0, 0, true);
		//SetWindowPos(propertiesWindow, HWND_TOP, 0.2*width, 0, 0.60*width, 0.3*height, SWP_NOACTIVATE);
		int propertiesX = (int)(0.15f*width);
		int propertiesY = (int)(0.9f*parentHeight)-28;
		int propertiesWidth = (int)(0.7f*width);
		int propertiesHeight = (int)(0.1f*parentHeight);
		MoveWindow(propertiesWindow, propertiesX, propertiesY, propertiesWidth, propertiesHeight, true);
		//SetWindowRgn(propertiesWindow, CreateRoundRectRgn(0, 0, propertiesWidth, propertiesHeight, 20, 20), true);
		//MoveWindow(propertiesButton, 0.4*width, 0.1*height, 100, 50, true);
		MoveWindow(helpText1, 0, (int)(0.05*propertiesHeight), propertiesWidth, 40, true);

		MoveWindow(textSizeDescription, (propertiesWidth / 2) - 400, (int)(0.2*propertiesHeight), 100, 50, true);
		MoveWindow(buttonSizeMinus, (propertiesWidth / 2) - 275, (int)(0.2*propertiesHeight), 50, 50, true);
		MoveWindow(textSize, (propertiesWidth / 2) - 200, (int)(0.2*propertiesHeight), 75, 50, true);
		MoveWindow(buttonSizePlus, (propertiesWidth / 2) - 100, (int)(0.2*propertiesHeight), 50, 50, true);


		MoveWindow(helpText2, 0, (int)(0.5*propertiesHeight), propertiesWidth, 40, true);

		MoveWindow(textSmoothnessDescription, (propertiesWidth / 2) + 15, (int)(0.2*propertiesHeight), 150, 50, true);
		MoveWindow(buttonSmoothnessMinus, (propertiesWidth / 2) + 185, (int)(0.2*propertiesHeight), 50, 50, true);
		MoveWindow(textSmoothness, (propertiesWidth / 2) + 255, (int)(0.2*propertiesHeight), 75, 50, true);
		MoveWindow(buttonSmoothnessPlus, (propertiesWidth / 2) + 350 , (int)(0.2*propertiesHeight), 50, 50, true);
		
		/*
		MoveWindow(textSizeDescription, propertiesWidth / 2 - 200, (int)(0.25*propertiesHeight), 150, 50, true);
		MoveWindow(buttonSizeMinus, propertiesWidth / 2 - 25, (int)(0.25*propertiesHeight), 50, 50, true);
		MoveWindow(textSize, propertiesWidth / 2 + 50, (int)(0.25*propertiesHeight), 75, 50, true);
		MoveWindow(buttonSizePlus, propertiesWidth / 2 + 150, (int)(0.25*propertiesHeight), 50, 50, true);


		MoveWindow(helpText2, 0, (int)(0.5*propertiesHeight), propertiesWidth, 40, true);

		MoveWindow(textSmoothnessDescription, propertiesWidth / 2 - 225, (int)(0.7*propertiesHeight), 175, 50, true);
		MoveWindow(buttonSmoothnessMinus, propertiesWidth / 2 - 25, (int)(0.7*propertiesHeight), 50, 50, true);
		MoveWindow(textSmoothness, propertiesWidth / 2 + 50, (int)(0.7*propertiesHeight), 75, 50, true);
		MoveWindow(buttonSmoothnessPlus, propertiesWidth / 2 + 150, (int)(0.7*propertiesHeight), 50, 50, true);*/


	}


	void PlaneSelectionWindow::UpdatePlaneSelectionValues()
	{
		int thicknessLabel = (int)(planeParams.planeThickness * 100);
		std::wstring thicknessString = std::to_wstring(thicknessLabel) + L"cm";
		SetDlgItemText(propertiesWindow, IDC_PLANE_TEXT_WALLSIZE, thicknessString.c_str());

		int smoothnessLabel = (int)(planeParams.planeSmoothness * 100);
		std::wstring smoothnessString = std::to_wstring(smoothnessLabel) + L"%";
		SetDlgItemText(propertiesWindow, IDC_PLANE_TEXT_WALLSMOOTHNESS, smoothnessString.c_str());
	}

	void PlaneSelectionWindow::Activate()
	{
		SubWindow::Activate();
		EnableWindow(propertiesWindow, true);
		EnableWindow(yesWindow, true);
		EnableWindow(noWindow, true);
	}

	void PlaneSelectionWindow::Deactivate()
	{
		SubWindow::Deactivate();
		EnableWindow(propertiesWindow, false);
		EnableWindow(yesWindow, false);
		EnableWindow(noWindow, false);
	}

	void PlaneSelectionWindow::Show()
	{
		SubWindow::Show();
		ShowWindow(propertiesWindow, SW_SHOW);
		ShowWindow(yesWindow, SW_SHOW);
		ShowWindow(noWindow, SW_SHOW);
	}

	void PlaneSelectionWindow::Hide()
	{
		SubWindow::Hide();
		ShowWindow(propertiesWindow, SW_HIDE);
		ShowWindow(yesWindow, SW_HIDE);
		ShowWindow(noWindow, SW_HIDE);
	}

	void PlaneSelectionWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"PlaneSelectionWindow::CleanUp()");
		wallUi.CleanUp();
		DeleteObject(propertyBackground);
		SubWindow::CleanUp();
	}
}