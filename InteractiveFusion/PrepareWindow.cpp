#include "PrepareWindow.h"
#include "uxtheme.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"

#include <boost/thread.hpp>

namespace InteractiveFusion {


	//PREPARE UI
	GUIContainer prepareUi;
	HWND hButtonStart;
	HWND hStartTextBig, hStartTextSmall, hHelpText;
	HWND hStartMeshQuality;
	HWND hCheckBoxHelp;
	HWND hCheckBoxScenarioOne, hCheckBoxScenarioTwo;

	GUIContainer countdownUi;
	HWND hCountdownText;

	//SLIDER IN PREPARE UI
	HWND hButtonSlider, hSliderBackground, hSliderText, hSliderDescription;
	int sliderPos = -1;
	POINT sliderAnchor;
	bool sliderDown = false;

	bool countdownFinished = false;

	//map << shared_ptr<ButtonLayout>, HWND > layoutTry;

	PrepareWindow::PrepareWindow()
	{
	}


	PrepareWindow::~PrepareWindow()
	{
	}

	void PrepareWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);
		
		countdownFont = CreateFont(300, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());

		hButtonStart = CreateWindowEx(0, L"Button", L"START", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, windowHandle, (HMENU)IDC_PREPARE_BUTTON_START, hInstance, 0);
		SetWindowPos(hButtonStart, HWND_TOP, 0, 0, 0, 0, 0);
		//buttonLayoutMap[hButtonStart] = innerDefaultLayout;
		DebugUtility::DbgOut(L"Initializing hButtonStart");
		//trialMap[hButtonStart] = unique_ptr<ButtonLayout>(new ButtonLayout(innerDefaultParams));
		
		//trialMap[hButtonStart]->SetLayoutParams(innerDefaultParams);
		buttonLayoutMap.emplace(hButtonStart, ButtonLayout());
		buttonLayoutMap[hButtonStart].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InnerDefault));

		hButtonSlider = CreateWindowEx(0, L"Button", L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 50, 500, 150, 50, windowHandle, (HMENU)IDC_PREPARE_SLIDER_BUTTON, hInstance, 0);
		SetWindowPos(hButtonSlider, HWND_TOP, 0, 0, 0, 0, 0);
		buttonLayoutMap.emplace(hButtonSlider, ButtonLayout());
		buttonLayoutMap[hButtonSlider].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InnerDefault));
		//buttonLayoutMap[hButtonSlider].SetInactiveGradient(Gradient{ { 50, 50, 50 }, { 30, 30, 30 } });
		hSliderBackground = CreateWindowEx(0, L"Button", L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 50, 500, 400, 50, windowHandle, (HMENU)IDC_PREPARE_SLIDER_BACKGROUND, hInstance, 0);
		SetWindowPos(hSliderBackground, hButtonSlider, 0, 0, 0, 0, 0);

		buttonLayoutMap.emplace(hSliderBackground, ButtonLayout());
		buttonLayoutMap[hSliderBackground].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InnerDefault));
		buttonLayoutMap[hSliderBackground].SetInactiveGradient(Gradient{ { 15, 15, 15 }, { 5, 5, 5 } });
		hSliderText = CreateWindowEx(0, L"STATIC", L"0m", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_CENTER | SS_CENTERIMAGE, 50, 50, 100, 50, windowHandle, (HMENU)IDC_PREPARE_SLIDER_TEXT, hInstance, 0);
		SetWindowPos(hSliderText, HWND_TOP, 0, 0, 0, 0, 0);
		SendMessage(hSliderText, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);

		hSliderDescription = CreateWindowEx(0, L"STATIC", L"Size", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | WS_CLIPSIBLINGS, 50, 50, 100, 50, windowHandle, (HMENU)IDC_PREPARE_SLIDER_LABEL, hInstance, 0);
		SendMessage(hSliderDescription, WM_SETFONT, (WPARAM)uiFontBig, TRUE);
		SetWindowPos(hSliderDescription, HWND_TOP, 0, 0, 0, 0, 0);
		hStartTextBig = CreateWindowEx(0, L"STATIC", L"Let's start with scanning your scene.", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_STATUS_BIG, hInstance, 0);

		SendMessage(hStartTextBig, WM_SETFONT, (WPARAM)uiFontBig, TRUE);

		hStartTextSmall = CreateWindowEx(0, L"STATIC", L"Please move the sensor slowly and smoothly.", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_STATUS_SMALL, hInstance, 0);

		SendMessage(hStartTextSmall, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);
		//ShowWindow(hStatusText, SW_HIDE);

		hCountdownText = CreateWindowEx(0, L"STATIC", L"3", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_COUNTDOWN, hInstance, 0);

		SendMessage(hCountdownText, WM_SETFONT, (WPARAM)countdownFont, TRUE);

		ShowWindow(hCountdownText, SW_HIDE);

		hHelpText = CreateWindowEx(0, L"STATIC", L"Press 'RESET' to start over and 'DONE' if you're satisfied with your scan.", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SDEBUG_TEXT_HELP, hInstance, 0);
		SetWindowPos(hHelpText, HWND_TOP, 0, 0, 0, 0, 0);
		SendMessage(hHelpText, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		hStartMeshQuality = CreateWindowEx(0, L"STATIC", L"Mesh Quality: LOW", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_TEXT_MESHQUALITY, hInstance, 0);

		SendMessage(hStartMeshQuality, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);


		EnableWindow(hButtonSlider, false);
		EnableWindow(hSliderBackground, false);


		hCheckBoxHelp = CreateWindowEx(0, L"Button", L"Show Help", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | BS_AUTOCHECKBOX, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_CHECK_HELP, hInstance, 0);



		SendMessage(hCheckBoxHelp, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);
		SetWindowTheme(hCheckBoxHelp, L"wstr", L"wstr");

		CheckDlgButton(windowHandle, IDC_PREPARE_CHECK_HELP, BST_CHECKED);

		hCheckBoxScenarioOne = CreateWindowEx(0, L"Button", L"Scenario BOWLING", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | BS_AUTOCHECKBOX, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_CHECK_SCENARIO_ONE, hInstance, 0);

		SendMessage(hCheckBoxScenarioOne, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);
		SetWindowTheme(hCheckBoxScenarioOne, L"wstr", L"wstr");

		hCheckBoxScenarioTwo = CreateWindowEx(0, L"Button", L"Scenario TWO", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | BS_AUTOCHECKBOX, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_CHECK_SCENARIO_TWO, hInstance, 0);

		SendMessage(hCheckBoxScenarioTwo, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);
		SetWindowTheme(hCheckBoxScenarioTwo, L"wstr", L"wstr");



		UpdateWindow(windowHandle);
		prepareUi.Add(hButtonStart);
		prepareUi.Add(hButtonSlider);
		prepareUi.Add(hSliderBackground);
		prepareUi.Add(hSliderText);
		prepareUi.Add(hSliderDescription);
		prepareUi.Add(hStartTextBig);
		prepareUi.Add(hStartTextSmall);
		prepareUi.Add(hHelpText);
		prepareUi.Add(hStartMeshQuality);
		prepareUi.Add(hCheckBoxHelp);
		prepareUi.Add(hCheckBoxScenarioOne);
		prepareUi.Add(hCheckBoxScenarioTwo);

		countdownUi.Add(hCountdownText);
		countdownUi.Add(hHelpText);

		UpdateButtonSliderValue();
	}


	

	LRESULT CALLBACK PrepareWindow::SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		PAINTSTRUCT ps;
		switch (message)
		{
		case WM_LBUTTONDOWN:
				if (!sliderDown)
				{
					if (IsMouseInHandle(hButtonSlider))
					{
						sliderDown = true;
						buttonLayoutMap[hButtonSlider].SetInactiveGradient(Gradient{ { 50, 50, 50 }, { 40, 40, 40 } });
						RedrawButtonSlider();
					}
				}
			break;
		case WM_LBUTTONUP:
			if (sliderDown)
			{
					sliderDown = false;
					buttonLayoutMap[hButtonSlider].SetInactiveGradient(Gradient{ { 30, 30, 30 }, { 20, 20, 20 } });
					RedrawButtonSlider();
			}
			break;
		case WM_MOUSEMOVE:
			if (mouseDown)
			{
				if (IsMouseInHandle(hButtonSlider))
					UpdateButtonSlider();
			}
			return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);
			break;
		}
		return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);;
	}

	void PrepareWindow::HandleEvents(MainWindow* _parentWindow)
	{
		if (countdownFinished)
		{
			eventQueue.push(PrepareWindowEvent::StateChange);
			countdownFinished = false;
		}
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case PrepareWindowEvent::StateChange:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::StateChange");
				_parentWindow->ChangeScanVolumeSize(voxelsPerMeter);
				_parentWindow->ChangeState(Scan);
				break;
				//DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::ChangeSize");
			}

			eventQueue.pop();
		}
	}

	void PrepareWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		DebugUtility::DbgOut(L"PrepareWindow::ProcessUI");
		if (IDC_PREPARE_CHECK_HELP == LOWORD(wParam))
		{
			if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_HELP))
			{
				DebugUtility::DbgOut(L"HELP checked");
				//params.showHelp = true;
			}
			else
			{
				//params.showHelp = false;
				DebugUtility::DbgOut(L"HELP unchecked");
			}
		}

		if (IDC_PREPARE_CHECK_SCENARIO_ONE == LOWORD(wParam))
		{
			if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_ONE))
			{
				if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_TWO))
				{
					CheckDlgButton(windowHandle, IDC_PREPARE_CHECK_SCENARIO_TWO, BST_UNCHECKED);
				}
				//stateManager.SetScenarioType(IF_SCENARIO_BOWLING);
				DebugUtility::DbgOut(L"SCENARIO ONE checked");
			}
			else
			{
				//stateManager.SetScenarioType(IF_SCENARIO_NONE);
				DebugUtility::DbgOut(L"SCENARIO ONE unchecked");
			}
		}

		if (IDC_PREPARE_CHECK_SCENARIO_TWO == LOWORD(wParam))
		{
			if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_TWO))
			{
				if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_ONE))
				{
					CheckDlgButton(windowHandle, IDC_PREPARE_CHECK_SCENARIO_ONE, BST_UNCHECKED);
				}
				//stateManager.SetScenarioType(IF_SCENARIO_TWO);
				DebugUtility::DbgOut(L"SCENARIO TWO checked");
			}
			else
			{
				//stateManager.SetScenarioType(IF_SCENARIO_NONE);
				DebugUtility::DbgOut(L"SCENARIO TWO unchecked");
			}
		}

		if (IDC_PREPARE_BUTTON_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"START");
			boost::thread(&PrepareWindow::CountdownThread, this);
			//StartScan();

			//SetWindowState(SCAN);
		}

	}

	

	void PrepareWindow::UpdateButtonSlider()
	{

		POINT p;
		GetCursorPos(&p);
		RECT borderRect; GetClientRect(hSliderBackground, &borderRect);
		RECT sliderRect; GetClientRect(hButtonSlider, &sliderRect);
		MapWindowPoints(hSliderBackground, windowHandle, (POINT *)&borderRect, 2);

		ScreenToClient(windowHandle, &p);
		if (p.x != sliderAnchor.x && p.x > borderRect.left + (sliderRect.right / 2) && p.x < borderRect.right - (sliderRect.right / 2))
		{
			sliderAnchor.x = p.x;
			RECT rect; GetClientRect(windowHandle, &rect);

			sliderPos = p.x - (int)(sliderRect.right / 2.0f) - borderRect.left;
			MoveButtonSlider(sliderPos);

			//32 <--> 256
			float channelWidth = borderRect.right - (sliderRect.right / 2.0f) - borderRect.left - (sliderRect.right / 2.0f);
			float percent = sliderPos / channelWidth;
			int value = (int)(percent * 352.0f) - 1;
			voxelsPerMeter = (416 - value);

			float volumeWidth = 800.0f / (float)voxelsPerMeter;
			float volumeHeight = 512.0f / (float)voxelsPerMeter;
			float volumeDepth = 800.0f / (float)voxelsPerMeter;

		//	DebugUtility::DbgOut(L"vpm: ", (int)voxelsPerMeter);
			wchar_t buffer[256];
			std::wstring s;
			int slen = swprintf(buffer, 255, L"%4.1fm x %4.1fm x %4.1fm", volumeWidth, volumeHeight, volumeDepth);
			s.assign(buffer, slen);

			SetDlgItemText(windowHandle, IDC_PREPARE_SLIDER_TEXT, s.c_str());

			//DetermineMeshQuality();
		}
		eventQueue.push(PrepareWindowEvent::ChangeSize);
	}

	void PrepareWindow::UpdateButtonSliderValue()
	{
		RECT borderRect; GetClientRect(hSliderBackground, &borderRect);
		RECT sliderRect; GetClientRect(hButtonSlider, &sliderRect);
		MapWindowPoints(hSliderBackground, windowHandle, (POINT *)&borderRect, 2);

		int value = 416 - (int)voxelsPerMeter;
		float percent = (value - 1) / 352.0f;
		float channelWidth = borderRect.right - (sliderRect.right / 2.0f) - borderRect.left - (sliderRect.right / 2.0f);
		sliderPos = (int)(percent * channelWidth);

		float volumeWidth = 800.0f / (float)voxelsPerMeter;
		float volumeHeight = 512.0f / (float)voxelsPerMeter;
		float volumeDepth = 800.0f / (float)voxelsPerMeter;

		wchar_t buffer[256];
		std::wstring s;
		int slen = swprintf(buffer, 255, L"%4.1fm x %4.1fm x %4.1fm", volumeWidth, volumeHeight, volumeDepth);
		s.assign(buffer, slen);

		SetDlgItemText(windowHandle, IDC_PREPARE_SLIDER_TEXT, s.c_str());
		MoveButtonSlider(sliderPos);
	}


	void PrepareWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"PrepareWindow::RESIZE");
		//RECT rRect;
		//GetClientRect(parentHandle, &rRect);

		SubWindow::Resize(parentWidth, parentHeight);
		
		DebugUtility::DbgOut(L"width: ", width);
		DebugUtility::DbgOut(L"height: ", height);

		MoveWindow(hCheckBoxHelp, width / 2 - 50, (int)(0.78*height) - 30, 100, 30, true);

		MoveWindow(hCheckBoxScenarioOne, width / 2 - 170, (int)(0.85*height) - 30, 150, 30, true);

		MoveWindow(hCheckBoxScenarioTwo, width / 2 + 20, (int)(0.85*height) - 30, 150, 30, true);

		MoveWindow(hButtonStart, (int)(0.425*width), (int)(0.6*height), (int)(0.15*width), 50, true);

		MoveWindow(hStartTextBig, 0, (int)(0.12 * height), width, (int)(0.07 * height), true);

		MoveWindow(hStartTextSmall, 0, (int)(0.20 * height), width, (int)(0.05 * height), true);


		MoveWindow(hHelpText, 0, (int)(0.90*height), width, 40, true);

		MoveWindow(hCountdownText, 0, 0, width, height, true);

		MoveWindow(hStartMeshQuality, width / 2 - 70, (int)(0.40*height) + 55, 200, 30, true);
		/*MoveWindow(hStartTextBig, width / 2 - 250, height / 2 - 210, 500, 40, true);

		MoveWindow(hStartTextSmall, width / 2 - 250, height / 2 - 170, 500, 30, true);
		

		MoveWindow(hHelpText, width / 2 - 450, height - 150, 900, 40, true);

		MoveWindow(hCountdownText, width / 2 - 250, height / 2 - 150, 500, 300, true);

		MoveWindow(hStartMeshQuality, width / 2 - 70, height / 2 + 50, 200, 30, true);

		MoveWindow(hCheckBoxHelp, width / 2 - 50, height - 100, 100, 30, true);

		MoveWindow(hCheckBoxScenarioOne, width / 2 - 170, height - 70, 150, 30, true);

		MoveWindow(hCheckBoxScenarioTwo, width / 2 + 20, height - 70, 150, 30, true);

		MoveWindow(hButtonStart, width / 2 - 100, height - 220, 200, 50, true);*/

		//fusionDebugDialog.MoveOnResize();

		if (sliderPos == -1)
		{
			MoveButtonSlider(0);
		}
		else
		{
			MoveButtonSlider(sliderPos);
		}

		UpdateWindow(windowHandle);
	}

	void PrepareWindow::MoveButtonSlider(int pos)
	{
		MoveWindow(hButtonSlider, width / 2 - 165 + pos, (int)(0.40*height), 50, 50, true);
		MoveWindow(hSliderBackground, width / 2 - 165, (int)(0.40*height), 400, 50, true);
		MoveWindow(hSliderText, width / 2 - 70, (int)(0.40*height - 32), 200, 25, true);
		MoveWindow(hSliderDescription, width / 2 - 235, (int)(0.40*height), 60, 50, true);
		/*
		MoveWindow(hButtonSlider, width / 2 - 165 + pos, height / 2 - 30, 50, 50, true);
		MoveWindow(hSliderBackground, width / 2 - 165, height / 2 - 30, 400, 50, true);
		MoveWindow(hSliderText, width / 2 - 70, height / 2 + 30, 200, 25, true);
		MoveWindow(hSliderDescription, width / 2 - 235, height / 2 - 25, 60, 50, true);*/
	}
	
	void PrepareWindow::RedrawButtonSlider()
	{
		RECT rect;
		GetClientRect(hButtonSlider, &rect);
		InvalidateRect(hButtonSlider, &rect, TRUE);
	}


	int PrepareWindow::CountdownThread()
	{

		int secondTimer = 3;
		//FusionWindow* pThis = reinterpret_cast<FusionWindow*>(::GetWindowLongPtr(parentHandle, GWLP_USERDATA));

		
		//pThis->SetWindowState(IF_FUSION_STATE_COUNTDOWN);
		prepareUi.Hide();
		countdownUi.Show();
		//ShowWindow(hCountdownText, SW_SHOW);

		while (secondTimer > 0)
		{
			SetWindowText(hCountdownText, std::to_wstring(secondTimer).c_str());
			//SetDlgItemText(_windowHandle, IDC_PREPARE_COUNTDOWN, std::to_wstring(secondTimer).c_str());
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			secondTimer--;
		}
		//OutputDebugString(L"wel")
		countdownFinished = true;
		//_parentWindow->ChangeState(Scan);
		countdownUi.Hide();
		prepareUi.Show();
		return 0;
	}

	void PrepareWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"PrepareWindow::CleanUp()");

		DeleteObject(countdownFont);
		prepareUi.CleanUp();
		countdownUi.CleanUp();
		SubWindow::CleanUp();
	}
}