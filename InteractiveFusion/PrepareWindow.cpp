#include "PrepareWindow.h"
#include "uxtheme.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"
#include "ButtonSlider.h"
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <memory>
namespace InteractiveFusion {

	ButtonSlider scanSizeSlider;
	
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
	HWND hSliderText, hSliderDescription;
	bool startCountDown = false;
	bool countdownFinished = false;
	bool checkForHelpWindowVisibility = false;

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
		
		buttonLayoutMap.emplace(hButtonStart, ButtonLayout());
		buttonLayoutMap[hButtonStart].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::InnerDefault));

		scanSizeSlider.Initialize(windowHandle, _hInstance);
		scanSizeSlider.SetLimits(64, 320);
		scanSizeSlider.SetValue(128);
		scanSizeSlider.SetStep(16);
		scanSizeSlider.SetLayout(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::InnerDefault), StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::InnerDefault));
		

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

		hCountdownText = CreateWindowEx(0, L"STATIC", L"3", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_COUNTDOWN, hInstance, 0);

		SendMessage(hCountdownText, WM_SETFONT, (WPARAM)countdownFont, TRUE);

		ShowWindow(hCountdownText, SW_HIDE);

		hHelpText = CreateWindowEx(0, L"STATIC", L"Press 'RESET' to start over and 'DONE' if you're satisfied with your scan.", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SDEBUG_TEXT_HELP, hInstance, 0);
		SetWindowPos(hHelpText, HWND_TOP, 0, 0, 0, 0, 0);
		SendMessage(hHelpText, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		hStartMeshQuality = CreateWindowEx(0, L"STATIC", L"Mesh Quality: LOW", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PREPARE_TEXT_MESHQUALITY, hInstance, 0);

		SendMessage(hStartMeshQuality, WM_SETFONT, (WPARAM)uiFontSmall, TRUE);


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

		UpdateScanVolumeSize();
	}


	

	LRESULT CALLBACK PrepareWindow::SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		PAINTSTRUCT ps;
		switch (message)
		{
		case WM_LBUTTONDOWN:
		{
			if (scanSizeSlider.HandleLeftMouseButtonDown())
				UpdateScanVolumeSize();
		}
			break;
		case WM_LBUTTONUP:
			scanSizeSlider.HandleLeftMouseButtonUp();
			break;
		case WM_MOUSEMOVE:
			if (scanSizeSlider.HandleMouseMove())
				UpdateScanVolumeSize();
			return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);
			break;
		case WM_DRAWITEM:
		{
			if (scanSizeSlider.HasHandle(((LPDRAWITEMSTRUCT)lParam)->hwndItem))
				return scanSizeSlider.Draw(((LPDRAWITEMSTRUCT)lParam)->hwndItem, lParam);
		}
		break;
		}
		return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);;
	}

	void PrepareWindow::HandleEvents(MainWindow& _parentWindow)
	{
		UpdateCountdown(_parentWindow);
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case PrepareWindowEvent::StateChange:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::StateChange");
				_parentWindow.ChangeScanVolumeSize(voxelsPerMeter);
				_parentWindow.ChangeState(WindowState::Scan);
				break;
			case PrepareWindowEvent::Start:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::Start");
				_parentWindow.SetAndShowHelpMessage(HelpMessage::ScanHelp);
				if (!_parentWindow.IsHelpEnabled())
					startCountDown = true;
				boost::thread(&PrepareWindow::CountdownThread, this);
				
				break;
			case PrepareWindowEvent::HelpChanged:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::HelpChanged");
				_parentWindow.ToggleHelp(helpActive);

				break;
			case PrepareWindowEvent::ScenarioChanged:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::ScenarioChanged");
				_parentWindow.SetScenarioType(scenarioType);
				break;
			}
		


			eventQueue.pop();
		}
	}

	void PrepareWindow::UpdateCountdown(MainWindow& _parentWindow)
	{
		if (checkForHelpWindowVisibility)
		{
			if (!_parentWindow.IsHelpEnabled() || !_parentWindow.IsHelpVisible())
			{
				startCountDown = true;
				checkForHelpWindowVisibility = false;
			}
		}
		if (countdownFinished)
		{
			eventQueue.push(PrepareWindowEvent::StateChange);
			countdownFinished = false;
		}
	}

	void PrepareWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		DebugUtility::DbgOut(L"PrepareWindow::ProcessUI");
		if (IDC_PREPARE_CHECK_HELP == LOWORD(wParam))
		{
			if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_HELP))
			{
				helpActive = true;
				DebugUtility::DbgOut(L"HELP checked");
				//params.showHelp = true;
			}
			else
			{
				helpActive = false;
				//params.showHelp = false;
				DebugUtility::DbgOut(L"HELP unchecked");
			}
			eventQueue.push(PrepareWindowEvent::HelpChanged);
		}

		if (IDC_PREPARE_CHECK_SCENARIO_ONE == LOWORD(wParam))
		{
			if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_ONE))
			{
				if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_TWO))
				{
					CheckDlgButton(windowHandle, IDC_PREPARE_CHECK_SCENARIO_TWO, BST_UNCHECKED);
				}
				scenarioType = ScenarioType::Bowling;
				//stateManager.SetScenarioType(IF_SCENARIO_BOWLING);
				DebugUtility::DbgOut(L"SCENARIO Bowling checked");
			}
			else
			{
				scenarioType = ScenarioType::None;
				//stateManager.SetScenarioType(IF_SCENARIO_NONE);
				DebugUtility::DbgOut(L"SCENARIO Bowling unchecked");
			}
			eventQueue.push(PrepareWindowEvent::ScenarioChanged);
		}

		if (IDC_PREPARE_CHECK_SCENARIO_TWO == LOWORD(wParam))
		{
			if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_TWO))
			{
				if (IsDlgButtonChecked(windowHandle, IDC_PREPARE_CHECK_SCENARIO_ONE))
				{
					CheckDlgButton(windowHandle, IDC_PREPARE_CHECK_SCENARIO_ONE, BST_UNCHECKED);
				}
				scenarioType = ScenarioType::Basic;
				//stateManager.SetScenarioType(IF_SCENARIO_TWO);
				DebugUtility::DbgOut(L"SCENARIO Basic checked");
			}
			else
			{
				scenarioType = ScenarioType::None;
				//stateManager.SetScenarioType(IF_SCENARIO_NONE);
				DebugUtility::DbgOut(L"SCENARIO Basic unchecked");
			}
			eventQueue.push(PrepareWindowEvent::ScenarioChanged);
		}

		if (IDC_PREPARE_BUTTON_START == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"START");
			eventQueue.push(PrepareWindowEvent::Start);
			//StartScan();

			//SetWindowState(SCAN);
		}

	}

	void PrepareWindow::DetermineMeshQuality()
	{
		if (voxelsPerMeter >= 300)
			SetDlgItemText(windowHandle, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: HIGH");
		else if (voxelsPerMeter >= 200)
			SetDlgItemText(windowHandle, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: MEDIUM");
		else
			SetDlgItemText(windowHandle, IDC_PREPARE_TEXT_MESHQUALITY, L"Mesh Quality: LOW");
	}

	void PrepareWindow::UpdateScanVolumeSize()
	{
		voxelsPerMeter = 416 - scanSizeSlider.GetValue();
		DebugUtility::DbgOut(L"VoxelsPerMeter: ", voxelsPerMeter);

		float volumeWidth = 800.0f / (float)voxelsPerMeter;
		float volumeHeight = 512.0f / (float)voxelsPerMeter;
		float volumeDepth = 800.0f / (float)voxelsPerMeter;

		//	DebugUtility::DbgOut(L"vpm: ", (int)voxelsPerMeter);
		wchar_t buffer[256];
		std::wstring s;
		int slen = swprintf(buffer, 255, L"%4.1fm x %4.1fm x %4.1fm", volumeWidth, volumeHeight, volumeDepth);
		s.assign(buffer, slen);

		SetDlgItemText(windowHandle, IDC_PREPARE_SLIDER_TEXT, s.c_str());

		DetermineMeshQuality();

		eventQueue.push(PrepareWindowEvent::ChangeSize);
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

		scanSizeSlider.Resize(width / 2 - 165, (int)(0.40f*height), 400, 50);
		MoveWindow(hSliderText, width / 2 - 70, (int)(0.40*height - 32), 200, 25, true);
		MoveWindow(hSliderDescription, width / 2 - 235, (int)(0.40*height), 60, 50, true);

		//UpdateWindow(windowHandle);
	}

	int PrepareWindow::CountdownThread()
	{
		checkForHelpWindowVisibility = true;
		while (!startCountDown)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			DebugUtility::DbgOut(L"still checking");
		}
		startCountDown = false;
		int secondTimer = 3;
		//FusionWindow* pThis = reinterpret_cast<FusionWindow*>(::GetWindowLongPtr(parentHandle, GWLP_USERDATA));

		
		//pThis->SetWindowState(IF_FUSION_STATE_COUNTDOWN);
		prepareUi.Hide();
		scanSizeSlider.Hide();
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
		DebugUtility::DbgOut(L"CountdownFinished");
		countdownFinished = true;
		//_parentWindow.ChangeState(Scan);
		countdownUi.Hide();
		prepareUi.Show();
		scanSizeSlider.Show();
		return 0;
	}

	void PrepareWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"PrepareWindow::CleanUp()");

		DeleteObject(countdownFont);
		prepareUi.CleanUp();
		countdownUi.CleanUp();
		scanSizeSlider.CleanUp();
		SubWindow::CleanUp();
	}
}