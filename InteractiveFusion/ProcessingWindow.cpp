#include "ProcessingWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"
namespace InteractiveFusion {

	//PROCESSING UI
	GUIContainer procUi;
	HWND buttonFillHoles, buttonRemoveComponents;
	HWND buttonRemoveComponentsPlus, buttonRemoveComponentsMinus;
	HWND textRemoveComponentsLabel, textRemoveComponents;
	HWND buttonProcessingDone;

	ProcessingWindow::ProcessingWindow()
	{
	}


	ProcessingWindow::~ProcessingWindow()
	{
	}

	void ProcessingWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		textRemoveComponents = CreateWindowEx(0, L"STATIC", L"200", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_TEXT_REMOVECOMPONENTS, hInstance, 0);



		SendMessage(textRemoveComponents, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRemoveComponentsLabel = CreateWindowEx(0, L"STATIC", L"COMP. SIZE", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_TEXT_REMOVECOMPONENTS_LABEL, hInstance, 0);

		SendMessage(textRemoveComponentsLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		buttonRemoveComponentsPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_BUTTON_REMOVECOMPONENTS_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRemoveComponentsPlus, ButtonLayout());
		buttonLayoutMap[buttonRemoveComponentsPlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonRemoveComponentsPlus].SetFontSize(60);
		buttonRemoveComponentsMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_BUTTON_REMOVECOMPONENTS_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRemoveComponentsMinus, ButtonLayout());
		buttonLayoutMap[buttonRemoveComponentsMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonRemoveComponentsMinus].SetFontSize(60);
		buttonRemoveComponents = CreateWindowEx(0, L"Button", L"Remove Components", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_BUTTON_REMOVECOMPONENTS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRemoveComponents, ButtonLayout());

		buttonLayoutMap[buttonRemoveComponents].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonRemoveComponents].SetFontSize(30);
		buttonFillHoles = CreateWindowEx(0, L"Button", L"Fill Holes", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_BUTTON_FILLHOLES, hInstance, 0);

		buttonLayoutMap.emplace(buttonFillHoles, ButtonLayout());

		buttonLayoutMap[buttonFillHoles].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonFillHoles].SetFontSize(30);
		buttonProcessingDone = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_BUTTON_DONE, hInstance, 0);

		buttonLayoutMap.emplace(buttonProcessingDone, ButtonLayout());

		buttonLayoutMap[buttonProcessingDone].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Green));

		procUi.Add(buttonFillHoles);
		procUi.Add(buttonRemoveComponents);
		procUi.Add(buttonRemoveComponentsMinus);
		procUi.Add(buttonRemoveComponentsPlus);
		procUi.Add(textRemoveComponents);
		procUi.Add(textRemoveComponentsLabel);
		procUi.Add(buttonProcessingDone);

		UpdateProcessingValues();
	}

	void ProcessingWindow::UpdateProcessingValues()
	{
		int componentSizeLabel = (int)maxComponentSizeToBeRemoved;
		std::wstring componentSizeString = std::to_wstring(componentSizeLabel) + L"";
		SetDlgItemText(windowHandle, IDC_PROCESSING_TEXT_REMOVECOMPONENTS, componentSizeString.c_str());
	}


	LRESULT CALLBACK ProcessingWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		return SubWindow::SubWindowProc(windowHandle, message, wParam, lParam);
	}

	void ProcessingWindow::HandleEvents(MainWindow* _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case ProcessingWindowEvent::StateChange:
				_parentWindow->FinishProcessing();
				_parentWindow->ChangeState(Interaction);
				break;
			case ProcessingWindowEvent::FillHoles:
				DebugUtility::DbgOut(L"ProcessingWindow::HandleEvents::FillHoles");
				_parentWindow->FillHoles(maxHoleSizeToBeClosed);
				break;
			case ProcessingWindowEvent::RemoveComponents:
				DebugUtility::DbgOut(L"ProcessingWindow::HandleEvents::RemoveComponents");
				_parentWindow->RemoveConnectedComponents(maxComponentSizeToBeRemoved);
				//do stuff
				break;
			}

			eventQueue.pop();
		}
	}

	void ProcessingWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if(IDC_PROCESSING_BUTTON_FILLHOLES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"ProcessingWindow::ProcessUI::FillHoles");
			/*appStatus.SetStatusBarMessage(L"Filling holes...");


			if (glSelector.selectedIndex != -1)
			{
				meshData[glSelector.selectedIndex]->ToggleSelectedColor(false);
				meshHelper.FillHoles(glSelector.selectedIndex, params.holeSize);
				meshData[glSelector.selectedIndex]->ToggleSelectedColor(true);
			}
			else
				meshHelper.FillHoles(params.holeSize);*/

			eventQueue.push(ProcessingWindowEvent::FillHoles);
			maxHoleSizeToBeClosed += 1000;
		}
		if (IDC_PROCESSING_BUTTON_DONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(ProcessingWindowEvent::StateChange);
			//parentWindow->ChangeState(Interaction);
			//stateManager.SetWindowMode(IF_MODE_INTERACTION);
		}
		if (IDC_PROCESSING_BUTTON_REMOVECOMPONENTS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			int step = 100;
			if (maxComponentSizeToBeRemoved < 100)
				step = 10;

			maxComponentSizeToBeRemoved += step;
			UpdateProcessingValues();
			//meshHelper.RemoveSmallComponents(1000);
		}
		if (IDC_PROCESSING_BUTTON_REMOVECOMPONENTS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (maxComponentSizeToBeRemoved <= 10)
				return;
			int step = 100;
			if (maxComponentSizeToBeRemoved <= 100)
				step = 10;

			maxComponentSizeToBeRemoved -= step;
			UpdateProcessingValues();
			
			//meshHelper.RemoveSmallComponents(1000);
		}
		if (IDC_PROCESSING_BUTTON_REMOVECOMPONENTS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(ProcessingWindowEvent::RemoveComponents);
		}
	}


	void ProcessingWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"ProcessingWindow::RESIZE");
		//RECT rRect;
		//GetClientRect(parentHandle, &rRect);

		SubWindow::Resize(parentWidth, parentHeight);

		/*if (stateManager.GetDeviceClass() == IF_DEVICE_PC)
		{
			MoveWindow(textRemoveComponentsLabel, width - 180, 290, 100, 30, true);
			MoveWindow(buttonRemoveComponentsMinus, width - 250, 300, 50, 50, true);
			MoveWindow(textRemoveComponents, width - 180, 320, 100, 30, true);
			MoveWindow(buttonRemoveComponentsPlus, width - 60, 300, 50, 50, true);

			MoveWindow(buttonRemoveComponents, width - 225, 370, 200, 50, true);

			MoveWindow(buttonFillHoles, width - 225, 470, 200, 50, true);

			MoveWindow(buttonProcessingDone, width - 200, height - 200, 150, 150, true);
		}
		else if (stateManager.GetDeviceClass() == IF_DEVICE_TABLET)
		{*/
		int textXPosition = (width / 2) - (int)(0.25*width);
		int textWidth = (int)(0.5f*width);
		MoveWindow(textRemoveComponentsLabel, textXPosition, (int)(0.15f*height), textWidth, 25, true);
		MoveWindow(buttonRemoveComponentsMinus, width / 2 - (int)(0.48f*width), (int)(0.15f*height), 50, 50, true);
		MoveWindow(textRemoveComponents, textXPosition, (int)(0.15f*height)+25, textWidth, 25, true);
		MoveWindow(buttonRemoveComponentsPlus, (int)(width / 2) + (int)(0.48f*width)-50, (int)(0.15f*height), 50, 50, true);

			MoveWindow(buttonRemoveComponents, (int)(0.1f*width), (int)(0.30f*height), (int)(0.80f*width), (int)(0.1f*height), true);

			MoveWindow(buttonFillHoles, (int)(0.1f*width), (int)(0.425f*height), (int)(0.80f*width), (int)(0.1f*height), true);

			MoveWindow(buttonProcessingDone, (int)(0.1f*width), (int)(0.75f*height), (int)(0.80f*width), (int)(0.2f*height), true);
		//}
	}




	void ProcessingWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"ProcessingWindow::CleanUp()");
		procUi.CleanUp();

		SubWindow::CleanUp();
	}
}