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
	HWND textRemoveComponentsLabel, textRemoveComponents;
	HWND textHoleSizeLabel, textHoleSize;
	HWND buttonProcessingDone;
	HWND buttonProcessingReset;
	ProcessingWindow::ProcessingWindow()
	{
	}


	ProcessingWindow::~ProcessingWindow()
	{
	}

	void ProcessingWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		textRemoveComponents = CreateWindowEx(0, L"STATIC", std::to_wstring(maxComponentSizeToBeRemoved).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_TEXT_REMOVECOMPONENTS, hInstance, 0);

		SendMessage(textRemoveComponents, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		
		textRemoveComponentsLabel = CreateWindowEx(0, L"STATIC", L"COMP. SIZE", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)0, hInstance, 0);

		SendMessage(textRemoveComponentsLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textHoleSize = CreateWindowEx(0, L"STATIC", std::to_wstring(maxHoleSizeToBeClosed).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_TEXT_HOLESIZE, hInstance, 0);

		SendMessage(textHoleSize, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textHoleSizeLabel = CreateWindowEx(0, L"STATIC", L"HOLE SIZE", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)0, hInstance, 0);

		SendMessage(textHoleSizeLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);


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

		buttonProcessingReset = CreateWindowEx(0, L"BUTTON", L"Reset", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PROCESSING_RESET, hInstance, 0);

		buttonLayoutMap.emplace(buttonProcessingReset, ButtonLayout());
		buttonLayoutMap[buttonProcessingReset].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Red));

		sliderMap.emplace(HoleSize, ButtonSlider());
		sliderMap.emplace(ComponentSize, ButtonSlider());

		for (auto& slider : sliderMap)
		{
			slider.second.Initialize(windowHandle, _hInstance);
			slider.second.SetLayout(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault), StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
			slider.second.SetStep(1);
		}

		sliderMap[HoleSize].SetLimits(10, 100000);
		sliderMap[HoleSize].SetValue(maxHoleSizeToBeClosed);
		
		sliderMap[ComponentSize].SetLimits(10, 10000);
		sliderMap[ComponentSize].SetValue(maxComponentSizeToBeRemoved);

		procUi.Add(sliderMap[HoleSize]);
		procUi.Add(sliderMap[ComponentSize]);
		procUi.Add(buttonFillHoles);
		procUi.Add(buttonRemoveComponents);
		procUi.Add(textHoleSize);
		procUi.Add(textHoleSizeLabel);
		procUi.Add(textRemoveComponents);
		procUi.Add(textRemoveComponentsLabel);
		procUi.Add(buttonProcessingDone);
		procUi.Add(buttonProcessingReset);

		UpdateProcessingValues();
	}

	void ProcessingWindow::UpdateProcessingValues()
	{
		if (maxComponentSizeToBeRemoved != sliderMap[ComponentSize].GetValue())
		{
			maxComponentSizeToBeRemoved = sliderMap[ComponentSize].GetValue();
			std::wstring componentSizeString = std::to_wstring(maxComponentSizeToBeRemoved) + L"";
			SetDlgItemText(windowHandle, IDC_PROCESSING_TEXT_REMOVECOMPONENTS, componentSizeString.c_str());
		}

		if (maxHoleSizeToBeClosed != sliderMap[HoleSize].GetValue())
		{
			maxHoleSizeToBeClosed = sliderMap[HoleSize].GetValue();
			std::wstring holeSizeString = std::to_wstring(maxHoleSizeToBeClosed) + L"";
			SetDlgItemText(windowHandle, IDC_PROCESSING_TEXT_HOLESIZE, holeSizeString.c_str());
		}
	}


	LRESULT CALLBACK ProcessingWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		bool valueChanged = false;
		switch (message)
		{
		case WM_LBUTTONDOWN:
		{
			for (auto& slider : sliderMap)
			{
				if (slider.second.HandleLeftMouseButtonDown())
					valueChanged = true;
			}
		}
			break;
		case WM_LBUTTONUP:
			for (auto& slider : sliderMap)
				slider.second.HandleLeftMouseButtonUp();
			break;
		case WM_MOUSEMOVE:
		{
			for (auto& slider : sliderMap)
			{
				if (slider.second.HandleMouseMove())
					valueChanged = true;
			}
		}
		break;
		case WM_DRAWITEM:
		{
			for (auto& slider : sliderMap)
			{
				if (slider.second.HasHandle(((LPDRAWITEMSTRUCT)lParam)->hwndItem))
					return slider.second.Draw(((LPDRAWITEMSTRUCT)lParam)->hwndItem, lParam);
			}
		}
		break;
		}

		if (valueChanged)
		{
			valueChanged = false;
			UpdateProcessingValues();
		}

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
			case ProcessingWindowEvent::Reset:
				DebugUtility::DbgOut(L"ProcessingWindow::HandleEvents::Reset");
				_parentWindow->ReloadModel();
				break;
			}

			eventQueue.pop();
		}
	}

	void ProcessingWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_PROCESSING_BUTTON_REMOVECOMPONENTS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(ProcessingWindowEvent::RemoveComponents);
		}
		if(IDC_PROCESSING_BUTTON_FILLHOLES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(ProcessingWindowEvent::FillHoles);
		}
		if (IDC_PROCESSING_RESET == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"ProcessingWindowEvent::ProcessUI::Reset");
			eventQueue.push(ProcessingWindowEvent::Reset);
		}
		if (IDC_PROCESSING_BUTTON_DONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(ProcessingWindowEvent::StateChange);
		}
		
	}

	void ProcessingWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"ProcessingWindow::RESIZE");

		SubWindow::Resize(parentWidth, parentHeight);

		int controlX = (int)(0.02f*width);
		int controlWidth = (int)(0.96f*width);
		int buttonHeight = (int)(0.08f*height);
		int componentsY = (int)(0.35f*height);
		int holeSizeY = componentsY + buttonHeight + 115;

		//Remove Components Group
		MoveWindow(buttonProcessingReset, controlX, (int)(0.05f*height), controlWidth, (int)(0.2f*height), true);
		MoveWindow(textRemoveComponentsLabel, controlX, componentsY, controlWidth / 2, 25, true);
		MoveWindow(textRemoveComponents, controlX + controlWidth / 2, componentsY, controlWidth / 2, 25, true);
		sliderMap[ComponentSize].Resize(controlX, componentsY + 30, controlWidth, 50);
		MoveWindow(buttonRemoveComponents, controlX, componentsY + 90, controlWidth, buttonHeight, true);

		MoveWindow(textHoleSizeLabel, controlX, holeSizeY, controlWidth / 2, 25, true);
		MoveWindow(textHoleSize, controlX + controlWidth / 2, holeSizeY, controlWidth / 2, 25, true);
		sliderMap[HoleSize].Resize(controlX, holeSizeY + 30, controlWidth, 50);
		MoveWindow(buttonFillHoles, controlX, holeSizeY + 90, controlWidth, buttonHeight, true);


		MoveWindow(buttonProcessingDone, controlX, (int)(0.75f*height), controlWidth, (int)(0.2f*height), true);
	}




	void ProcessingWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"ProcessingWindow::CleanUp()");
		procUi.CleanUp();
		for (auto& slider : sliderMap)
			slider.second.CleanUp();
		sliderMap.clear();
		SubWindow::CleanUp();
	}
}