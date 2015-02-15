#include "HelpWindow.h"
#include "uxtheme.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
namespace InteractiveFusion {


	//PREPARE UI
	//map << shared_ptr<ButtonLayout>, HWND > layoutTry;

	HWND hButtonOk;
	HWND hTextHelp;
	HelpWindow::HelpWindow()
	{
	}


	HelpWindow::~HelpWindow()
	{
	}

	void HelpWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		SetWindowLong(windowHandle, GWL_EXSTYLE, GetWindowLong(windowHandle, GWL_EXSTYLE) | WS_EX_TOPMOST);
		SetWindowLong(windowHandle, GWL_STYLE, GetWindowLong(windowHandle, GWL_STYLE) | WS_BORDER);
		hButtonOk = CreateWindowEx(0, L"Button", L"OK", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, windowHandle, (HMENU)IDC_HELP_BUTTON_OK, hInstance, 0);
		SetWindowPos(hButtonOk, HWND_TOP, 0, 0, 0, 0, 0);

		buttonLayoutMap.emplace(hButtonOk, ButtonLayout());
		buttonLayoutMap[hButtonOk].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[hButtonOk].SetFontSize(30);
		hTextHelp = CreateWindowEx(0, L"STATIC", L"Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nunc ut tempus odio, congue molestie dolor. Aliquam sollicitudin venenatis dui, sed malesuada lectus iaculis vitae.", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER, 250, 50, 150, 50, windowHandle, (HMENU)IDC_HELP_TEXT, hInstance, 0);

		SendMessage(hTextHelp, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);


		UpdateWindow(windowHandle);
	}




	LRESULT CALLBACK HelpWindow::SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);;
	}

	void HelpWindow::HandleEvents(MainWindow* _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case HelpWindowEvent::OK:
				DebugUtility::DbgOut(L"HelpWindow::HandleEvents::OK");
				//_parentWindow->ChangeScanVolumeSize(voxelsPerMeter);
				//_parentWindow->ChangeState(Scan);
				break;
				//DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::ChangeSize");
			}

			eventQueue.pop();
		}
	}

	void HelpWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		DebugUtility::DbgOut(L"HelpWindow::ProcessUI");
		if (IDC_HELP_BUTTON_OK == LOWORD(wParam))
		{
			Hide();
			eventQueue.push(HelpWindowEvent::OK);
		}
	}

	void HelpWindow::Show()
	{
		SubWindow::Show();
		SetForegroundWindow(windowHandle);
	}

	void HelpWindow::Hide()
	{
		SubWindow::Hide();
	}

	

	void HelpWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"HelpWindow::RESIZE");

		SubWindow::Resize(parentWidth, parentHeight);

		MoveWindow(hTextHelp, 0, (int)(0.12 * height), width, height - (int)(0.12*height) - 100, true);
		MoveWindow(hButtonOk, (int)(width/2 - 50), (int)(0.9*height-50), 100, 50, true);

	}

	void HelpWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"PrepareWindow::CleanUp()");

		SubWindow::CleanUp();
	}
}