#include "PlaneCutWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"

namespace InteractiveFusion {

	//PROCESSING UI
	GUIContainer planeCutUi;
	HWND buttonPlane, buttonExecutePlaneCut;
	HWND buttonAxisX, buttonAxisY, buttonAxisZ;
	HWND buttonPlaneCutDone;

	bool showPlaneCutPlane = false;

	

	PlaneCutWindow::PlaneCutWindow()
	{
	}


	PlaneCutWindow::~PlaneCutWindow()
	{
	}

	void PlaneCutWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		buttonPlane = CreateWindowEx(0, L"Button", L"Show Plane", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_SHOW_PLANE, hInstance, 0);

		buttonLayoutMap.emplace(buttonPlane, ButtonLayout());

		buttonLayoutMap[buttonPlane].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonPlane].SetFontSize(30);
		
		buttonAxisX = CreateWindowEx(0, L"Button", L"X", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_XAXIS, hInstance, 0);

		buttonLayoutMap.emplace(buttonAxisX, ButtonLayout());

		buttonLayoutMap[buttonAxisX].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonAxisX].SetFontSize(40);

		buttonAxisY = CreateWindowEx(0, L"Button", L"Y", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_YAXIS, hInstance, 0);

		buttonLayoutMap.emplace(buttonAxisY, ButtonLayout());

		buttonLayoutMap[buttonAxisY].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonAxisY].SetFontSize(40);

		buttonAxisZ = CreateWindowEx(0, L"Button", L"Z", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_ZAXIS, hInstance, 0);

		buttonLayoutMap.emplace(buttonAxisZ, ButtonLayout());

		buttonLayoutMap[buttonAxisZ].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonAxisZ].SetFontSize(40);

		buttonExecutePlaneCut = CreateWindowEx(0, L"Button", L"Plane Cut", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_EXECUTE, hInstance, 0);

		buttonLayoutMap.emplace(buttonExecutePlaneCut, ButtonLayout());

		buttonLayoutMap[buttonExecutePlaneCut].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonExecutePlaneCut].SetFontSize(30);

		buttonPlaneCutDone = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_DONE, hInstance, 0);

		buttonLayoutMap.emplace(buttonPlaneCutDone, ButtonLayout());

		buttonLayoutMap[buttonPlaneCutDone].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Green));

		planeCutUi.Add(buttonPlane);
		planeCutUi.Add(buttonAxisX);
		planeCutUi.Add(buttonAxisY);
		planeCutUi.Add(buttonAxisZ);
		planeCutUi.Add(buttonExecutePlaneCut);
		planeCutUi.Add(buttonPlaneCutDone);

		ChangePlaneCutMode(AxisY);

	}


	LRESULT CALLBACK PlaneCutWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		return SubWindow::SubWindowProc(windowHandle, message, wParam, lParam);
	}

	void PlaneCutWindow::HandleEvents(MainWindow* _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case PlaneCutWindowEvent::StateChange:
				_parentWindow->FinishProcessing();
				_parentWindow->ChangeState(Processing);
				break;
			case PlaneCutWindowEvent::ActivatePlane:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ActivatePlane");
				_parentWindow->SetPlaneRenderer(showPlaneCutPlane);
				break;
			case PlaneCutWindowEvent::ExecutePlaneCut:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ExecutePlaneCut");
				_parentWindow->ExecutePlaneCut();
				break;
			case PlaneCutWindowEvent::ChangePlaneCutMode:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ChangePlaneCutMode");
				_parentWindow->ChangePlaneCutMode(planeCutMode);
				break;
			}

			eventQueue.pop();
		}
	}

	void PlaneCutWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_PLANECUT_SHOW_PLANE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::ShowPlane");

			showPlaneCutPlane = !showPlaneCutPlane;

			if (showPlaneCutPlane)
				buttonLayoutMap[buttonPlane].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));
			else
				buttonLayoutMap[buttonPlane].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));

			buttonLayoutMap[buttonPlane].SetFontSize(30);
			eventQueue.push(PlaneCutWindowEvent::ActivatePlane);

		}
		if (IDC_PLANECUT_XAXIS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::X Axis");
			ChangePlaneCutMode(AxisX);
		}
		if (IDC_PLANECUT_YAXIS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Y Axis");
			ChangePlaneCutMode(AxisY);
		}
		if (IDC_PLANECUT_ZAXIS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Z Axis");
			ChangePlaneCutMode(AxisZ);
		}
		if (IDC_PLANECUT_EXECUTE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Execute");
			eventQueue.push(PlaneCutWindowEvent::ExecutePlaneCut);
		}
		if (IDC_PLANECUT_DONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Done");
			eventQueue.push(PlaneCutWindowEvent::StateChange);
		}
	}


	void PlaneCutWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"PlaneCutWindow::RESIZE");


		SubWindow::Resize(parentWidth, parentHeight);

		MoveWindow(buttonAxisX, (int)(0.1f*width), (int)(0.27f*height), (int)(0.25f*width), (int)(0.05f*height), true);
		MoveWindow(buttonAxisY, (int)(0.375f*width), (int)(0.27f*height), (int)(0.25f*width), (int)(0.05f*height), true);
		MoveWindow(buttonAxisZ, (int)(0.65f*width), (int)(0.27f*height), (int)(0.25f*width), (int)(0.05f*height), true);
		MoveWindow(buttonPlane, (int)(0.1f*width), (int)(0.15f*height), (int)(0.80f*width), (int)(0.1f*height), true);

		MoveWindow(buttonExecutePlaneCut, (int)(0.1f*width), (int)(0.425f*height), (int)(0.80f*width), (int)(0.1f*height), true);

		MoveWindow(buttonPlaneCutDone, (int)(0.1f*width), (int)(0.75f*height), (int)(0.80f*width), (int)(0.2f*height), true);
	}

	void PlaneCutWindow::ChangePlaneCutMode(PlaneCutMode _mode)
	{
		planeCutMode = _mode;

		buttonLayoutMap[buttonAxisX].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonAxisY].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonAxisZ].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		
		
		if (planeCutMode == AxisX)
			buttonLayoutMap[buttonAxisX].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));
		else if (planeCutMode == AxisY)
			buttonLayoutMap[buttonAxisY].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));
		else if (planeCutMode == AxisZ)
			buttonLayoutMap[buttonAxisZ].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));

		buttonLayoutMap[buttonAxisX].SetFontSize(40);
		buttonLayoutMap[buttonAxisY].SetFontSize(40);
		buttonLayoutMap[buttonAxisZ].SetFontSize(40);
		planeCutUi.Redraw();
		eventQueue.push(PlaneCutWindowEvent::ChangePlaneCutMode);

	}

	void PlaneCutWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"ProcessingWindow::CleanUp()");
		planeCutUi.CleanUp();

		SubWindow::CleanUp();
	}
}