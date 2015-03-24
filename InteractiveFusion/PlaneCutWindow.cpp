#include "PlaneCutWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"

namespace InteractiveFusion {

	//PROCESSING UI
	GUIContainer planeCutUi;
	HWND buttonPlaneRotation, buttonPlaneTranslation;
	HWND buttonExecutePlaneCut;
	HWND hButtonFreeCamera;
	HWND buttonAxisX, buttonAxisY, buttonAxisZ;
	HWND buttonPlaneCutDone;
	HWND buttonCutReset;

	bool rotatePlane = false;

	

	PlaneCutWindow::PlaneCutWindow()
	{
	}


	PlaneCutWindow::~PlaneCutWindow()
	{
	}

	void PlaneCutWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		buttonPlaneRotation = CreateWindowEx(0, L"Button", L"Rotate", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_ROTATE, hInstance, 0);

		buttonLayoutMap.emplace(buttonPlaneRotation, ButtonLayout());

		buttonLayoutMap[buttonPlaneRotation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonPlaneRotation].SetFontSize(30);

		buttonPlaneTranslation = CreateWindowEx(0, L"Button", L"Translate", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_TRANSLATE, hInstance, 0);

		buttonLayoutMap.emplace(buttonPlaneTranslation, ButtonLayout());

		buttonLayoutMap[buttonPlaneTranslation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonPlaneTranslation].SetFontSize(30);
		
		buttonAxisX = CreateWindowEx(0, L"Button", L"X", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_XAXIS, hInstance, 0);

		buttonLayoutMap.emplace(buttonAxisX, ButtonLayout());

		buttonLayoutMap[buttonAxisX].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonAxisX].SetFontSize(40);

		buttonAxisY = CreateWindowEx(0, L"Button", L"Y", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_YAXIS, hInstance, 0);

		buttonLayoutMap.emplace(buttonAxisY, ButtonLayout());

		buttonLayoutMap[buttonAxisY].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonAxisY].SetFontSize(40);

		buttonAxisZ = CreateWindowEx(0, L"Button", L"Z", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_ZAXIS, hInstance, 0);

		buttonLayoutMap.emplace(buttonAxisZ, ButtonLayout());

		buttonLayoutMap[buttonAxisZ].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonAxisZ].SetFontSize(40);

		buttonExecutePlaneCut = CreateWindowEx(0, L"Button", L"Cut", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_EXECUTE, hInstance, 0);

		buttonLayoutMap.emplace(buttonExecutePlaneCut, ButtonLayout());

		buttonLayoutMap[buttonExecutePlaneCut].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonExecutePlaneCut].SetFontSize(30);

		buttonPlaneCutDone = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_DONE, hInstance, 0);

		buttonLayoutMap.emplace(buttonPlaneCutDone, ButtonLayout());

		buttonLayoutMap[buttonPlaneCutDone].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Green));

		buttonCutReset = CreateWindowEx(0, L"BUTTON", L"Reset", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_RESET, hInstance, 0);

		buttonLayoutMap.emplace(buttonCutReset, ButtonLayout());
		buttonLayoutMap[buttonCutReset].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Red));

		hButtonFreeCamera = CreateWindowEx(0, L"BUTTON", L"Free Camera", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW | BS_MULTILINE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_PLANECUT_FREE_CAMERA, hInstance, 0);

		buttonLayoutMap.emplace(hButtonFreeCamera, ButtonLayout());
		buttonLayoutMap[hButtonFreeCamera].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[hButtonFreeCamera].SetFontSize(30);

		planeCutUi.Add(buttonPlaneRotation);
		planeCutUi.Add(buttonPlaneTranslation);
		planeCutUi.Add(hButtonFreeCamera);
		planeCutUi.Add(buttonAxisX);
		planeCutUi.Add(buttonAxisY);
		planeCutUi.Add(buttonAxisZ);
		planeCutUi.Add(buttonExecutePlaneCut);
		planeCutUi.Add(buttonPlaneCutDone);
		planeCutUi.Add(buttonCutReset);

		ChangePlaneCutAxis(PlaneCutAxis::AxisY);
		ChangePlaneTransformation(PlaneCutTransformation::Translate);
	}


	LRESULT CALLBACK PlaneCutWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		return SubWindow::SubWindowProc(windowHandle, message, wParam, lParam);
	}

	void PlaneCutWindow::HandleEvents(MainWindow& _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case PlaneCutWindowEvent::StateChange:
				_parentWindow.FinishProcessing();
				_parentWindow.ChangeState(WindowState::Processing);
				_parentWindow.SetAndShowHelpMessage(HelpMessage::ProcessingHelp);
				break;
			case PlaneCutWindowEvent::ChangePlaneTransformation:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ChangePlaneCutTransformation");
				_parentWindow.ChangePlaneCutTransformation(planeCutTransformation);
				break;
			case PlaneCutWindowEvent::ExecutePlaneCut:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ExecutePlaneCut");
				_parentWindow.ExecutePlaneCut();
				break;
			case PlaneCutWindowEvent::ChangePlaneCutAxis:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ChangePlaneCutAxis");
				_parentWindow.ChangePlaneCutAxis(planeCutAxis);
				break;
			case PlaneCutWindowEvent::Reset:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::Reset");
				_parentWindow.ReloadModel();
				break;
			case PlaneCutWindowEvent::ChangeCameraMovement:
				DebugUtility::DbgOut(L"PlaneCutWindow::HandleEvents::ChangeCameraMode");
				_parentWindow.SetCameraMovementEnabled(cameraMovementEnabled);
				break;
			}

			eventQueue.pop();
		}
	}

	void PlaneCutWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_PLANECUT_ROTATE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (cameraMovementEnabled)
			{
				ToggleCameraMovementEnabled();
				eventQueue.push(PlaneCutWindowEvent::ChangeCameraMovement);
			}
			ChangePlaneTransformation(PlaneCutTransformation::Rotate);

			eventQueue.push(PlaneCutWindowEvent::ChangePlaneTransformation);

		}
		if (IDC_PLANECUT_XAXIS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::X Axis");
			ChangePlaneCutAxis(PlaneCutAxis::AxisX);
		}
		if (IDC_PLANECUT_YAXIS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Y Axis");
			ChangePlaneCutAxis(PlaneCutAxis::AxisY);
		}
		if (IDC_PLANECUT_ZAXIS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Z Axis");
			ChangePlaneCutAxis(PlaneCutAxis::AxisZ);
		}
		if (IDC_PLANECUT_RESET == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"PlaneCutWindow::ProcessUI::Reset");
			eventQueue.push(PlaneCutWindowEvent::Reset);
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
		if (IDC_PLANECUT_FREE_CAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			ToggleCameraMovementEnabled();
			if (cameraMovementEnabled)
				ChangePlaneTransformation(PlaneCutTransformation::None);
			else
				ChangePlaneTransformation(planeCutTransformation);

			eventQueue.push(PlaneCutWindowEvent::ChangeCameraMovement);
		}
		if (IDC_PLANECUT_TRANSLATE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (cameraMovementEnabled)
			{
				ToggleCameraMovementEnabled();
				eventQueue.push(PlaneCutWindowEvent::ChangeCameraMovement);
			}
			ChangePlaneTransformation(PlaneCutTransformation::Translate);
				
			eventQueue.push(PlaneCutWindowEvent::ChangePlaneTransformation);
		}
		

	}
	void PlaneCutWindow::ToggleCameraMovementEnabled()
	{
		cameraMovementEnabled = !cameraMovementEnabled;

		if (!cameraMovementEnabled)
		{
			buttonLayoutMap[hButtonFreeCamera].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
			buttonLayoutMap[hButtonFreeCamera].SetFontSize(30);
		}
		else
		{
			buttonLayoutMap[hButtonFreeCamera].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Blue));
			buttonLayoutMap[hButtonFreeCamera].SetFontSize(30);
		}
		planeCutUi.Redraw();
	}

	void PlaneCutWindow::ChangePlaneTransformation(PlaneCutTransformation _transformationMode)
	{
		if (_transformationMode == PlaneCutTransformation::Translate)
		{
			buttonLayoutMap[buttonPlaneTranslation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Blue));
			buttonLayoutMap[buttonPlaneRotation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
			planeCutTransformation = _transformationMode;
		}
		else if (_transformationMode == PlaneCutTransformation::Rotate)
		{
			buttonLayoutMap[buttonPlaneTranslation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
			buttonLayoutMap[buttonPlaneRotation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Blue));
			planeCutTransformation = _transformationMode;
		}
		else
		{
			buttonLayoutMap[buttonPlaneTranslation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
			buttonLayoutMap[buttonPlaneRotation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		}
		
		planeCutUi.Redraw();
	}

	void PlaneCutWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"PlaneCutWindow::RESIZE");

		SubWindow::Resize(parentWidth, parentHeight);

		int controlX = (int)(0.02f*width);
		int controlWidth = (int)(0.96f*width);
		int buttonHeight = (int)(0.08f*height);

		MoveWindow(buttonCutReset, controlX, (int)(0.05f*height), controlWidth, (int)(0.2f*height), true);
		MoveWindow(buttonAxisX, controlX, (int)(0.4f*height), (int)(0.30f*width), (int)(0.075f*height), true);
		MoveWindow(buttonAxisY, (int)(0.35f*width), (int)(0.4f*height), (int)(0.30f*width), (int)(0.075f*height), true);
		MoveWindow(buttonAxisZ, (int)(0.68f*width), (int)(0.4f*height), (int)(0.30f*width), (int)(0.075f*height), true);
		MoveWindow(buttonPlaneRotation, controlX, (int)(0.50f*height), controlWidth/2 - controlX, (int)(0.1f*height), true);
		MoveWindow(buttonPlaneTranslation, controlWidth/2 + 2*controlX, (int)(0.50f*height), controlWidth/2 - controlX, (int)(0.1f*height), true);

		MoveWindow(hButtonFreeCamera, controlX, (int)(0.275f*height), controlWidth, (int)(0.1f*height), true);
		MoveWindow(buttonExecutePlaneCut, controlX, (int)(0.625f*height), controlWidth, (int)(0.1f*height), true);
		MoveWindow(buttonPlaneCutDone, controlX, (int)(0.75f*height), controlWidth, (int)(0.2f*height), true);
	}

	void PlaneCutWindow::ChangePlaneCutAxis(PlaneCutAxis _axis)
	{
		planeCutAxis = _axis;

		buttonLayoutMap[buttonAxisX].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonAxisY].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[buttonAxisZ].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		
		
		if (planeCutAxis == PlaneCutAxis::AxisX)
			buttonLayoutMap[buttonAxisX].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Blue));
		else if (planeCutAxis == PlaneCutAxis::AxisY)
			buttonLayoutMap[buttonAxisY].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Blue));
		else if (planeCutAxis == PlaneCutAxis::AxisZ)
			buttonLayoutMap[buttonAxisZ].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Blue));

		buttonLayoutMap[buttonAxisX].SetFontSize(40);
		buttonLayoutMap[buttonAxisY].SetFontSize(40);
		buttonLayoutMap[buttonAxisZ].SetFontSize(40);
		planeCutUi.Redraw();
		eventQueue.push(PlaneCutWindowEvent::ChangePlaneCutAxis);

	}

	void PlaneCutWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"ProcessingWindow::CleanUp()");
		planeCutUi.CleanUp();

		SubWindow::CleanUp();
	}
}