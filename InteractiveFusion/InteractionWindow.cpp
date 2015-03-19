#include "InteractionWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"
namespace InteractiveFusion {

	//INTERACTION
	GUIContainer  interactionSubUi;
	HWND buttonExport, buttonReset;
	HWND buttonDuplicate;
	HWND buttonTransformation;
	HWND buttonFreeCamera;


	//BITMAPS
	HBITMAP bmpTrash;
	HBITMAP bmpTrashMask;

	HICON deleteIcon;

	InteractionWindow::InteractionWindow()
	{
	}


	InteractionWindow::~InteractionWindow()
	{
	}

	void InteractionWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		buttonExport = CreateWindowEx(0, L"BUTTON", L"Export", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_INTERACTION_BUTTON_EXPORT, hInstance, 0);

		buttonLayoutMap.emplace(buttonExport, ButtonLayout());
		buttonLayoutMap[buttonExport].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Green));
		buttonDuplicate = CreateWindowEx(0, L"BUTTON", L"Duplicate", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_INTERACTION_BUTTON_DUPLICATE, hInstance, 0);

		buttonLayoutMap.emplace(buttonDuplicate, ButtonLayout());
		buttonLayoutMap[buttonDuplicate].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonReset = CreateWindowEx(0, L"BUTTON", L"Reset", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_INTERACTION_BUTTON_RESET, hInstance, 0);

		buttonLayoutMap.emplace(buttonReset, ButtonLayout());
		buttonLayoutMap[buttonReset].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Red));
		deleteIcon = (HICON)LoadImage(hInstance, MAKEINTRESOURCE(IDI_INTERACTION_TRASH), IMAGE_ICON, 100, 100, NULL);

		buttonTransformation = CreateWindowEx(0, L"BUTTON", L"Transform", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_INTERACTION_BUTTON_TRANSFORMATION, hInstance, 0);

		buttonLayoutMap.emplace(buttonTransformation, ButtonLayout());
		buttonLayoutMap[buttonTransformation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonFreeCamera = CreateWindowEx(0, L"BUTTON", L"Free Camera", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_OWNERDRAW | BS_MULTILINE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_INTERACTION_BUTTON_FREE_CAMERA, hInstance, 0);

		buttonLayoutMap.emplace(buttonFreeCamera, ButtonLayout());
		buttonLayoutMap[buttonFreeCamera].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonFreeCamera].SetFontSize(30);

		interactionSubUi.Add(buttonTransformation);
		interactionSubUi.Add(buttonFreeCamera);
		interactionSubUi.Add(buttonExport);
		interactionSubUi.Add(buttonDuplicate);

		interactionSubUi.Add(buttonReset);
	}


	LRESULT CALLBACK InteractionWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		return SubWindow::SubWindowProc(windowHandle, message, wParam, lParam);
	}

	void InteractionWindow::HandleEvents(MainWindow* _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case InteractionWindowEvent::StateChange:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::StateChange");
				_parentWindow->ChangeState(Scan);
				break;
			case InteractionWindowEvent::ExportModel:
				DebugUtility::DbgOut(L"PrepareWindow::HandleEvents::ExportModel");
				_parentWindow->ExportModel();
				break;
			case InteractionWindowEvent::ChangeCameraMode:
				_parentWindow->SetCameraMode(cameraMode);
				break;
			case InteractionWindowEvent::ChangeManipulationMode:
				_parentWindow->ChangeInteractionMode(interactionMode);
				//do stuff
				break;
			case InteractionWindowEvent::Reset:
				_parentWindow->ResetModel();
				//do stuff
				break;
			}

			eventQueue.pop();
		}
	}

	void InteractionWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_INTERACTION_BUTTON_TRANSFORMATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (cameraMode == Sensor)
			{
				if (interactionMode == Transformation)
					ChangeInteractionMode(None);
				else
					ChangeInteractionMode(Transformation);
			}
		}
		if (IDC_INTERACTION_BUTTON_FREE_CAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (cameraMode == Free)
			{
				buttonLayoutMap[buttonFreeCamera].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
				buttonLayoutMap[buttonFreeCamera].SetFontSize(30);
				cameraMode = Sensor;
			}
			else
			{
				if (interactionMode != None)
					ChangeInteractionMode(None);
				buttonLayoutMap[buttonFreeCamera].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));
				buttonLayoutMap[buttonFreeCamera].SetFontSize(30);
				cameraMode = Free;
			}
			eventQueue.push(InteractionWindowEvent::ChangeCameraMode);
		}
		if (IDC_INTERACTION_BUTTON_RESET == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			
			//glSelector.Unselect();
			//meshHelper.ResetAll();
			eventQueue.push(InteractionWindowEvent::Reset);
		}
		if (IDC_INTERACTION_BUTTON_DUPLICATE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (cameraMode == Sensor)
			{
				if (interactionMode == Duplication)
					ChangeInteractionMode(None);
				else
					ChangeInteractionMode(Duplication);
			}
		}
		if (IDC_INTERACTION_BUTTON_EXPORT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"InteractionWindow::ProcessUI::Export");
			//appStatus.SetStatusBarMessage(L"Combining and exporting all meshes...");

			/*if (glSelector.selectedIndex != -1)
				meshHelper.Export(glSelector.selectedIndex);
			else
				meshHelper.ExportForUnity();*/
			//meshHelper.CombineAndExport();
			eventQueue.push(InteractionWindowEvent::ExportModel);

		}
	}

	void InteractionWindow::ChangeInteractionMode(InteractionMode _interactionMode)
	{
		interactionMode = _interactionMode;

		if (interactionMode == Transformation)
		{
			buttonLayoutMap[buttonTransformation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));
			buttonLayoutMap[buttonDuplicate].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		}
		else if (interactionMode == Duplication)
		{
			buttonLayoutMap[buttonTransformation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
			buttonLayoutMap[buttonDuplicate].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Blue));
		}
		else if (interactionMode == None)
		{
			buttonLayoutMap[buttonTransformation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
			buttonLayoutMap[buttonDuplicate].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		}

		interactionSubUi.Redraw();
		eventQueue.push(InteractionWindowEvent::ChangeManipulationMode);
	}


	void InteractionWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"ProcessingWindow::RESIZE");
		//RECT rRect;
		//GetClientRect(parentHandle, &rRect);

		SubWindow::Resize(parentWidth, parentHeight);

		int controlX = (int)(0.02f*width);
		int controlWidth = (int)(0.96f*width);
		int buttonHeight = (int)(0.08f*height);

		MoveWindow(buttonReset, controlX, (int)(0.05f*height), controlWidth, (int)(0.2f*height), true);
		MoveWindow(buttonFreeCamera, controlX, (int)(0.350f*height), controlWidth, (int)(0.15f*height), true);
		MoveWindow(buttonDuplicate, controlWidth / 2 + controlX, (int)(0.50f*height), controlWidth / 2, (int)(0.15f*height), true);
		MoveWindow(buttonTransformation, controlX, (int)(0.50f*height), controlWidth/2, (int)(0.15f*height), true);

		MoveWindow(buttonExport, controlX, (int)(0.75f*height), controlWidth, (int)(0.2f*height), true);
	}

	void InteractionWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"InteractionWindow::CleanUp()");
		interactionSubUi.CleanUp();

		SubWindow::CleanUp();
	}
}