#include "HelpWindow.h"
#include "uxtheme.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include <sstream>
#include "StringConverter.h"
namespace InteractiveFusion {


	//PREPARE UI
	//map << shared_ptr<ButtonLayout>, HWND > layoutTry;
	HelpMessage currentHelpState;
	int currentIndex = 0;

	std::unordered_map<HelpMessage, std::vector<std::wstring>> helpTextMap;
	std::unordered_map<WindowState, HelpMessage> standardMessageMap;
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
		buttonLayoutMap[hButtonOk].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault));
		buttonLayoutMap[hButtonOk].SetFontSize(30);
		//hTextHelp = CreateWindowEx(0, L"STATIC", L"Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nunc ut tempus odio, congue molestie dolor. Aliquam sollicitudin venenatis dui, sed malesuada lectus iaculis vitae.", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER, 250, 50, 150, 50, windowHandle, (HMENU)IDC_HELP_TEXT, hInstance, 0);
		//hTextHelp = CreateWindowEx(0, L"STATIC", L"", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER, 250, 50, 150, 50, windowHandle, (HMENU)IDC_HELP_TEXT, hInstance, 0);
		
		standardMessageMap = std::unordered_map<WindowState, HelpMessage>();
		standardMessageMap.emplace(WindowState::Prepare, HelpMessage::PrepareHelp);
		standardMessageMap.emplace(WindowState::Scan, HelpMessage::ScanHelp);
		standardMessageMap.emplace(WindowState::PlaneSelection, HelpMessage::PlaneSelectionHelp);
		standardMessageMap.emplace(WindowState::Segmentation, HelpMessage::SegmentationHelp);
		standardMessageMap.emplace(WindowState::PlaneCut, HelpMessage::PlaneCutHelp);
		standardMessageMap.emplace(WindowState::Processing, HelpMessage::ProcessingHelp);
		standardMessageMap.emplace(WindowState::Interaction, HelpMessage::InteractionHelp);
		helpTextMap = std::unordered_map < HelpMessage, std::vector<std::wstring> >();
		//IntroHelp, PrepareHelp, ScanHelp, PlaneSelectionHelp, SegmentationHelp, PlaneCutHelp, ProcessingHelp, InteractionHelp
		helpTextMap.emplace(HelpMessage::PrepareHelp, std::vector<std::wstring>());
		helpTextMap.emplace(HelpMessage::ScanHelp, std::vector<std::wstring>());
		
		helpTextMap.emplace(HelpMessage::PlaneSelectionHelp, std::vector<std::wstring>());
		helpTextMap.emplace(HelpMessage::SegmentationHelp, std::vector<std::wstring>());
		helpTextMap.emplace(HelpMessage::PlaneCutHelp, std::vector<std::wstring>());
		helpTextMap.emplace(HelpMessage::ProcessingHelp, std::vector<std::wstring>());
		helpTextMap.emplace(HelpMessage::InteractionHelp, std::vector<std::wstring>());
		std::stringstream ss;

		//PREPARE
		ss << "This application let's you scan, virtually reconstruct, subsequently segment it into planes and moveable objects and process it by filling holes or removing unwanted components.";
		ss << "Afterwards, you will be able to navigate and manipulate the scene by physically moving around and interacting with the touchpad.";

		helpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "This is the PREPARE screen. Here, you can adjust the size of the area (width x height x depth) that you want to scan by adjusting the slider handle.\r\n ";
		ss << "Press ''START'' when you are ready to scan your scene. If you don't want any more help messages, just uncheck the ''Show Help'' checkbox.";

		helpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//SCAN
		ss << "After a short countdown, you will be taken to the SCAN screen and the scanning process will start. ";
		ss << "But before we start, try to make the following final preparations.";
		helpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		
		ss << "Be sure to position the tablet/camera about one meter away from the object you want to scan while facing in it's direction. ";
		ss << "For better results, it's also advised to hold the camera in an angle that is parallel to the ground when the scan starts.";

		helpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "While scanning, please move slowly around the object/scene and try to capture it from as many different angles as possible.";
ss << "If you don't get it right the first time, you can always press the ''RESET''-Button on the left to restart the scanning process. If you are finished, press ''DONE''.";

		helpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();


		//PLANE SELECTION
		ss << "You are now working with the virtual reconstruction of your scan in the first part of the SEGMENT screen.";
		ss << "Here we will try to find major static planes in your scene like the ground, walls and  or the ceiling which will remain immovable during interaction.";
		
		helpTextMap[HelpMessage::PlaneSelectionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();
		
		ss << "One after another, possible planes that have been found in your scene will be highlighted in red.If it is indeed a static part of your scene, press ''YES'', otherwise press ''NO''.";
		ss << "If the highlighted area is too thick or too thin(i.e.only a part of the wall is highlighted), change the wall thickness on the bottom of the screen.";
		helpTextMap[HelpMessage::PlaneSelectionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();
		ss << "If the highlighted area has gaps, decrease smoothness; if it is not flat enough(i.e.contains other objects), increase it.";
		ss << "Note that the subsequent moveable object segmentation heavily relies on the presence of at least one static ground plane.";
		helpTextMap[HelpMessage::PlaneSelectionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//SEGMENTATION
		ss << "Segmentation placeholder\r\n\r\n";
		ss << "Segmentation placeholder\r\n\r\n";

		helpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//PLANECUT
		ss << "PlaneCutHelp placeholder\r\n\r\n";
		ss << "PlaneCutHelp placeholder\r\n\r\n";

		helpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//PROCESSING
		ss << "ProcessingHelp placeholder\r\n\r\n";
		ss << "ProcessingHelp placeholder\r\n\r\n";

		helpTextMap[HelpMessage::ProcessingHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//INTERACTION
		ss << "InteractionHelp placeholder\r\n\r\n";
		ss << "InteractionHelp placeholder\r\n\r\n";

		helpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		currentHelpState = HelpMessage::PlaneSelectionHelp;
		currentIndex = 0;
		hTextHelp = CreateWindowEx(0, L"STATIC", helpTextMap[currentHelpState][currentIndex].c_str(), WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_CENTER, 250, 50, 150, 50, windowHandle, (HMENU)IDC_HELP_TEXT, hInstance, 0);

		SendMessage(hTextHelp, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		

		UpdateWindow(windowHandle);
	}




	LRESULT CALLBACK HelpWindow::SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);;
	}

	void HelpWindow::HandleEvents(MainWindow& _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case HelpWindowEvent::OK:
				DebugUtility::DbgOut(L"HelpWindow::HandleEvents::OK");
				//_parentWindow.ChangeScanVolumeSize(voxelsPerMeter);
				//_parentWindow.ChangeState(Scan);
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
			if (currentIndex < helpTextMap[currentHelpState].size() - 1)
				NextHelpMessage();
			else
			{
				Hide();
				eventQueue.push(HelpWindowEvent::OK);
			}
		}
	}

	void HelpWindow::NextHelpMessage()
	{
		currentIndex++;
		SetDlgItemText(windowHandle, IDC_HELP_TEXT, helpTextMap[currentHelpState][currentIndex].c_str());
	}

	void HelpWindow::Show()
	{
		SetWindowPos(windowHandle, HWND_TOP, 0, 0, 0, 0, 0);
		SetForegroundWindow(windowHandle);
		SubWindow::Show();
		
		
	}

	void HelpWindow::Hide()
	{
		SubWindow::Hide();
	}

	void HelpWindow::SetHelpState(HelpMessage state)
	{
		currentHelpState = state;
		currentIndex = 0;
		SetDlgItemText(windowHandle, IDC_HELP_TEXT, helpTextMap[currentHelpState][currentIndex].c_str());
	}

	void HelpWindow::SetDefaultMessage(WindowState state)
	{
		auto it = standardMessageMap.find(state);
		if (it != standardMessageMap.end())
		{
			SetHelpState(it->second);
		}
	}

	void HelpWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"HelpWindow::RESIZE");

		SubWindow::Resize(parentWidth, parentHeight);

		MoveWindow(hTextHelp, (int)(0.05f*width), (int)(0.12 * height), (int)(0.9f*width), height - (int)(0.12*height) - 60, true);
		MoveWindow(hButtonOk, (int)(width/2 - 50), (int)(0.9*height-50), 100, 50, true);

	}

	void HelpWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"PrepareWindow::CleanUp()");

		SubWindow::CleanUp();
	}
}