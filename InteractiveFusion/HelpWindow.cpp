#include "HelpWindow.h"
#include "uxtheme.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include <sstream>
#include "StringConverter.h"
#include "Logger.h"
namespace InteractiveFusion {


	//PREPARE UI
	//map << shared_ptr<ButtonLayout>, HWND > layoutTry;
	HelpMessage currentHelpState;
	int currentIndex = 0;
	std::unordered_map<ScenarioType, std::unordered_map<HelpMessage, std::vector<std::wstring>>> metaMap;
	std::unordered_map<HelpMessage, std::vector<std::wstring>> helpTextMap;
	std::unordered_map<HelpMessage, std::vector<std::wstring>> bowlingHelpTextMap;
	std::unordered_map<HelpMessage, std::vector<std::wstring>> basicHelpTextMap;

	std::unordered_map<WindowState, HelpMessage> standardMessageMap;
	HWND hButtonOk;
	HWND hTextHelp;
	HWND hMessageCount;
	int lineCount = 0;
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

		bowlingHelpTextMap = std::unordered_map < HelpMessage, std::vector<std::wstring> >();
		bowlingHelpTextMap.emplace(HelpMessage::PrepareHelp, std::vector<std::wstring>());
		bowlingHelpTextMap.emplace(HelpMessage::ScanHelp, std::vector<std::wstring>());

		bowlingHelpTextMap.emplace(HelpMessage::PlaneSelectionHelp, std::vector<std::wstring>());
		bowlingHelpTextMap.emplace(HelpMessage::SegmentationHelp, std::vector<std::wstring>());
		bowlingHelpTextMap.emplace(HelpMessage::PlaneCutHelp, std::vector<std::wstring>());
		bowlingHelpTextMap.emplace(HelpMessage::ProcessingHelp, std::vector<std::wstring>());
		bowlingHelpTextMap.emplace(HelpMessage::InteractionHelp, std::vector<std::wstring>());

		basicHelpTextMap = std::unordered_map < HelpMessage, std::vector<std::wstring> >();
		basicHelpTextMap.emplace(HelpMessage::PrepareHelp, std::vector<std::wstring>());
		basicHelpTextMap.emplace(HelpMessage::ScanHelp, std::vector<std::wstring>());

		basicHelpTextMap.emplace(HelpMessage::PlaneSelectionHelp, std::vector<std::wstring>());
		basicHelpTextMap.emplace(HelpMessage::SegmentationHelp, std::vector<std::wstring>());
		basicHelpTextMap.emplace(HelpMessage::PlaneCutHelp, std::vector<std::wstring>());
		basicHelpTextMap.emplace(HelpMessage::ProcessingHelp, std::vector<std::wstring>());
		basicHelpTextMap.emplace(HelpMessage::InteractionHelp, std::vector<std::wstring>());

		std::stringstream ss;


		ss << "In this scenario, your goal is to prepare a bowling setup with about ten bowling pins.\r\n\r\n";
		ss << "To achieve that, you have to do the following:\r\n\r\n";
		ss << "1. Scan the single bowling pin on the highlighted (yellow) area.\r\n\r\n";
		ss << "2. Segment the scene in order to separate the pin from the floor.\r\n\r\n";
		ss << "3. Remove unnecessary parts of the scene and fill holes.\r\n\r\n";
		ss << "4. Duplicate and enlargen the pin and place them in a custom bowling setup.\r\n\r\n\r\n";
		ss << "The application will display help messages and give hints at the beginning of every stage.\r\n\r\n";
		bowlingHelpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//PREPARE BASIC 
		ss << "In this scenario, you have to perform the following steps:\r\n\r\n";
		ss << "1. Scan the highlighted (yellow) area with all the differently colored items placed on it.\r\n\r\n"; 
		ss << "2. Separate the ground plane from movable objects.\r\n\r\n";
		ss << "3. Use object segmentation to roughly seperate different object groups.\r\n\r\n"; 
		ss << "4. Use plane cut segmentation to separate objects that are still connected as cleanly as possible.\r\n\r\n";
		ss << "The application will display help messages and give hints at the beginning of every stage.\r\n\r\n";
		basicHelpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();


		//PREPARE
		ss << "This application let's you scan and virtually reconstruct a three-dimensional scene, subsequently segment it into planes and moveable objects and process it by filling holes or removing unwanted components.\r\n\r\n";
		ss << "Afterwards, you will be able to navigate and manipulate the scene by physically moving around and interacting with the touchscreen.\r\n\r\n";
		bowlingHelpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		helpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "This is the PREPARE screen.\r\n\r\n";
		ss << "Select the right size for the space you want to scan. The smaller the space, the higher the quality of the reconstruction.\r\n\r\n";
		ss << "If you don't want any more automatic help messages, just uncheck the ''Show Help'' checkbox. You can always look at the currently available help by selecting the ''?'' in the top left corner.\r\n\r\n";
		ss << "Press ''START'' when you are ready to scan your scene. ";
		bowlingHelpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		helpTextMap[HelpMessage::PrepareHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//SCAN

		ss << "Here are some general instructions for the scanning procedure that is about to start.\r\n\r\n";
		ss << "At the beginning of the scan:\r\n\r\n";
		ss << "1. Position the tablet about 50cm away from the scene.\r\n\r\n";
		ss << "2. Hold the tablet perpendicular to the floor and point it at the center of the scene.\r\n\r\n";
		ss << "During the scan:\r\n\r\n";
		ss << "1. Keep the camera at a minimum distance of 50cm from any physical objects to stabilize positional tracking.\r\n\r\n";
		ss << "2. Move slowly around the object/scene and try to capture it from as many different angles as possible to fill all the gaps in it's reconstructed surface.";
		bowlingHelpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "You can reset the scan by pressing the ''RESET'' button. Make sure to revert to your starting position before resetting. \r\n\r\n";
		ss << "When you are finished, press ''DONE'' to proceed to the next screen.\r\n\r\n";
		ss << "After a short countdown, the scanning process will start.";
		bowlingHelpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::ScanHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();


		//PLANE SELECTION
		ss << "This is the first part of the SEGMENT screen.\r\n";
		ss << "You are now working with the virtual reconstruction of your scan.\r\n\r\n";
		ss << "The application will propose major planes like the floor and walls as static to keep them immovable during the scene manipulation. Select ''YES'' only if the area highlighted in red contains the entire static plane and as little of the other objects as possible. Try to improve the highlighted area by adjusting thickness and smoothness. If you cannot achieve the desired results, press ''NO'' to get a new suggestion.\r\n\r\n";
		ss << "You can orbit the camera around freely by sliding your finger over the scene. You can also zoom in and out by scrolling up and down.\r\n\r\n";
		ss << "Note that you should try to at least find the floor and major walls, if present, as the subsequent segmentation of movable objects heavily relies on the presence of at least one static ground plane.";
		bowlingHelpTextMap[HelpMessage::PlaneSelectionHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::PlaneSelectionHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::PlaneSelectionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();


		//SEGMENTATION
		ss << "This is the second part of the SEGMENT screen.\r\n\r\n";
		ss << "The non-static parts will now be segmented into distinct movable objects.\r\n\r\n";
		ss << "You can choose between two different methods of segmentation (Euclidean and Region Growth Segmentation) and adjust respective parameters on the right.\r\n\r\n";
		bowlingHelpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "Right now, a segmentation preview using Euclidean Segmentation with default parameters is shown. For most purposes, this preset is the best choice.\r\n\r\n";
		ss << "In a few cases, however, you might need to adjust the ""TOLERANCE"" (the maximum euclidean distance between two unconnected parts wherein they are still considered to be part of the same object).\r\n\r\n";
		ss << "Press ''PREVIEW'' to get a preview of a segmentation with your currently selected parameters, where each distinct object is highlighted in a different color.\r\n\r\n";
		ss << "Press ''DONE'' to continue to the next stage.\r\n\r\n";
		ss << "(NOTE: You can reset the camera position by pressing the button with the camera icon on the lower right corner.)";
		bowlingHelpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::SegmentationHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//PLANECUT
		ss << "This is the last part of the SEGMENT screen.\r\n\r\n";
		ss << "If there are still objects that have not yet been separated from each other, you can refine the segmentation by cutting them in two with a plane. You can only cut movable objects, not static planes.\r\n\r\n";
		ss << "Select an object that you want to cut by simply clicking on it. You can then drag the plane to reposition it.\r\n\r\n";
		bowlingHelpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "By switching to the ''Rotate'' mode on the right side, you can adjust the plane's rotation up to a certain point. The general orientation of the plane can be changed by selecting it's perpendicular axis (''X'', ''Y'' or ''Z'').\r\n\r\n";
		ss << "Whenever you move the plane, you will see an updated segmentation preview of the currently selected object, where green and blue regions represent the two resulting segments.\r\n\r\n";
		ss << "Press ''Cut'' to perform a plane cut. You can also switch in and out of free camera movement by clicking on ''Free Camera'', although you wont be able to position the plane while navigating.\r\n\r\n";
		ss << "Press ''DONE'' to proceed to the next step.";
		bowlingHelpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "As a reminder: In this scenario, your goal is to seperate physically connected differently colored boxes.\r\n\r\n";
		ss << "Use a plane parallel to the floor to cut boxes that are standing on top of each other into two distinct objects. It doesn't have to be perfect, but try to seperate them as cleanly as possible.\r\n\r\n";
		basicHelpTextMap[HelpMessage::PlaneCutHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//PROCESSING
		ss << "This is the PROCESS screen.\r\n\r\n";
		ss << "During segmentation, the non-visible touching surfaces of two separated objects result in holes in the reconstruction. It is recommended to fill these holes as well as those that might have been left open during the scanning process. This can be done by adjusting the ''Hole Size'' and selecting ''Fill Holes''.\r\n\r\n";
		ss << "It is also possible to remove components that contain less than a certain number of vertices by adjusting the ''Component Size'' and clicking ''Remove Components'' on the right side.\r\n\r\n";
		ss << "You can also select a specific segment in your scene (which will be highlighted in red) and perform hole filling to only fill holes in that object.\r\n\r\n";
		ss << "Press ''DONE'' to proceed to the next stage.";
		bowlingHelpTextMap[HelpMessage::ProcessingHelp].push_back(StringConverter::StringToWString(ss.str()));
		basicHelpTextMap[HelpMessage::ProcessingHelp].push_back(StringConverter::StringToWString(ss.str()));

		helpTextMap[HelpMessage::ProcessingHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//INTERACTION
		ss << "This is the MANIPULATE screen.\r\n\r\n";
		ss << "You can navigate the scene by using the tablet with the Kinect camera as a ''window'' into the virtual world. This means that you can physically move around in the scene by pointing the tablet at previously scanned real objects.\r\n\r\n";
		ss << "If the system has problems tracking your perspective, point the tablet away from the scene for a second, before focusing on the reconstruction again. \r\n\r\n";
		ss << "You are also able to pick movable objects up by clicking on them, carry them around by keeping your finger on the touch screen and moving and reposition them by dropping them in another place.\r\n\r\n";
		ss << "By switching into ''Transform'' mode on the right side, you can rotate an object by dragging it in the desired rotational direction, as well as scale it by using scrolling touch input.\r\n\r\n";
		basicHelpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		bowlingHelpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		helpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		ss << "When ''Duplicate' is active, you will always pick up a duplicate of an object instead of the real one, so you can place multiple models of the same object in the scene.\r\n\r\n";
		ss << "To delete an object, drag and drop it onto the trash can in the lower right corner.\r\n\r\n";
		ss << "You can always switch to a free camera to review your changes by clicking ''Free Camera'', although you wont be able to manipulate the scene while doing this.\r\n\r\n";
		ss << "When you are satisfied with your scene, click on ''Export'' to save your final creation as a 3D model that can be imported in other applications for further use.\r\n\r\n";
		basicHelpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		bowlingHelpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		helpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//SCENARIO BOWLING

		ss << "As a reminder: In this scenario, your goal is to prepare a bowling setup with about ten bowling pins.\r\n\r\n";
		ss << "To achieve that, you should first remove everything but the bowling pin and the ground floor from the scene. After that, duplicate the pin as often as you like and place them somewhere in the marked area.\r\n\r\n";
		ss << "Try to make (some of) the pins bigger by scaling them with 'Transform'. Make sure that all the pins are standing upright and don't touch each other. They don't have to be uniformly scaled. You also don't have to place them in a typical triangle setup.\r\n\r\n";
		bowlingHelpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		//SCENARIO BASIC

		ss << "As a reminder: In this scenario, your goal was to segment the scene into distinct objects. You don't have to manipulate the scene, but can instead press ''EXPORT'' to end this scenario.\r\n\r\n";
		ss << "However, if you want to edit the scene, you can move, transform and/or duplicate items. Just make sure to have at least one copy of every original object in the scene when pressing ''EXPORT''.\r\n\r\n";
		basicHelpTextMap[HelpMessage::InteractionHelp].push_back(StringConverter::StringToWString(ss.str()));
		ss.str(std::string());
		ss.clear();

		metaMap = std::unordered_map<ScenarioType, std::unordered_map<HelpMessage, std::vector<std::wstring>>>();
		metaMap.emplace(ScenarioType::None, helpTextMap);
		metaMap.emplace(ScenarioType::Bowling, bowlingHelpTextMap);
		metaMap.emplace(ScenarioType::Basic, basicHelpTextMap);


		currentHelpState = HelpMessage::PlaneSelectionHelp;
		currentIndex = 0;
		hTextHelp = CreateWindowEx(0, L"STATIC", helpTextMap[currentHelpState][currentIndex].c_str(), WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_LEFT, 250, 50, 150, 50, windowHandle, (HMENU)IDC_HELP_TEXT, hInstance, 0);

		SendMessage(hTextHelp, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		
		hMessageCount = CreateWindowEx(0, L"STATIC", L"1/2", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | SS_LEFT, 250, 50, 150, 50, windowHandle, (HMENU)IDC_HELP_COUNT, hInstance, 0);

		SendMessage(hMessageCount, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

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
		if (IDC_HELP_BUTTON_OK == LOWORD(wParam))
		{
			if (currentIndex < metaMap[scenarioType][currentHelpState].size() - 1)
				NextHelpMessage();
			else
			{
				Logger::WriteToLog(L"Finished showing help messages", Logger::info);
				Hide();
				eventQueue.push(HelpWindowEvent::OK);
			}
		}
	}

	void HelpWindow::UpdateMessageCount()
	{
		std::wstring countString = std::to_wstring(currentIndex + 1) + L"/" + std::to_wstring(metaMap[scenarioType][currentHelpState].size());
		SetDlgItemText(windowHandle, IDC_HELP_COUNT, countString.c_str());
	}

	void HelpWindow::NextHelpMessage()
	{
		currentIndex++;
		Logger::WriteToLog(L"Showing help message " + std::to_wstring(currentIndex) + L" of " + std::to_wstring(metaMap[scenarioType][currentHelpState].size()), Logger::info);
		UpdateLineCount();
		SetDlgItemText(windowHandle, IDC_HELP_TEXT, metaMap[scenarioType][currentHelpState][currentIndex].c_str());
		UpdateMessageCount();
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
		currentIndex = -1;
		NextHelpMessage();
	}

	void HelpWindow::SetDefaultMessage(WindowState state)
	{
		auto it = standardMessageMap.find(state);
		if (it != standardMessageMap.end())
		{
			SetHelpState(it->second);
		}
	}

	void HelpWindow::SetScenarioType(ScenarioType type)
	{
		scenarioType = type;
	}

	void HelpWindow::UpdateLineCount()
	{
		/*lineCount = std::ceil((int)helpTextMap[currentHelpState][currentIndex].length() / 100);
		return;
		std::wstring sep = L"\r\n";
		std::wstring::size_type found = helpTextMap[currentHelpState][currentIndex].find(sep);
		lineCount = 0;
		while (found != std::wstring::npos)
		{
			found = helpTextMap[currentHelpState][currentIndex].find(sep, found + 1);
			lineCount++;
		}
		RECT rRect;
		GetClientRect(parentHandle, &rRect);
		Resize(rRect.right, rRect.bottom);
		DebugUtility::DbgOut(L"LineCount::", lineCount);*/
	}

	void HelpWindow::Resize(int parentWidth, int parentHeight)
	{

		SubWindow::Resize(parentWidth, parentHeight);

		//int textHeight = 2*(lineCount*(int)(0.06f*height));
		//MoveWindow(hTextHelp, (int)(0.05f*width), (int)(0.5f*height) - textHeight / 2, (int)(0.9f*width), textHeight, true);

		MoveWindow(hTextHelp, (int)(0.05f*width), (int)(0.05 * height), (int)(0.9f*width), height - (int)(0.12*height) - 60, true);
		MoveWindow(hButtonOk, (int)(width/2 - 50), (int)(0.95*height-50), 100, 50, true);
		MoveWindow(hMessageCount, (int)(width-60), (int)(height-40), 50, 30, true);

	}

	void HelpWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"PrepareWindow::CleanUp()");

		SubWindow::CleanUp();
	}
}