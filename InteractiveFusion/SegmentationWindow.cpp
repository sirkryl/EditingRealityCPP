#include "SegmentationWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"
#include "SegmentationParams.h"

#include <sstream>
#include <iomanip>
using namespace std;

namespace InteractiveFusion {

	

	//SEGMENTATION PREVIEW
	GUIContainer generalSegmentationUi;
	HWND buttonSegmentationFinish;
	HWND buttonSegmentationBegin;

	//EUCLIDEAN UI
	GUIContainer euclideanSegmentationUi;
	HWND textClusterToleranceLabel, textClusterTolerance;

	//REGION GROWTH UI
	GUIContainer  regionSegmentationUi;
	HWND buttonEuclideanSegmentation, buttonRegionGrowthSegmentation;
	HWND textRGSmoothnessLabel, textRGSmoothness;
	HWND textRGCurvatureLabel, textRGCurvature;
	HWND textRGNeighborsLabel, textRGNeighbors;
	HWND textRGKSearchLabel, textRGKSearch;

	EuclideanSegmentationParams euclideanParams;
	RegionGrowthSegmentationParams regionGrowthParams;

	ObjectSegmentationType selectedSegmentationType = Euclidean;

	SegmentationWindow::SegmentationWindow()
	{
	}


	SegmentationWindow::~SegmentationWindow()
	{
	}

	void SegmentationWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		euclideanParams.clusterTolerance = 0.02f;

		regionGrowthParams.kSearchValue = 20;
		regionGrowthParams.numberOfNeighbors = 20;
		regionGrowthParams.smoothnessThreshold = 100;
		regionGrowthParams.curvatureThreshold = 10;

		buttonSegmentationFinish = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_BUTTON_DONE, hInstance, 0);

		buttonLayoutMap.emplace(buttonSegmentationFinish, ButtonLayout());
		buttonLayoutMap[buttonSegmentationFinish].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(Green));
		buttonSegmentationBegin = CreateWindowEx(0, L"Button", L"SEGMENT", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_BUTTON_UPDATE, hInstance, 0);

		
		buttonLayoutMap.emplace(buttonSegmentationBegin, ButtonLayout());
		buttonLayoutMap[buttonSegmentationBegin].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonLayoutMap[buttonSegmentationBegin].SetFontSize(30);
		textClusterTolerance = CreateWindowEx(0, L"STATIC", std::to_wstring((int)(euclideanParams.clusterTolerance*1000.0f)).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_E_TEXT_CLUSTERTOLERANCE, hInstance, 0);

		SendMessage(textClusterTolerance, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textClusterToleranceLabel = CreateWindowEx(0, L"STATIC", L"TOLERANCE", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_E_TEXT_CLUSTERTOLERANCE_LABEL, hInstance, 0);

		SendMessage(textClusterToleranceLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		buttonRegionGrowthSegmentation = CreateWindowEx(0, L"Button", L"RG", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_REGIONGROWTH, hInstance, 0);

		buttonLayoutMap.emplace(buttonRegionGrowthSegmentation, ButtonLayout());
		buttonLayoutMap[buttonRegionGrowthSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
		//buttonLayoutMap[buttonRegionGrowthSegmentation].SetFontSize(30);
		buttonEuclideanSegmentation = CreateWindowEx(0, L"Button", L"EUCLIDEAN", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_EUCLIDEAN, hInstance, 0);

		buttonLayoutMap.emplace(buttonEuclideanSegmentation, ButtonLayout());
		buttonLayoutMap[buttonEuclideanSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));

		textRGSmoothness = CreateWindowEx(0, L"STATIC", std::to_wstring((int)regionGrowthParams.smoothnessThreshold).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_SMOOTHNESS, hInstance, 0);

		SendMessage(textRGSmoothness, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGSmoothnessLabel = CreateWindowEx(0, L"STATIC", L"SMOOTHNESS", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_SMOOTHNESS_LABEL, hInstance, 0);

		SendMessage(textRGSmoothnessLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textRGCurvature = CreateWindowEx(0, L"STATIC", std::to_wstring((int)(regionGrowthParams.curvatureThreshold*10.0f)).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_CURVATURE, hInstance, 0);
		SendMessage(textRGCurvature, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGCurvatureLabel = CreateWindowEx(0, L"STATIC", L"CURVATURE", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_CURVATURE_LABEL, hInstance, 0);

		SendMessage(textRGCurvatureLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textRGNeighbors = CreateWindowEx(0, L"STATIC", std::to_wstring(regionGrowthParams.numberOfNeighbors).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_NEIGHBORS, hInstance, 0);

		SendMessage(textRGNeighbors, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGNeighborsLabel = CreateWindowEx(0, L"STATIC", L"NEIGHBORS", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_NEIGHBORS_LABEL, hInstance, 0);

		SendMessage(textRGNeighborsLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textRGKSearch = CreateWindowEx(0, L"STATIC", std::to_wstring(regionGrowthParams.minComponentSize).c_str(), WS_CHILD | WS_VISIBLE | SS_RIGHT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_SIZE, hInstance, 0);

		SendMessage(textRGKSearch, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGKSearchLabel = CreateWindowEx(0, L"STATIC", L"MIN SIZE", WS_CHILD | WS_VISIBLE | SS_LEFT | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_KSEARCH_LABEL, hInstance, 0);

		SendMessage(textRGKSearchLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		sliderMap.emplace(Tolerance, ButtonSlider());
		sliderMap.emplace(Smoothness, ButtonSlider());
		sliderMap.emplace(Curvature, ButtonSlider());
		sliderMap.emplace(Neighbors, ButtonSlider());
		sliderMap.emplace(MinSize, ButtonSlider());

		for (auto& slider : sliderMap)
		{
			slider.second.Initialize(windowHandle, _hInstance);
			slider.second.SetLayout(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault), StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
			slider.second.SetLimits(1, 100);
			slider.second.SetStep(1);
		}
		
		sliderMap[Smoothness].SetLimits(1, 200);
		sliderMap[Curvature].SetLimits(1, 200);
		sliderMap[MinSize].SetLimits(100, 10000);

		sliderMap[Tolerance].SetValue((int)(euclideanParams.clusterTolerance*1000.0f));
		sliderMap[Smoothness].SetValue((int)regionGrowthParams.smoothnessThreshold);
		sliderMap[Curvature].SetValue((int)(regionGrowthParams.curvatureThreshold*10.0f));
		sliderMap[Neighbors].SetValue(regionGrowthParams.numberOfNeighbors);
		sliderMap[MinSize].SetValue(regionGrowthParams.minComponentSize);
		

		generalSegmentationUi.Add(buttonSegmentationFinish);
		generalSegmentationUi.Add(buttonSegmentationBegin);
		generalSegmentationUi.Add(buttonRegionGrowthSegmentation);
		generalSegmentationUi.Add(buttonEuclideanSegmentation);


		regionSegmentationUi.Add(sliderMap[Smoothness]);
		regionSegmentationUi.Add(sliderMap[Curvature]);
		regionSegmentationUi.Add(sliderMap[Neighbors]);
		regionSegmentationUi.Add(sliderMap[MinSize]);

		regionSegmentationUi.Add(textRGSmoothness);
		regionSegmentationUi.Add(textRGSmoothnessLabel);

		regionSegmentationUi.Add(textRGCurvature);
		regionSegmentationUi.Add(textRGCurvatureLabel);
		regionSegmentationUi.Add(textRGNeighbors);
		regionSegmentationUi.Add(textRGNeighborsLabel);
		regionSegmentationUi.Add(textRGKSearch);
		regionSegmentationUi.Add(textRGKSearchLabel);

		euclideanSegmentationUi.Add(sliderMap[Tolerance]);
		euclideanSegmentationUi.Add(textClusterTolerance);
		euclideanSegmentationUi.Add(textClusterToleranceLabel);

		selectedSegmentationType = Euclidean;
		UpdateSegmentationPreviewValues();
		UpdateSegmentationUI();
	}


	void SegmentationWindow::UpdateSegmentationUI()
	{
		switch (selectedSegmentationType)
		{
		case Euclidean:
			regionSegmentationUi.Hide();
			buttonLayoutMap[buttonEuclideanSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ActiveMode));
			buttonLayoutMap[buttonRegionGrowthSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
			euclideanSegmentationUi.Show();
			break;
		case RegionGrowth:
			euclideanSegmentationUi.Hide();
			buttonLayoutMap[buttonRegionGrowthSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ActiveMode));
			buttonLayoutMap[buttonEuclideanSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
			regionSegmentationUi.Show();
			break;
		}
		generalSegmentationUi.Redraw();
	}

	void SegmentationWindow::UpdateSegmentationPreviewValues()
	{
		if (euclideanParams.clusterTolerance != (float)sliderMap[Tolerance].GetValue() / 1000.0f)
		{
			euclideanParams.clusterTolerance = (float)sliderMap[Tolerance].GetValue() / 1000.0f;
			wstring clusterToleranceString = to_wstring(sliderMap[Tolerance].GetValue()) + L"cm";
			SetDlgItemText(windowHandle, IDC_SEGMENTATION_E_TEXT_CLUSTERTOLERANCE, clusterToleranceString.c_str());
			DebugUtility::DbgOut(L"euclideanParams.clusterTolerance = ", euclideanParams.clusterTolerance);
		}

		if (regionGrowthParams.smoothnessThreshold != sliderMap[Smoothness].GetValue())
		{
			regionGrowthParams.smoothnessThreshold = sliderMap[Smoothness].GetValue();
			wstring smoothnessString = to_wstring((int)regionGrowthParams.smoothnessThreshold) + L"";
			SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_SMOOTHNESS, smoothnessString.c_str());
			DebugUtility::DbgOut(L"regionGrowthParams.smoothnessThreshold = ", regionGrowthParams.smoothnessThreshold);
		}

		if (regionGrowthParams.curvatureThreshold != (float)sliderMap[Curvature].GetValue() / 10.0f)
		{
			regionGrowthParams.curvatureThreshold = (float)sliderMap[Curvature].GetValue() / 10.0f;

			std::wstringstream curvatureString;
			curvatureString << std::setprecision(2) << (float)sliderMap[Curvature].GetValue() / 10.0f;
			//wstring curvatureString = to_wstring(regionGrowthParams.curvatureThreshold) + L"";
			SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_CURVATURE, curvatureString.str().c_str());
			DebugUtility::DbgOut(L"regionGrowthParams.curvatureThreshold = ", regionGrowthParams.curvatureThreshold);
		}

		if (regionGrowthParams.numberOfNeighbors != sliderMap[Neighbors].GetValue())
		{
			regionGrowthParams.numberOfNeighbors = sliderMap[Neighbors].GetValue();
			wstring neighborsString = to_wstring(regionGrowthParams.numberOfNeighbors) + L"";
			SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_NEIGHBORS, neighborsString.c_str());
			DebugUtility::DbgOut(L"regionGrowthParams.numberOfNeighbors = ", regionGrowthParams.numberOfNeighbors);
		}

		if (regionGrowthParams.minComponentSize != sliderMap[MinSize].GetValue())
		{
			regionGrowthParams.minComponentSize = sliderMap[MinSize].GetValue();
			wstring minComponentsString = to_wstring(regionGrowthParams.minComponentSize) + L"";
			SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_SIZE, minComponentsString.c_str());
			DebugUtility::DbgOut(L"regionGrowthParams.minComponentSize = ", regionGrowthParams.minComponentSize);
		}
	}


	LRESULT CALLBACK SegmentationWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
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
			UpdateSegmentationPreviewValues();
		}

		return SubWindow::SubWindowProc(windowHandle, message, wParam, lParam);
	}

	void SegmentationWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_SEGMENTATION_BUTTON_UPDATE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(SegmentationWindowEvent::UpdateSegmentation);
			
		}
		if (IDC_SEGMENTATION_BUTTON_DONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"SegmentationWindow::ProcessUI::Done");
			eventQueue.push(SegmentationWindowEvent::StateChange);
		}
		if (IDC_SEGMENTATION_EUCLIDEAN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"SegmentationWindow::ProcessUI::Euclidean");
			selectedSegmentationType = Euclidean;
			UpdateSegmentationUI();
		}
		if (IDC_SEGMENTATION_REGIONGROWTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"SegmentationWindow::ProcessUI::RG");
			selectedSegmentationType = RegionGrowth;
			UpdateSegmentationUI();

		}
	}

	void SegmentationWindow::HandleEvents(MainWindow* _parentWindow)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case SegmentationWindowEvent::StateChange:
				_parentWindow->FinishObjectSegmentation();
				_parentWindow->ChangeState(PlaneCut);
				break;
			case SegmentationWindowEvent::SegmentationFinished:
				//do stuff
				break;
			case SegmentationWindowEvent::UpdateSegmentation:
				switch (selectedSegmentationType)
				{
				case Euclidean:
					_parentWindow->UpdateObjectSegmentation(&euclideanParams);
					break;
				case RegionGrowth:
					_parentWindow->UpdateObjectSegmentation(&regionGrowthParams);
					break;
				}
			}

			eventQueue.pop();
		}
	}

	void SegmentationWindow::Show()
	{
		SubWindow::Show();
		UpdateSegmentationUI();
	}

	void SegmentationWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"SegmentationWindow::RESIZE");

		SubWindow::Resize(parentWidth, parentHeight);

		int controlX = (int)(0.02f*width);
		int controlWidth = (int)(0.96f*width);
		int buttonHeight = (int)(0.08f*height);

		MoveWindow(buttonSegmentationFinish, controlX, (int)(0.75f*height), controlWidth, (int)(0.2f*height), true);
		MoveWindow(buttonSegmentationBegin, controlX, (int)(0.625f*height), controlWidth, (int)(0.1f*height), true);

		int leftSegmentationXPosition = width / 2 - (int)(0.48f*width);
		int rightSegmentationRightXPosition = (int)(width / 2) + (int)(0.48f*width);
		MoveWindow(buttonEuclideanSegmentation, 0, 0, (int)(width/2), (int)(0.1f*height), true);
		MoveWindow(buttonRegionGrowthSegmentation, (int)(width / 2), 0, (int)(width / 2), (int)(0.1f*height), true);

		//int controlWidth = rightSegmentationRightXPosition - leftSegmentationXPosition;

		MoveWindow(textClusterToleranceLabel, leftSegmentationXPosition, (int)(0.12f*height), controlWidth / 2, 25, true);
		sliderMap[Tolerance].Resize(leftSegmentationXPosition, (int)(0.12f*height) + 30, controlWidth, 40);
		MoveWindow(textClusterTolerance, leftSegmentationXPosition + controlWidth / 2, (int)(0.12f*height), controlWidth / 2, 25, true);

		MoveWindow(textRGSmoothnessLabel, leftSegmentationXPosition, (int)(0.12f*height), controlWidth / 2, 25, true);
		sliderMap[Smoothness].Resize(leftSegmentationXPosition, (int)(0.12f*height) + 30, controlWidth, 40);
		MoveWindow(textRGSmoothness, leftSegmentationXPosition + controlWidth / 2, (int)(0.12f*height), controlWidth / 2, 25, true);
			
		MoveWindow(textRGCurvatureLabel, leftSegmentationXPosition, (int)(0.12f*height) + 75, controlWidth/2, 25, true);
		sliderMap[Curvature].Resize(leftSegmentationXPosition, (int)(0.12f*height) + 105, controlWidth, 40);
		MoveWindow(textRGCurvature, leftSegmentationXPosition + controlWidth / 2, (int)(0.12f*height) + 75, controlWidth / 2, 25, true);

		MoveWindow(textRGNeighborsLabel, leftSegmentationXPosition, (int)(0.12f*height) + 150, controlWidth / 2, 25, true);
		sliderMap[Neighbors].Resize(leftSegmentationXPosition, (int)(0.12f*height) + 180, controlWidth, 40);
		MoveWindow(textRGNeighbors, leftSegmentationXPosition + controlWidth / 2, (int)(0.12f*height) + 150, controlWidth / 2, 25, true);

		MoveWindow(textRGKSearchLabel, leftSegmentationXPosition, (int)(0.12f*height) + 225, controlWidth / 2, 25, true);
		sliderMap[MinSize].Resize(leftSegmentationXPosition, (int)(0.12f*height) + 255, controlWidth, 40);
		MoveWindow(textRGKSearch, leftSegmentationXPosition + controlWidth / 2, (int)(0.12f*height) + 225, controlWidth / 2, 25, true);


	}


	

	void SegmentationWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"SegmentationWindow::CleanUp()");

		generalSegmentationUi.CleanUp();
		euclideanSegmentationUi.CleanUp();
		regionSegmentationUi.CleanUp();
		for (auto& slider : sliderMap)
			slider.second.CleanUp();
		SubWindow::CleanUp();
	}
}