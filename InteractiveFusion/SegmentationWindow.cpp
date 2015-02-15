#include "SegmentationWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "GUIContainer.h"
#include "SegmentationParams.h"

using namespace std;

namespace InteractiveFusion {


	//SEGMENTATION PREVIEW
	GUIContainer generalSegmentationUi;
	HWND buttonSegmentationFinish;
	HWND buttonSegmentationBegin;

	//EUCLIDEAN UI
	GUIContainer euclideanSegmentationUi;
	HWND buttonClusterTolerancePlus, buttonClusterToleranceMinus;
	HWND textClusterToleranceLabel, textClusterTolerance;

	//REGION GROWTH UI
	GUIContainer  regionSegmentationUi;
	HWND buttonRegionGrowthSegmentation, buttonEuclideanSegmentation;
	HWND buttonRGSmoothnessPlus, buttonRGSmoothnessMinus;
	HWND textRGSmoothnessLabel, textRGSmoothness;
	HWND buttonRGCurvaturePlus, buttonRGCurvatureMinus;
	HWND textRGCurvatureLabel, textRGCurvature;
	HWND buttonRGNeighborsPlus, buttonRGNeighborsMinus;
	HWND textRGNeighborsLabel, textRGNeighbors;
	HWND buttonRGKSearchPlus, buttonRGKSearchMinus;
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
		textClusterTolerance = CreateWindowEx(0, L"STATIC", L"2cm", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_E_TEXT_CLUSTERTOLERANCE, hInstance, 0);

		SendMessage(textClusterTolerance, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textClusterToleranceLabel = CreateWindowEx(0, L"STATIC", L"TOLERANCE", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_E_TEXT_CLUSTERTOLERANCE_LABEL, hInstance, 0);

		SendMessage(textClusterToleranceLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		buttonClusterTolerancePlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_E_BUTTON_CLUSTERTOLERANCE_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonClusterTolerancePlus, ButtonLayout());
		buttonLayoutMap[buttonClusterTolerancePlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));

		buttonClusterToleranceMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_E_BUTTON_CLUSTERTOLERANCE_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonClusterToleranceMinus, ButtonLayout());
		buttonLayoutMap[buttonClusterToleranceMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRegionGrowthSegmentation = CreateWindowEx(0, L"Button", L"RG", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_REGIONGROWTH, hInstance, 0);

		buttonLayoutMap.emplace(buttonRegionGrowthSegmentation, ButtonLayout());
		buttonLayoutMap[buttonRegionGrowthSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
		//buttonLayoutMap[buttonRegionGrowthSegmentation].SetFontSize(30);
		buttonEuclideanSegmentation = CreateWindowEx(0, L"Button", L"EUCLIDEAN", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_EUCLIDEAN, hInstance, 0);

		buttonLayoutMap.emplace(buttonEuclideanSegmentation, ButtonLayout());
		buttonLayoutMap[buttonEuclideanSegmentation].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(InactiveMode));
		//buttonLayoutMap[buttonEuclideanSegmentation].SetFontSize(30);
		buttonRGSmoothnessPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_SMOOTHNESS_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGSmoothnessPlus, ButtonLayout());
		buttonLayoutMap[buttonRGSmoothnessPlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRGSmoothnessMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_SMOOTHNESS_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGSmoothnessMinus, ButtonLayout());
		buttonLayoutMap[buttonRGSmoothnessMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		textRGSmoothness = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_SMOOTHNESS, hInstance, 0);

		SendMessage(textRGSmoothness, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGSmoothnessLabel = CreateWindowEx(0, L"STATIC", L"SMOOTHNESS", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_SMOOTHNESS_LABEL, hInstance, 0);

		SendMessage(textRGSmoothnessLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		buttonRGCurvaturePlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_CURVATURE_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGCurvaturePlus, ButtonLayout());
		buttonLayoutMap[buttonRGCurvaturePlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRGCurvatureMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_CURVATURE_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGCurvatureMinus, ButtonLayout());
		buttonLayoutMap[buttonRGCurvatureMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRGNeighborsPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_NEIGHBORS_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGNeighborsPlus, ButtonLayout());
		buttonLayoutMap[buttonRGNeighborsPlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRGNeighborsMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_NEIGHBORS_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGNeighborsMinus, ButtonLayout());
		buttonLayoutMap[buttonRGNeighborsMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRGKSearchPlus = CreateWindowEx(0, L"Button", L"+", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_KSEARCH_PLUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGKSearchPlus, ButtonLayout());
		buttonLayoutMap[buttonRGKSearchPlus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		buttonRGKSearchMinus = CreateWindowEx(0, L"Button", L"-", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_BUTTON_KSEARCH_MINUS, hInstance, 0);

		buttonLayoutMap.emplace(buttonRGKSearchMinus, ButtonLayout());
		buttonLayoutMap[buttonRGKSearchMinus].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(GlobalDefault));
		textRGCurvature = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_CURVATURE, hInstance, 0);

		SendMessage(textRGCurvature, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGCurvatureLabel = CreateWindowEx(0, L"STATIC", L"CURVATURE", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_CURVATURE_LABEL, hInstance, 0);

		SendMessage(textRGCurvatureLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textRGNeighbors = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_NEIGHBORS, hInstance, 0);

		SendMessage(textRGNeighbors, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGNeighborsLabel = CreateWindowEx(0, L"STATIC", L"NEIGHBORS", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_NEIGHBORS_LABEL, hInstance, 0);

		SendMessage(textRGNeighborsLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);

		textRGKSearch = CreateWindowEx(0, L"STATIC", L"10", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_KSEARCH, hInstance, 0);

		SendMessage(textRGKSearch, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);
		textRGKSearchLabel = CreateWindowEx(0, L"STATIC", L"KSEARCH", WS_CHILD | WS_VISIBLE | SS_CENTER | SS_CENTERIMAGE, 250, 50, 150, 50, windowHandle, (HMENU)IDC_SEGMENTATION_RG_TEXT_KSEARCH_LABEL, hInstance, 0);

		SendMessage(textRGKSearchLabel, WM_SETFONT, (WPARAM)uiFontMedium, TRUE);


		generalSegmentationUi.Add(buttonSegmentationFinish);
		generalSegmentationUi.Add(buttonSegmentationBegin);
		generalSegmentationUi.Add(buttonRegionGrowthSegmentation);
		generalSegmentationUi.Add(buttonEuclideanSegmentation);

		regionSegmentationUi.Add(buttonRGSmoothnessPlus);
		regionSegmentationUi.Add(buttonRGSmoothnessMinus);
		regionSegmentationUi.Add(textRGSmoothness);
		regionSegmentationUi.Add(textRGSmoothnessLabel);

		regionSegmentationUi.Add(buttonRGCurvaturePlus);
		regionSegmentationUi.Add(buttonRGCurvatureMinus);
		regionSegmentationUi.Add(buttonRGNeighborsPlus);
		regionSegmentationUi.Add(buttonRGNeighborsMinus);
		regionSegmentationUi.Add(buttonRGKSearchPlus);
		regionSegmentationUi.Add(buttonRGKSearchMinus);
		regionSegmentationUi.Add(textRGCurvature);
		regionSegmentationUi.Add(textRGCurvatureLabel);
		regionSegmentationUi.Add(textRGNeighbors);
		regionSegmentationUi.Add(textRGNeighborsLabel);
		regionSegmentationUi.Add(textRGKSearch);
		regionSegmentationUi.Add(textRGKSearchLabel);

		euclideanSegmentationUi.Add(textClusterTolerance);
		euclideanSegmentationUi.Add(textClusterToleranceLabel);
		euclideanSegmentationUi.Add(buttonClusterTolerancePlus);
		euclideanSegmentationUi.Add(buttonClusterToleranceMinus);
	
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
		int clusterToleranceLabel = (int)(euclideanParams.clusterTolerance * 1000);
		wstring clusterToleranceString = to_wstring(clusterToleranceLabel) + L"cm";
		SetDlgItemText(windowHandle, IDC_SEGMENTATION_E_TEXT_CLUSTERTOLERANCE, clusterToleranceString.c_str());

		int smoothnessLabel = (int)regionGrowthParams.smoothnessThreshold;
		wstring smoothnessString = to_wstring(smoothnessLabel) + L"";
		SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_SMOOTHNESS, smoothnessString.c_str());

		int curvatureLabel = (int)regionGrowthParams.curvatureThreshold;
		wstring curvatureString = to_wstring(curvatureLabel) + L"";
		SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_CURVATURE, curvatureString.c_str());

		int neighborsLabel = (int)regionGrowthParams.numberOfNeighbors;
		wstring neighborsString = to_wstring(neighborsLabel) + L"";
		SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_NEIGHBORS, neighborsString.c_str());

		int kSearchLabel = (int)regionGrowthParams.kSearchValue;
		wstring kSearchString = to_wstring(kSearchLabel) + L"";
		SetDlgItemText(windowHandle, IDC_SEGMENTATION_RG_TEXT_KSEARCH, kSearchString.c_str());
	}


	LRESULT CALLBACK SegmentationWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
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
			//parentWindow->ChangeState(Processing);
			/*if (stateManager.GetWindowMode() == IF_MODE_SEGMENTATION && glSegmentation.GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
			{
				appStatus.SetViewportStatusMessage(L"Finishing segmentation");
				glSegmentation.previewMode = false;
				glSegmentation.StartSegmentation();
			}*/
		}
		if (IDC_SEGMENTATION_EUCLIDEAN == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"SegmentationWindow::ProcessUI::Euclidean");
			selectedSegmentationType = Euclidean;
			UpdateSegmentationUI();
			/*if (stateManager.GetWindowMode() == IF_MODE_SEGMENTATION && glSegmentation.GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
			{
				glSegmentation.SetSegmentationMode(SEGMENTATION_EUCLIDEAN);
				EnableWindow(hButtonRegionGrowthSegmentation, true);
				EnableWindow(hButtonEuclideanSegmentation, false);
				regionGrowthUi->Hide();
				euclideanUi->Show();
				MoveButtonsOnResize();
			}*/
		}
		if (IDC_SEGMENTATION_REGIONGROWTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"SegmentationWindow::ProcessUI::RG");
			selectedSegmentationType = RegionGrowth;
			UpdateSegmentationUI();
			/*if (stateManager.GetWindowMode() == IF_MODE_SEGMENTATION && glSegmentation.GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
			{
				glSegmentation.SetSegmentationMode(SEGMENTATION_REGIONGROWTH);
				EnableWindow(hButtonRegionGrowthSegmentation, false);
				EnableWindow(hButtonEuclideanSegmentation, true);
				euclideanUi->Hide();
				regionGrowthUi->Show();
				MoveButtonsOnResize();
			}*/
		}
		if (IDC_SEGMENTATION_RG_BUTTON_CURVATURE_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
				float step = 0.0f;
				if (regionGrowthParams.curvatureThreshold > 10)
					step = 1;
				else if (regionGrowthParams.curvatureThreshold > 3)
					step = 0.5f;
				else
					step = 0.1f;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.curvatureThreshold += step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_CURVATURE_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (regionGrowthParams.curvatureThreshold <= 0.1f)
					return;
				float step = 0.0f;
				if (regionGrowthParams.curvatureThreshold > 10)
					step = 1;
				else if (regionGrowthParams.curvatureThreshold > 3)
					step = 0.5f;
				else
					step = 0.1f;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.curvatureThreshold -= step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_NEIGHBORS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
				int step = 1;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.numberOfNeighbors += step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_NEIGHBORS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (regionGrowthParams.numberOfNeighbors <= 1)
					return;
				int step = 1;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.numberOfNeighbors -= step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_KSEARCH_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
				int step = 1;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.kSearchValue += step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_KSEARCH_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (regionGrowthParams.kSearchValue <= 1)
					return;
				int step = 1;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.kSearchValue -= step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_SMOOTHNESS_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
				float step = 0.0f;
				if (regionGrowthParams.smoothnessThreshold > 100)
					step = 10;
				else if (regionGrowthParams.smoothnessThreshold > 20)
					step = 5;
				else
					step = 1;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.smoothnessThreshold += step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_RG_BUTTON_SMOOTHNESS_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (regionGrowthParams.smoothnessThreshold <= 1)
					return;
				float step = 0.0f;
				if (regionGrowthParams.smoothnessThreshold > 100)
					step = 10;
				else if (regionGrowthParams.smoothnessThreshold > 20)
					step = 5;
				else
					step = 1;

				//glSegmentation.segmentValuesChanged = true;
				regionGrowthParams.smoothnessThreshold -= step;
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_E_BUTTON_CLUSTERTOLERANCE_PLUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
				float step = 0.0f;
				if (euclideanParams.clusterTolerance >= 0.05f)
					step = 0.01f;
				else if (euclideanParams.clusterTolerance >= 0.01f)
					step = 0.005f;
				else
					step = 0.001f;

				//glSegmentation.segmentValuesChanged = true;
				euclideanParams.clusterTolerance += step;
				DebugUtility::DbgOut(L"PLUS: ", euclideanParams.clusterTolerance);
				UpdateSegmentationPreviewValues();
		}
		if (IDC_SEGMENTATION_E_BUTTON_CLUSTERTOLERANCE_MINUS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (euclideanParams.clusterTolerance <= 0.001f)
					return;
				float step = 0.0f;
				if (euclideanParams.clusterTolerance > 0.05f)
					step = 0.01f;
				else if (euclideanParams.clusterTolerance > 0.01f)
					step = 0.005f;
				else
					step = 0.001f;

				//glSegmentation.segmentValuesChanged = true;
				euclideanParams.clusterTolerance -= step;
				DebugUtility::DbgOut(L"MINUS: ", euclideanParams.clusterTolerance);
				UpdateSegmentationPreviewValues();
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

	void SegmentationWindow::Resize(int parentWidth, int parentHeight)
	{
		DebugUtility::DbgOut(L"SegmentationWindow::RESIZE");
		//RECT rRect;
		//GetClientRect(parentHandle, &rRect);

		SubWindow::Resize(parentWidth, parentHeight);

		MoveWindow(buttonSegmentationFinish, (int)(0.1f*width), (int)(0.75f*height), (int)(0.80f*width), (int)(0.2f*height), true);
		MoveWindow(buttonSegmentationBegin, (int)(0.1f*width), (int)(0.625f*height), (int)(0.80f*width), (int)(0.1f*height), true);

		int leftSegmentationXPosition = width / 2 - (int)(0.48f*width);
		int rightSegmentationRightXPosition = (int)(width / 2) + (int)(0.48f*width);
		MoveWindow(buttonEuclideanSegmentation, 0, (int)(0.05*height), (int)(width/2), 50, true);
		MoveWindow(buttonRegionGrowthSegmentation, (int)(width / 2), (int)(0.05*height), (int)(width / 2), 50, true);
		//MoveWindow(buttonEuclideanSegmentation, (int)(0.05f*width), (int)(0.15f*height), (int)(0.45f*width), 50, true);
		//MoveWindow(buttonRegionGrowthSegmentation, (int)(0.50f*width), (int)(0.15f*height), (int)(0.45f*width), 50, true);

		//if (glSegmentation.GetSegmentationMode() == SEGMENTATION_EUCLIDEAN)
		//{
		int textXPosition = (width / 2) - (int)(0.25*width);
		int textWidth = (int)(0.5f*width);
		MoveWindow(textClusterToleranceLabel, textXPosition, (int)(0.15f*height), textWidth, 25, true);
		MoveWindow(buttonClusterToleranceMinus, leftSegmentationXPosition, (int)(0.15f*height), 50, 50, true);
		MoveWindow(textClusterTolerance, textXPosition, (int)(0.15f*height) + 25, textWidth, 25, true);
			MoveWindow(buttonClusterTolerancePlus, rightSegmentationRightXPosition - 50, (int)(0.15f*height), 50, 50, true);
		//}
		//else if (glSegmentation.GetSegmentationMode() == SEGMENTATION_REGIONGROWTH)
		//{
			
			MoveWindow(textRGSmoothnessLabel, textXPosition, (int)(0.15f*height), textWidth, 25, true);
			MoveWindow(buttonRGSmoothnessMinus, leftSegmentationXPosition, (int)(0.15f*height), 50, 50, true);
			MoveWindow(textRGSmoothness, textXPosition, (int)(0.15f*height) + 25, textWidth, 25, true);
			MoveWindow(buttonRGSmoothnessPlus, rightSegmentationRightXPosition - 50, (int)(0.15f*height), 50, 50, true);

			MoveWindow(textRGCurvatureLabel, textXPosition, (int)(0.15f*height) + 70, textWidth, 25, true);
			MoveWindow(buttonRGCurvatureMinus, leftSegmentationXPosition, (int)(0.15f*height) + 70, 50, 50, true);
			MoveWindow(textRGCurvature, textXPosition, (int)(0.15f*height) + 95, textWidth, 25, true);
			MoveWindow(buttonRGCurvaturePlus, rightSegmentationRightXPosition - 50, (int)(0.15f*height) + 70, 50, 50, true);

			MoveWindow(textRGNeighborsLabel, textXPosition, (int)(0.15f*height) + 140, textWidth, 25, true);
			MoveWindow(buttonRGNeighborsMinus, leftSegmentationXPosition, (int)(0.15f*height) + 140, 50, 50, true);
			MoveWindow(textRGNeighbors, textXPosition, (int)(0.15f*height) + 165, textWidth, 25, true);
			MoveWindow(buttonRGNeighborsPlus, rightSegmentationRightXPosition - 50, (int)(0.15f*height) + 140, 50, 50, true);

			MoveWindow(textRGKSearchLabel, textXPosition, (int)(0.15f*height) + 210, textWidth, 25, true);
			MoveWindow(buttonRGKSearchMinus, leftSegmentationXPosition, (int)(0.15f*height) + 210, 50, 50, true);
			MoveWindow(textRGKSearch, textXPosition, (int)(0.15f*height) + 235, textWidth, 25, true);
			MoveWindow(buttonRGKSearchPlus, rightSegmentationRightXPosition - 50, (int)(0.15f*height) + 210, 50, 50, true);
		//}


	}


	

	void SegmentationWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"SegmentationWindow::CleanUp()");

		generalSegmentationUi.CleanUp();
		euclideanSegmentationUi.CleanUp();
		regionSegmentationUi.CleanUp();
		SubWindow::CleanUp();
	}
}