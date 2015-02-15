#include "InteractionDebugDialog.h"
#include "StateManager.h"
#include "InteractionWindow.h"

namespace InteractiveFusion {

	void InteractionDebugDialog::InitializeDialog(HINSTANCE _hInstance, HWND _parent, shared_ptr<Parameters> _params, shared_ptr<SegmentationHelper> _segmentationHelper, shared_ptr<SelectionHelper> _selectionHelper, shared_ptr<MeshHelper> _meshHelper, shared_ptr<OpenGLCamera> _openGLCamera)
	{
		hInstance = _hInstance;
		parentHandle = _parent;
		interactionParams = _params;
		segmentationHelper = _segmentationHelper;
		selectionHelper = _selectionHelper;
		meshHelper = _meshHelper;
		openGLCamera = _openGLCamera;
		debugHandle = CreateDialogParam(hInstance, MAKEINTRESOURCE(IDD_INTERACTION_ADVANCED_OPTIONS), parentHandle, (DLGPROC)MessageRouter, reinterpret_cast<LPARAM>(this));
		ShowWindow(debugHandle, SW_HIDE);

		InitializeUIControls();
	}

	/// <summary>
	/// Initialize the UI
	/// </summary>
	void InteractionDebugDialog::InitializeUIControls()
	{
		HWND editKSearchHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_RG_KSEARCHVALUE);

		SetWindowLongPtr(editKSearchHandle, GWLP_USERDATA,
				reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		oldEditProc = (WNDPROC)SetWindowLongPtr(editKSearchHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		HWND editMinClustersHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_RG_MINCLUSTERSIZE);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editMinClustersHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editMinClustersHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editMaxClustersHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_RG_MAXCLUSTERSIZE);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editMaxClustersHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editMaxClustersHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editNonHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_RG_NON);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editNonHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editNonHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editSmoothnessHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_RG_SMOOTHNESS);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editSmoothnessHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editSmoothnessHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editCurvatureHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_RG_CURVATURE);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editCurvatureHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editCurvatureHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editFillHoleHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_FILLHOLES);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editFillHoleHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editFillHoleHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editRemoveComponentHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_REMOVESEGMENTS);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editRemoveComponentHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editRemoveComponentHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		HWND editCarryDistanceHandle = GetDlgItem(debugHandle, IDC_IDEBUG_EDIT_SELECTION_DISTANCE);
		oldEditProc = (WNDPROC)SetWindowLongPtr(editCarryDistanceHandle, GWLP_WNDPROC, (LONG_PTR)InteractionDebugDialog::EditMessageRouter);

		SetWindowLongPtr(editCarryDistanceHandle, GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>((reinterpret_cast<InteractionDebugDialog*>(this))));

		//HWND hCheck = GetDlgItem(interactionWindow->glWindowParent, IDC_IDEBUG_CHECK_PLACING_SNAPTOVERTEX);

		//PostMessage(hCheck, BM_SETCHECK, BST_CHECKED, 0);


		ResetEditControls();

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_KSEARCH_VALUE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_KSEARCH_VALUE, MAX_RG_KSEARCH_VALUE));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_REMOVESEGMENTS, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_REMOVESEGMENTS_VALUE, MAX_REMOVESEGMENTS_VALUE));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_SELECTION_DISTANCE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_CARRYDISTANCE, MAX_CARRYDISTANCE));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MINCLUSTERSIZE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_MINCLUSTER, MAX_RG_MINCLUSTER));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_MAXCLUSTER, MAX_RG_MAXCLUSTER));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_NON, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_NEIGHBORS, MAX_RG_NEIGHBORS));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_SMOOTHNESS, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_SMOOTHNESS, MAX_RG_SMOOTHNESS));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_CURVATURE, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_RG_CURVATURE, MAX_RG_CURVATURE));

		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_FILLHOLES, TBM_SETRANGE, TRUE, MAKELPARAM(MIN_FILLHOLES, MAX_FILLHOLES));

		ResetSliders();
	}

	void InteractionDebugDialog::ResetEditControls()
	{
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_KSEARCHVALUE, interactionParams->kSearchValue, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_REMOVESEGMENTS, interactionParams->maxComponentSize, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_SELECTION_DISTANCE, interactionParams->carryDistance, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_FILLHOLES, interactionParams->holeSize * 100, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_MINCLUSTERSIZE, interactionParams->minClusterSize, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_MAXCLUSTERSIZE, interactionParams->maxClusterSize * 1000, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_NON, interactionParams->numberOfNeighbors, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_SMOOTHNESS, (UINT)interactionParams->smoothnessThreshold, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_CURVATURE, (UINT)interactionParams->curvatureThreshold, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_BACKGROUND_RED, 211, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_BACKGROUND_GREEN, 211, FALSE);
		SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_BACKGROUND_BLUE, 211, FALSE);
	}

	void InteractionDebugDialog::ResetSliders()
	{
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_KSEARCH_VALUE, TBM_SETPOS, TRUE, (UINT)interactionParams->kSearchValue);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_REMOVESEGMENTS, TBM_SETPOS, TRUE, (UINT)interactionParams->maxComponentSize);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_SELECTION_DISTANCE, TBM_SETPOS, TRUE, (UINT)interactionParams->carryDistance);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MINCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)interactionParams->minClusterSize);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)interactionParams->maxClusterSize);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_NON, TBM_SETPOS, TRUE, (UINT)interactionParams->numberOfNeighbors);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_SMOOTHNESS, TBM_SETPOS, TRUE, (UINT)interactionParams->smoothnessThreshold);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_CURVATURE, TBM_SETPOS, TRUE, (UINT)interactionParams->curvatureThreshold);
		SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_FILLHOLES, TBM_SETPOS, TRUE, (UINT)interactionParams->holeSize);
	}

	void InteractionDebugDialog::UpdateSliderText()
	{
		wstringstream strs;
		strs << interactionParams->kSearchValue;
		wstring concLabel;
		concLabel.append(strs.str());
		SetDlgItemText(debugHandle, IDC_IDEBUG_TEXT_RG_KSEARCH_VALUE, concLabel.c_str());

		strs.str(L"");
		concLabel = L"";
		strs << interactionParams->minClusterSize;
		concLabel.append(strs.str());
		SetDlgItemText(debugHandle, IDC_IDEBUG_TEXT_RG_MINCLUSTERSIZE, concLabel.c_str());

		strs.str(L"");
		concLabel = L"";
		strs << interactionParams->maxComponentSize;
		concLabel.append(strs.str());
		SetDlgItemText(debugHandle, IDC_IDEBUG_TEXT_RG_MINCLUSTERSIZE, concLabel.c_str());

		strs.str(L"");
		concLabel = L"";
		strs << interactionParams->numberOfNeighbors;
		concLabel.append(strs.str());
		SetDlgItemText(debugHandle, IDC_IDEBUG_TEXT_RG_NON, concLabel.c_str());

		strs.str(L"");
		concLabel = L"";
		strs << interactionParams->smoothnessThreshold;
		concLabel.append(strs.str());
		SetDlgItemText(debugHandle, IDC_IDEBUG_TEXT_RG_SMOOTHNESS, concLabel.c_str());

		strs.str(L"");
		concLabel = L"";
		strs << interactionParams->curvatureThreshold;
		concLabel.append(strs.str());
		SetDlgItemText(debugHandle, IDC_IDEBUG_TEXT_RG_CURVATURE, concLabel.c_str());
	}

	void InteractionDebugDialog::UpdateSliders()
	{
		segmentationHelper->segmentValuesChanged = true;
		int kSearchValuePos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_KSEARCH_VALUE, TBM_GETPOS, 0, 0);

		if (kSearchValuePos >= MIN_RG_KSEARCH_VALUE && kSearchValuePos <= MAX_RG_KSEARCH_VALUE)
		{
			interactionParams->kSearchValue = kSearchValuePos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_KSEARCHVALUE, interactionParams->kSearchValue, FALSE);
		}

		int maxComponetSizePos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_REMOVESEGMENTS, TBM_GETPOS, 0, 0);

		if (maxComponetSizePos >= MIN_REMOVESEGMENTS_VALUE && maxComponetSizePos <= MAX_REMOVESEGMENTS_VALUE)
		{
			interactionParams->maxComponentSize = maxComponetSizePos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_REMOVESEGMENTS, interactionParams->maxComponentSize, FALSE);
		}

		int carryDistancePos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_SELECTION_DISTANCE, TBM_GETPOS, 0, 0);

		if (carryDistancePos >= MIN_CARRYDISTANCE && carryDistancePos <= MAX_CARRYDISTANCE)
		{
			interactionParams->carryDistance = carryDistancePos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_SELECTION_DISTANCE, interactionParams->carryDistance, FALSE);
		}


		int minClusterPos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MINCLUSTERSIZE, TBM_GETPOS, 0, 0);

		if (minClusterPos >= MIN_RG_MINCLUSTER && minClusterPos <= MAX_RG_MINCLUSTER)
		{
			interactionParams->minClusterSize = minClusterPos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_MINCLUSTERSIZE, interactionParams->minClusterSize, FALSE);
		}

		int maxClusterPos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MAXCLUSTERSIZE, TBM_GETPOS, 0, 0);

		if (maxClusterPos >= MIN_RG_MAXCLUSTER && maxClusterPos <= MAX_RG_MAXCLUSTER)
		{
			interactionParams->maxClusterSize = maxClusterPos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_MAXCLUSTERSIZE, interactionParams->maxClusterSize * 1000, FALSE);
		}

		int nonPos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_NON, TBM_GETPOS, 0, 0);

		if (nonPos >= MIN_RG_NEIGHBORS && nonPos <= MAX_RG_NEIGHBORS)
		{
			interactionParams->numberOfNeighbors = nonPos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_NON, interactionParams->numberOfNeighbors, FALSE);
		}

		int smoothnessPos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_SMOOTHNESS, TBM_GETPOS, 0, 0);

		if (smoothnessPos >= MIN_RG_SMOOTHNESS && smoothnessPos <= MAX_RG_SMOOTHNESS)
		{
			interactionParams->smoothnessThreshold = smoothnessPos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_SMOOTHNESS, (UINT)interactionParams->smoothnessThreshold, FALSE);
		}

		int curvaturePos = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_CURVATURE, TBM_GETPOS, 0, 0);

		if (curvaturePos >= MIN_RG_CURVATURE && curvaturePos <= MAX_RG_CURVATURE)
		{
			interactionParams->curvatureThreshold = curvaturePos;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_CURVATURE, (UINT)interactionParams->curvatureThreshold, FALSE);
		}

		int holeSize = (int)SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_FILLHOLES, TBM_GETPOS, 0, 0);

		if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
		{
			interactionParams->holeSize = holeSize;
			SetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_FILLHOLES, interactionParams->holeSize * 100, FALSE);
		}
	}

	LRESULT CALLBACK InteractionDebugDialog::EditMessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		InteractionDebugDialog *pThis;

		
		pThis = reinterpret_cast<InteractionDebugDialog*>(
				GetWindowLongPtr(hWnd, GWLP_USERDATA));

		if (pThis) {
			return pThis->SubEditProc(hWnd, message, wParam, lParam);
		}

		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	LRESULT CALLBACK InteractionDebugDialog::SubEditProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam)
	{
		switch (msg)
		{
		case WM_KEYDOWN:
			switch (wParam)
			{
			case VK_RETURN:
				segmentationHelper->segmentValuesChanged = true;
				if (IDC_IDEBUG_EDIT_RG_KSEARCHVALUE == LOWORD(wParam))
				{
					int kSearchValue = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_KSEARCHVALUE, NULL, FALSE);
					if (kSearchValue >= MIN_RG_KSEARCH_VALUE && kSearchValue <= MAX_RG_KSEARCH_VALUE)
					{
						interactionParams->kSearchValue = kSearchValue;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_KSEARCH_VALUE, TBM_SETPOS, TRUE, (UINT)interactionParams->kSearchValue);
					}
				}
				else if (IDC_IDEBUG_EDIT_REMOVESEGMENTS == LOWORD(wParam))
				{
					int maxComponentSize = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_REMOVESEGMENTS, NULL, FALSE);
					if (maxComponentSize >= MIN_REMOVESEGMENTS_VALUE && maxComponentSize <= MAX_REMOVESEGMENTS_VALUE)
					{
						interactionParams->maxComponentSize = maxComponentSize;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_REMOVESEGMENTS, TBM_SETPOS, TRUE, (UINT)interactionParams->maxComponentSize);
					}
				}
				else if (IDC_IDEBUG_EDIT_SELECTION_DISTANCE == LOWORD(wParam))
				{
					int carryDistance = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_SELECTION_DISTANCE, NULL, FALSE);
					if (carryDistance >= MIN_CARRYDISTANCE && carryDistance <= MAX_CARRYDISTANCE)
					{
						interactionParams->carryDistance = carryDistance;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_SELECTION_DISTANCE, TBM_SETPOS, TRUE, (UINT)interactionParams->carryDistance);
					}
				}
				else if (IDC_IDEBUG_EDIT_RG_MINCLUSTERSIZE == LOWORD(wParam))
				{
					int minClusterSize = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_MINCLUSTERSIZE, NULL, FALSE);
					if (minClusterSize >= MIN_RG_MINCLUSTER && minClusterSize <= MAX_RG_MINCLUSTER)
					{
						interactionParams->minClusterSize = minClusterSize;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MINCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)interactionParams->minClusterSize);
					}
				}
				else if (IDC_IDEBUG_EDIT_RG_MAXCLUSTERSIZE == LOWORD(wParam))
				{
					int maxClusterSize = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_MAXCLUSTERSIZE, NULL, FALSE) / 1000;
					if (maxClusterSize >= MIN_RG_MAXCLUSTER && maxClusterSize <= MAX_RG_MAXCLUSTER)
					{
						interactionParams->maxClusterSize = maxClusterSize;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_MAXCLUSTERSIZE, TBM_SETPOS, TRUE, (UINT)interactionParams->maxClusterSize);
					}
				}
				else if (IDC_IDEBUG_EDIT_RG_NON == LOWORD(wParam))
				{
					int numberOfNeighbors = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_NON, NULL, FALSE);
					if (numberOfNeighbors >= MIN_RG_NEIGHBORS && numberOfNeighbors <= MAX_RG_NEIGHBORS)
					{
						interactionParams->numberOfNeighbors = numberOfNeighbors;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_NON, TBM_SETPOS, TRUE, (UINT)interactionParams->numberOfNeighbors);
					}
				}
				else if (IDC_IDEBUG_EDIT_RG_SMOOTHNESS == LOWORD(wParam))
				{
					int smoothnessThreshold = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_SMOOTHNESS, NULL, FALSE);
					if (smoothnessThreshold >= MIN_RG_SMOOTHNESS && smoothnessThreshold <= MAX_RG_SMOOTHNESS)
					{
						interactionParams->smoothnessThreshold = smoothnessThreshold;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_SMOOTHNESS, TBM_SETPOS, TRUE, (UINT)interactionParams->smoothnessThreshold);
					}
				}
				else if (IDC_IDEBUG_EDIT_RG_CURVATURE == LOWORD(wParam))
				{
					int curvatureThreshold = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_RG_CURVATURE, NULL, FALSE);
					if (curvatureThreshold >= MIN_RG_CURVATURE && curvatureThreshold <= MAX_RG_CURVATURE)
					{
						interactionParams->curvatureThreshold = curvatureThreshold;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_RG_CURVATURE, TBM_SETPOS, TRUE, (UINT)interactionParams->curvatureThreshold);
					}
				}
				else if (IDC_IDEBUG_EDIT_FILLHOLES == LOWORD(wParam))
				{
					int holeSize = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_FILLHOLES, NULL, FALSE) / 100;
					if (holeSize >= MIN_FILLHOLES && holeSize <= MAX_FILLHOLES)
					{
						interactionParams->holeSize = holeSize;
						SendDlgItemMessage(debugHandle, IDC_IDEBUG_SLIDER_FILLHOLES, TBM_SETPOS, TRUE, (UINT)interactionParams->holeSize);
					}
				}
				break;
			}
		default:
			return CallWindowProc(oldEditProc, wnd, msg, wParam, lParam);
		}
		return 0;
	}

	void InteractionDebugDialog::SetBackgroundColor(int redValue, int greenValue, int blueValue)
	{
		interactionParams->bgRed = redValue / 255.0f;
		interactionParams->bgGreen = greenValue / 255.0f;
		interactionParams->bgBlue = blueValue / 255.0f;
	}

	void InteractionDebugDialog::Show()
	{
		ShowWindow(debugHandle, SW_SHOW);

		ResetEditControls();
		ResetSliders();

		MoveOnResize();
	}

	void InteractionDebugDialog::Hide()
	{
		ShowWindow(debugHandle, SW_HIDE);
	}

	bool InteractionDebugDialog::IsVisible()
	{
		return IsWindowVisible(debugHandle);
	}

	void InteractionDebugDialog::MoveOnResize()
	{
		RECT rRect;
		GetClientRect(parentHandle, &rRect);
		MoveWindow(debugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
	}

	LRESULT CALLBACK InteractionDebugDialog::MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		InteractionDebugDialog *pThis;

		if (WM_INITDIALOG == message) {
			pThis = reinterpret_cast<InteractionDebugDialog*>(lParam);
			SetWindowLongPtr(hWnd, GWLP_USERDATA,
				reinterpret_cast<LONG_PTR>(pThis));
		}
		else {
			pThis = reinterpret_cast<InteractionDebugDialog*>(
				GetWindowLongPtr(hWnd, GWLP_USERDATA));
		}

		if (pThis) {
			return pThis->InteractionDebugRouter(hWnd, message, wParam, lParam);
		}

		return DefWindowProc(hWnd, message, wParam, lParam);
	}


	LRESULT CALLBACK InteractionDebugDialog::InteractionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		switch (message)
		{
		case WM_INITDIALOG:
			InitializeUIControls();
			break;
		case WM_CLOSE:
			DestroyWindow(hWnd);
			break;
		case WM_DESTROY:
			break;
		case WM_COMMAND:
			ProcessDebugUI(wParam, lParam);
			break;
		case WM_HSCROLL:
			UpdateSliders();
			break;
		case WM_SIZE:
			//DebugUtility::DbgOut(L"DebugDlgProc WM_SIZE");
			break;
		case WM_NOTIFY:
			break;
		}
		return FALSE;
	}

	void InteractionDebugDialog::ProcessDebugUI(WPARAM wParam, LPARAM lParam)
	{

		if (IDC_IDEBUG_BUTTON_FILLHOLES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			meshHelper->FillHoles(interactionParams->holeSize);
		}
		if (IDC_IDEBUG_BUTTON_RG_RESETVALUES == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			segmentationHelper->segmentValuesChanged = true;
			interactionParams->kSearchValue = 20;
			interactionParams->minClusterSize = 100;
			interactionParams->maxClusterSize = 1000;
			interactionParams->numberOfNeighbors = 20;
			interactionParams->smoothnessThreshold = 100;
			interactionParams->curvatureThreshold = 10;
			ResetEditControls();
			ResetSliders();
		}

		if (IDC_IDEBUG_CHECK_WIREFRAME == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle our internal state for near mode
			interactionParams->wireFrameMode = !interactionParams->wireFrameMode;
		}
		if (IDC_IDEBUG_CHECK_FREECAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (openGLCamera->mode == CAMERA_FREE)
				openGLCamera->mode = CAMERA_SENSOR;
			else
				openGLCamera->mode = CAMERA_FREE;
		}
		if (IDC_IDEBUG_CHECK_COLORSELECTION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle our internal state for near mode
			selectionHelper->colorSelection = !selectionHelper->colorSelection;
			if (!selectionHelper->colorSelection)
				selectionHelper->Unselect();
		}
		if (IDC_IDEBUG_CHECK_PLACING_RAYCAST == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			selectionHelper->placeWithRaycast = !selectionHelper->placeWithRaycast;
		}
		if (IDC_IDEBUG_CHECK_PLACING_SNAPTOVERTEX == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle our internal state for near mode
			selectionHelper->snapToVertex = !selectionHelper->snapToVertex;
		}
		if (IDC_IDEBUG_CHECK_SHOWBB == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle our internal state for near mode
			interactionParams->showBB = !interactionParams->showBB;
		}
		if (IDC_IDEBUG_CHECK_RG_ESTIMATENORMALS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			segmentationHelper->segmentValuesChanged = true;
			interactionParams->estimateNormals = !interactionParams->estimateNormals;
		}
		if (IDC_IDEBUG_CHECK_HELPINGVISUALS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			interactionParams->helpingVisuals = !interactionParams->helpingVisuals;
		}
		if (IDC_IDEBUG_BUTTON_RG_EXTERNALPREVIEW == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			//ShowPCLViewer();
		}
		if (IDC_IDEBUG_BUTTON_WALL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"select wall: ");
			selectionHelper->SelectWallObject();
		}
		if (IDC_IDEBUG_BUTTON_MLS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"select MLS: ");
			//MLS();
		}
		if (IDC_IDEBUG_BUTTON_REMOVESEGMENTS == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"select Remove Segments: ");
			int size = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_REMOVESEGMENTS, NULL, FALSE);
			meshHelper->RemoveSmallComponents(size);
		}
		if (IDC_IDEBUG_BUTTON_RESETWALL == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			selectionHelper->ResetWallObject();
		}
		if (IDC_IDEBUG_BUTTON_SETBACKGROUND == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			//GetDlgItemInt
			int redValue = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_BACKGROUND_RED, NULL, FALSE);
			int greenValue = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_BACKGROUND_GREEN, NULL, FALSE);
			int blueValue = GetDlgItemInt(debugHandle, IDC_IDEBUG_EDIT_BACKGROUND_BLUE, NULL, FALSE);
			if (redValue >= 0 && redValue <= 255
				&& greenValue >= 0 && greenValue <= 255
				&& blueValue >= 0 && blueValue <= 255)
			{
				SetBackgroundColor(redValue, greenValue, blueValue);
			}
			//int redValue = (int)itemBuff;

			//m_processor.ResetReconstruction();
		}
		if (IDC_IDEBUG_BUTTON_RG_SEGMENTATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			//interactionWindow->segmentationMode = REGION_GROWTH_SEGMENTATION;
			segmentationHelper->SetSegmentationMode(SEGMENTATION_REGIONGROWTH);
			segmentationHelper->previewMode = false;
			glSegmentation.StartSegmentation();
			//m_processor.ResetReconstruction();
		}

		if (IDC_IDEBUG_BUTTON_CLEANMESH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			if (stateManager.GetWindowBusyState() == IF_BUSYSTATE_DEFAULT)
				stateManager.SetWindowBusyState(IF_BUSYSTATE_BUSY);
			else
				stateManager.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
			/*DebugUtility::DbgOut(L"clean mesh", 0);
			interactionWindow->ShowStatusBarMessage(L"Cleaning mesh...");
			meshHelper->CleanMesh();*/

		}
		if (IDC_IDEBUG_BUTTON_RG_PREVIEW == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			segmentationHelper->SetSegmentationMode(SEGMENTATION_REGIONGROWTH);
			segmentationHelper->previewMode = true;
			segmentationHelper->StartSegmentation();
			//m_processor.ResetReconstruction();
		}
		if (IDC_IDEBUG_BUTTON_RESETCAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			openGLCamera->ResetCameraPosition();
		}

	}

	void InteractionDebugDialog::CleanUp()
	{
		DestroyWindow(debugHandle);
	}

}