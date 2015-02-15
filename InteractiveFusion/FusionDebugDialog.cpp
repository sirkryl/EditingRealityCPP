#include "FusionDebugDialog.h"
#include "DebugUtility.h"

#include "IFResources.h"
#include "IFValues.h"

#include <Shlobj.h>

using namespace std;

namespace InteractiveFusion {

	void FusionDebugDialog::InitializeDialog(HINSTANCE _hInstance, HWND _parent, shared_ptr<KinectFusionParams> _params, shared_ptr<KinectFusionProcessor> _processor)
	{
		hInstance = _hInstance;
		parentHandle = _parent;
		fusionParams = _params;
		fusionProcessor = _processor;
		debugHandle = CreateDialogParam(hInstance, MAKEINTRESOURCE(IDD_SCAN_ADVANCED_OPTIONS), parentHandle, (DLGPROC)MessageRouter, reinterpret_cast<LPARAM>(this));
		ShowWindow(debugHandle, SW_HIDE);
		
		InitializeUIControls();
	}

	/// <summary>
	/// Initialize the UI
	/// </summary>
	void FusionDebugDialog::InitializeUIControls()
	{
		// Set slider ranges
		SendDlgItemMessage(
			debugHandle,
			IDC_SDEBUG_SLIDER_DEPTH_MIN,
			TBM_SETRANGE,
			TRUE,
			MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

		SendDlgItemMessage(debugHandle,
			IDC_SDEBUG_SLIDER_DEPTH_MAX,
			TBM_SETRANGE,
			TRUE,
			MAKELPARAM(MIN_DEPTH_DISTANCE_MM, MAX_DEPTH_DISTANCE_MM));

		SendDlgItemMessage(
			debugHandle,
			IDC_SDEBUG_SLIDER_INTEGRATION_WEIGHT,
			TBM_SETRANGE,
			TRUE,
			MAKELPARAM(MIN_INTEGRATION_WEIGHT, MAX_INTEGRATION_WEIGHT));

		// Set slider positions
		SendDlgItemMessage(
			debugHandle,
			IDC_SDEBUG_SLIDER_DEPTH_MAX,
			TBM_SETPOS,
			TRUE,
			(UINT)fusionParams->m_fMaxDepthThreshold * 1000);

		SendDlgItemMessage(
			debugHandle,
			IDC_SDEBUG_SLIDER_DEPTH_MIN,
			TBM_SETPOS,
			TRUE,
			(UINT)fusionParams->m_fMinDepthThreshold * 1000);

		SendDlgItemMessage(
			debugHandle,
			IDC_SDEBUG_SLIDER_INTEGRATION_WEIGHT,
			TBM_SETPOS,
			TRUE,
			(UINT)fusionParams->m_cMaxIntegrationWeight);

		// Set intermediate slider tics at meter intervals
		for (int i = 1; i<(MAX_DEPTH_DISTANCE_MM / 1000); i++)
		{
			SendDlgItemMessage(debugHandle, IDC_SDEBUG_SLIDER_DEPTH_MAX, TBM_SETTIC, 0, i * 1000);
			SendDlgItemMessage(debugHandle, IDC_SDEBUG_SLIDER_DEPTH_MIN, TBM_SETTIC, 0, i * 1000);
		}

		// Update slider text
		WCHAR str[MAX_PATH];
		swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", fusionParams->m_fMinDepthThreshold);
		SetDlgItemText(debugHandle, IDC_SDEBUG_TEXT_MIN_DIST, str);
		swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", fusionParams->m_fMaxDepthThreshold);
		SetDlgItemText(debugHandle, IDC_SDEBUG_TEXT_MAX_DIST, str);

		swprintf_s(str, ARRAYSIZE(str), L"%d", fusionParams->m_cMaxIntegrationWeight);
		SetDlgItemText(debugHandle, IDC_SDEBUG_TEXT_INTEGRATION_WEIGHT, str);

		// Set the radio buttons for Volume Parameters
		switch ((int)fusionParams->m_reconstructionParams.voxelsPerMeter)
		{
		case 768:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_768, BST_CHECKED);
			break;
		case 640:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_640, BST_CHECKED);
			break;
		case 512:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_512, BST_CHECKED);
			break;
		case 384:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_384, BST_CHECKED);
			break;
		case 256:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_256, BST_CHECKED);
			break;
		case 128:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_128, BST_CHECKED);
			break;
		default:
			//m_params.m_reconstructionParams.voxelsPerMeter = 256.0f;	// set to medium default
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VPM_256, BST_CHECKED);
			break;
		}

		switch ((int)fusionParams->m_reconstructionParams.voxelCountX)
		{
		case 640:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_X_640, BST_CHECKED);
			break;
		case 512:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_X_512, BST_CHECKED);
			break;
		case 384:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_X_384, BST_CHECKED);
			break;
		case 256:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_X_256, BST_CHECKED);
			break;
		case 128:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_X_128, BST_CHECKED);
			break;
		default:
			//m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_X_384, BST_CHECKED);
			break;
		}

		switch ((int)fusionParams->m_reconstructionParams.voxelCountY)
		{
		case 640:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_640, BST_CHECKED);
			break;
		case 512:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_512, BST_CHECKED);
			break;
		case 384:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_384, BST_CHECKED);
			break;
		case 256:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_256, BST_CHECKED);
			break;
		case 128:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_128, BST_CHECKED);
			break;
		default:
			//m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Y_384, BST_CHECKED);
			break;
		}

		switch ((int)fusionParams->m_reconstructionParams.voxelCountZ)
		{
		case 640:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_640, BST_CHECKED);
			break;
		case 512:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_512, BST_CHECKED);
			break;
		case 384:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_384, BST_CHECKED);
			break;
		case 256:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_256, BST_CHECKED);
			break;
		case 128:
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_128, BST_CHECKED);
			break;
		default:
			//m_params.m_reconstructionParams.voxelCountX = 384;	// set to medium default
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_VOXELS_Z_384, BST_CHECKED);
			break;
		}

		if (fusionParams->m_bCaptureColor)
		{
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_CAPTURE_COLOR, BST_CHECKED);
		}

		if (fusionParams->m_bAutoFindCameraPoseWhenLost)
		{
			CheckDlgButton(debugHandle, IDC_SDEBUG_CHECK_CAMERA_POSE_FINDER, BST_CHECKED);
		}
	}


	void FusionDebugDialog::Show()
	{
		ShowWindow(debugHandle, SW_SHOW);
		MoveOnResize();
	}

	void FusionDebugDialog::Hide()
	{
		ShowWindow(debugHandle, SW_HIDE);
	}

	bool FusionDebugDialog::IsVisible()
	{
		return IsWindowVisible(debugHandle);
	}

	void FusionDebugDialog::MoveOnResize()
	{
		RECT rRect;
		GetClientRect(parentHandle, &rRect);
		MoveWindow(debugHandle, rRect.right - 400, 0, 400, rRect.bottom, true);
	}

	LRESULT CALLBACK FusionDebugDialog::MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		FusionDebugDialog *pThis;

		if (WM_INITDIALOG == message) {
			pThis = reinterpret_cast<FusionDebugDialog*>(lParam);
			SetWindowLongPtr(hWnd, GWLP_USERDATA,
				reinterpret_cast<LONG_PTR>(pThis));
		}
		else {
			pThis = reinterpret_cast<FusionDebugDialog*>(
				GetWindowLongPtr(hWnd, GWLP_USERDATA));
		}

		if (pThis) {
			return pThis->FusionDebugRouter(hWnd, message, wParam, lParam);
		}

		return DefWindowProc(hWnd, message, wParam, lParam);
	}


	LRESULT CALLBACK FusionDebugDialog::FusionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		//DebugUtility::DbgOut(L"msg?");
		switch (message)
		{
			case WM_COMMAND:
				ProcessDebugUI(wParam, lParam);
				break;
			case  WM_HSCROLL:
				UpdateHSliders();
				break;
		}
		return FALSE;
	}

	void FusionDebugDialog::ProcessDebugUI(WPARAM wParam, LPARAM lParam)
	{
		
		// If it was for the near mode control and a clicked event, change near mode
		if (IDC_SDEBUG_CHECK_NEARMODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle our internal state for near mode
			fusionParams->m_bNearMode = !fusionParams->m_bNearMode;
			DebugUtility::DbgOut(L"yep");
		}
		// If it was for the display surface normals toggle this variable
		if (IDC_SDEBUG_CHECK_CAPTURE_COLOR == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle capture color
			fusionParams->m_bCaptureColor = !fusionParams->m_bCaptureColor;
			/*if (fusionParams->m_reconstructionParams.voxelsPerMeter != 64)
			fusionParams->m_reconstructionParams.voxelsPerMeter = 64;
			else
			fusionParams->m_reconstructionParams.voxelsPerMeter = 256;*/
		}
		// If it was for the display surface normals toggle this variable
		if (IDC_SDEBUG_CHECK_MIRROR_DEPTH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle depth mirroring
			fusionParams->m_bMirrorDepthFrame = !fusionParams->m_bMirrorDepthFrame;

			fusionProcessor->ResetReconstruction();
		}
		if (IDC_SDEBUG_CHECK_CAMERA_POSE_FINDER == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_bAutoFindCameraPoseWhenLost = !fusionParams->m_bAutoFindCameraPoseWhenLost;

			if (!fusionParams->m_bAutoFindCameraPoseWhenLost)
			{
				// Force pause integration off when unchecking use of camera pose finder
				fusionParams->m_bPauseIntegration = false;
			}
		}

		if (IDC_SDEBUG_CHECK_PAUSE_INTEGRATION == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			// Toggle the pause state of the reconstruction
			fusionParams->m_bPauseIntegration = !fusionParams->m_bPauseIntegration;
		}
		if (IDC_SDEBUG_CHECK_VPM_768 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelsPerMeter = 768.0f;
		}
		if (IDC_SDEBUG_CHECK_VPM_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelsPerMeter = 640.0f;
		}
		if (IDC_SDEBUG_CHECK_VPM_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelsPerMeter = 512.0f;
		}
		if (IDC_SDEBUG_CHECK_VPM_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelsPerMeter = 384.0f;
		}
		if (IDC_SDEBUG_CHECK_VPM_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelsPerMeter = 256.0f;
		}
		if (IDC_SDEBUG_CHECK_VPM_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelsPerMeter = 128.0f;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_X_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountX = 640;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_X_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountX = 512;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_X_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountX = 384;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_X_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountX = 256;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_X_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountX = 128;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Y_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountY = 640;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Y_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountY = 512;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Y_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountY = 384;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Y_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountY = 256;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Y_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountY = 128;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Z_640 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountZ = 640;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Z_512 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountZ = 512;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Z_384 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountZ = 384;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Z_256 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountZ = 256;
		}
		if (IDC_SDEBUG_CHECK_VOXELS_Z_128 == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			fusionParams->m_reconstructionParams.voxelCountZ = 128;
		}
		fusionProcessor->SetParams(*fusionParams);

	}

	void FusionDebugDialog::UpdateHSliders()
	{
		int mmMinPos = (int)SendDlgItemMessage(debugHandle, IDC_SDEBUG_SLIDER_DEPTH_MIN, TBM_GETPOS, 0, 0);

		if (mmMinPos >= MIN_DEPTH_DISTANCE_MM && mmMinPos <= MAX_DEPTH_DISTANCE_MM)
		{
			fusionParams->m_fMinDepthThreshold = (float)mmMinPos * 0.001f;
		}

		int mmMaxPos = (int)SendDlgItemMessage(debugHandle, IDC_SDEBUG_SLIDER_DEPTH_MAX, TBM_GETPOS, 0, 0);

		if (mmMaxPos >= MIN_DEPTH_DISTANCE_MM && mmMaxPos <= MAX_DEPTH_DISTANCE_MM)
		{
			fusionParams->m_fMaxDepthThreshold = (float)mmMaxPos * 0.001f;
		}

		int maxWeight = (int)SendDlgItemMessage(debugHandle, IDC_SDEBUG_SLIDER_INTEGRATION_WEIGHT, TBM_GETPOS, 0, 0);
		fusionParams->m_cMaxIntegrationWeight = maxWeight % (MAX_INTEGRATION_WEIGHT + 1);


		// update text
		WCHAR str[MAX_PATH];
		swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", fusionParams->m_fMinDepthThreshold);
		SetDlgItemText(debugHandle, IDC_SDEBUG_TEXT_MIN_DIST, str);
		swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", fusionParams->m_fMaxDepthThreshold);
		SetDlgItemText(debugHandle, IDC_SDEBUG_TEXT_MAX_DIST, str);

		swprintf_s(str, ARRAYSIZE(str), L"%d", fusionParams->m_cMaxIntegrationWeight);
		SetDlgItemText(debugHandle, IDC_SDEBUG_TEXT_INTEGRATION_WEIGHT, str);

		fusionProcessor->SetParams(*fusionParams);
	}

	void FusionDebugDialog::CleanUp()
	{
		DestroyWindow(debugHandle);
	}

}