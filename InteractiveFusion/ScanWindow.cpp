#define NOMINMAX
#include "ScanWindow.h"
#include "GUIContainer.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "MeshContainer.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "KinectFusion.h"
#include <boost/thread.hpp>

namespace InteractiveFusion {

	#define WM_FRAMEREADY           (WM_USER + 0)
	#define WM_UPDATESENSORSTATUS   (WM_USER + 1)

	boost::thread kinectFusionThread;

	GUIContainer scanUi;
	HWND buttonScanDone, buttonScanReset;
	HWND reconstructionView, depthView, residualsView;

	std::unique_ptr<IScanner> scanner;
	//KinectFusion kinectFusionScanner;
	std::shared_ptr<MeshContainer> scannedMesh;
	glm::mat4 cameraMatrix;

	int voxelsPerMeter;

	bool pauseFusion = false;

	DWORD scanTickLastFpsUpdate;

	ScanWindow::ScanWindow()
	{
	}


	ScanWindow::~ScanWindow()
	{
	}

	void ScanWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		SubWindow::Initialize(_parentHandle, _hInstance, _marginTop, _marginBottom, _marginRight, _marginLeft, _className, _backgroundColor);

		buttonScanDone = CreateWindowEx(0, L"Button", L"DONE", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 50, 150, 50, windowHandle, (HMENU)IDC_SCAN_BUTTON_DONE, hInstance, 0);

		buttonLayoutMap.emplace(buttonScanDone, ButtonLayout());
		buttonLayoutMap[buttonScanDone].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Green));
		buttonScanReset = CreateWindowEx(0, L"Button", L"RESET", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, 50, 500, 150, 50, windowHandle, (HMENU)IDC_SCAN_BUTTON_RESET, hInstance, 0);

		buttonLayoutMap.emplace(buttonScanReset, ButtonLayout());
		buttonLayoutMap[buttonScanReset].SetLayoutParams(StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::Red));

		scanUi.Add(buttonScanDone);
		scanUi.Add(buttonScanReset);

		reconstructionView = CreateWindowEx(0, L"Static", 0, WS_CHILD | WS_VISIBLE | SS_BLACKFRAME | WS_CLIPSIBLINGS, 0, 0, 758, 532, windowHandle, (HMENU)IDC_SCAN_RECONSTRUCTION_VIEW, hInstance, 0);


		/*depthView = CreateWindowEx(0, L"Static", 0, WS_CHILD | WS_VISIBLE | SS_BLACKFRAME | WS_CLIPSIBLINGS, 7, 7, 320, 215, windowHandle, (HMENU)IDC_SCAN_DEPTH_VIEW, hInstance, 0);
		SetWindowPos(depthView, HWND_TOP, 0, 0, 0, 0, 0);
		residualsView = CreateWindowEx(0, L"Static", 0, WS_CHILD | WS_VISIBLE | SS_BLACKFRAME | WS_CLIPSIBLINGS, 7, 222, 320, 215, windowHandle, (HMENU)IDC_SCAN_TRACKING_RESIDUALS_VIEW, hInstance, 0);
		SetWindowPos(residualsView, HWND_TOP, 0, 0, 0, 0, 0);*/

		scanUi.Add(reconstructionView);
		scanUi.Add(depthView);
		scanUi.Add(residualsView);
		scanner = std::unique_ptr<KinectFusion>(new KinectFusion());

		scanner->Initialize(windowHandle, depthView, reconstructionView, residualsView);
		scanner->PauseRendering();
		PauseScan();
		//kinectFusionScanner.Initialize(windowHandle, depthView, reconstructionView, residualsView);
		//kinectFusionScanner.PauseRendering();
	}

	LRESULT CALLBACK ScanWindow::SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		
		PAINTSTRUCT ps;
		switch (message)
		{
		case WM_MOUSEMOVE:
			return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);
			break;
		case WM_NOTIFY:
			//kinectFusionScanner.ResolvePotentialSensorConflict(lParam);
			scanner->ResolvePotentialSensorConflict(lParam);
			break;
		case WM_FRAMEREADY:
			if (!pauseFusion)
			{
				scanner->HandleCompletedFrame();
				cameraMatrix = scanner->GetTransformedCameraMatrix();
				//kinectFusionScanner.HandleCompletedFrame();
				//cameraMatrix = kinectFusionScanner.GetTransformedCameraMatrix();
			}
			break;
		case WM_UPDATESENSORSTATUS:
			scanner->UpdateSensorStatus(wParam);
			//kinectFusionScanner.UpdateSensorStatus(wParam);
			break;
		}
		return SubWindow::SubWindowProc(hWnd, message, wParam, lParam);
	}


	void ScanWindow::HandleEvents(MainWindow& _parentWindow)
	{
		//if (scanner->IsReconstructionReady())
		//{
		//	_parentWindow.InitializeOpenGLScene(scanner->GetScannedMesh());
		//}
		
		if (!IsVisible())
		{
			_parentWindow.ForwardCameraMatrix(cameraMatrix);
			return;
		}

		int newVolumeSize = _parentWindow.GetScanVolumeSize();
		if (HasVolumeSizeChanged(newVolumeSize))
		{
			voxelsPerMeter = newVolumeSize;
			scanner->ChangeVolumeSize(newVolumeSize);
			//kinectFusionScanner.ChangeVolumeSize(newVolumeSize);
		}

		if (GetTickCount() - scanTickLastFpsUpdate > _parentWindow.updateFpsCounterInMilliSeconds)
		{
			_parentWindow.SetFramesPerSecond(scanner->GetFramesPerSecond());
			//_parentWindow.SetFramesPerSecond(kinectFusionScanner.GetFramesPerSecond());
			scanTickLastFpsUpdate = GetTickCount();
		}
		std::wstring fusionStatus = scanner->GetAndResetStatus();
		//std::wstring fusionStatus = kinectFusionScanner.GetAndResetStatus();
		if (fusionStatus.length() > 0)
			_parentWindow.SetStatusBarMessage(fusionStatus);
		/*if (scanner->GetGpuMemory() > 0 && scanner->GetGpuMemory() != _parentWindow.GetGpuMemory())
		{
			_parentWindow.SetGpuMemory(scanner->GetGpuMemory());
		}*/
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case ScanWindowEvent::StateChange:
				pauseFusion = true;
				_parentWindow.ChangeState(WindowState::PlaneSelection);
				_parentWindow.SetAndShowHelpMessage(HelpMessage::PlaneSelectionHelp);
				scanner->PrepareMeshSave();
				//kinectFusionScanner.PrepareMeshSave();
				//_parentWindow.InitializeOpenGLScene(kinectFusionScanner.GetScannedMesh());
				//kinectFusionScanner.FinishMeshSave();
				boost::thread(&ScanWindow::FinishFusionScan, this, boost::ref(_parentWindow));
				//_parentWindow.InitializeOpenGLScene(kinectFusionScanner.FinishScan());
				break;
			case ScanWindowEvent::Reset:
				//do stuff
				break;
			}

			eventQueue.pop();
		}
	}

	void ScanWindow::FinishFusionScan(MainWindow& _parentWindow)
	{
		scanner->FinishMeshSave();
		_parentWindow.InitializeOpenGLScene(scanner->GetScannedVertices(), scanner->GetScannedTriangles());
		//_parentWindow.InitializeOpenGLScene(MeshContainer());
		//_parentWindow.InitializeOpenGLScene(kinectFusionScanner.GetScannedMesh());
		//kinectFusionScanner.FinishMeshSave();
	}

	void ScanWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		DebugUtility::DbgOut(L"ScanWindow::ProcessUI");
		// If it was the reset button clicked, clear the volume
		if (IDC_SCAN_BUTTON_RESET == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"RESET");
			scanner->ResetScan();
			//kinectFusionScanner.ResetScan();
			//m_processor->ResetReconstruction();
		}


		if (IDC_SCAN_BUTTON_DONE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			DebugUtility::DbgOut(L"DONE");
			eventQueue.push(ScanWindowEvent::StateChange);
			//parentWindow->ChangeState(PlaneSelection);
			//FinishScan(0);
		}
	}

	void ScanWindow::Resize(int parentWidth, int parentHeight)
	{
		SubWindow::Resize(parentWidth, parentHeight);
		MoveWindow(buttonScanDone, (int)(parentWidth - 0.135*parentWidth), (int)(height / 2 - (int)(0.12*parentWidth) / 2) - 28, (int)(0.12*parentWidth), (int)(0.12*parentWidth), true);
		
		int reconstructionHeight = parentHeight - 70;
		int reconstructionWidth = glm::min(width-(int)(0.30f*parentWidth),(int)((parentHeight - 70)*1.33f));

		//MoveWindow(reconstructionView, (int)(0.15f*parentWidth), 0, (int)(0.7f*parentWidth), parentHeight - 70, true);
		MoveWindow(reconstructionView, width/2 - reconstructionWidth/2, 0, reconstructionWidth, reconstructionHeight, true);
		MoveWindow(buttonScanReset, (int)(0.015f*parentWidth), (int)(height / 2 - (int)(0.12*parentWidth) / 2) - 28, (int)(0.12*parentWidth), (int)(0.12*parentWidth), true);
		/*MoveWindow(residualsView, 0, 0, 0.11*parentWidth, 0.11*parentWidth, true);
		MoveWindow(depthView, 0, 0.11*parentWidth, 0.11*parentWidth, 0.11*parentWidth, true);*/
	}

	glm::mat4 ScanWindow::GetCameraMatrix()
	{
		return cameraMatrix;
	}

	void ScanWindow::Show()
	{
		scanner->ResumeRendering();
		SubWindow::Show();
		DebugUtility::DbgOut(L"Showing ScanWindow");
	}

	void ScanWindow::Hide()
	{
		if (scanner != nullptr)
			scanner->PauseRendering();
		SubWindow::Hide();
		DebugUtility::DbgOut(L"Hiding ScanWindow");
	}

	void ScanWindow::PauseScan()
	{
		scanner->PauseProcessing(true);
		scanner->PauseIntegration();
		pauseFusion = true;
		DebugUtility::DbgOut(L"Pause Scanning");
	}

	void ScanWindow::UnpauseScan()
	{
		scanner->PauseProcessing(false);
		scanner->PauseIntegration();
		pauseFusion = false;
		DebugUtility::DbgOut(L"Unpause Scanning");
	}

	void ScanWindow::ResetScan()
	{
		scanner->ResetScan();
	}

	bool ScanWindow::HasVolumeSizeChanged(int _voxelsPerMeter)
	{
		return voxelsPerMeter != _voxelsPerMeter;
	}

	void ScanWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"ScanWindow::CleanUp()");

		scanUi.CleanUp();
		scanner->CleanUp();
		SubWindow::CleanUp();
	}

}
