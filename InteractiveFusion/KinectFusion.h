//------------------------------------------------------------------------------
// <copyright file="KinectFusionExplorer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "common.h"
#include "ImageRenderer.h"
#include <NuiSensorChooserUI.h>
#include "KinectFusionParams.h"
#include "KinectFusionProcessor.h"

enum FusionState { START, COUNTDOWN, SCAN };

/// <summary>
/// KinectFusionExplorer sample.
/// </summary>
class KinectFusion
{
	static const DWORD          cStatusTimeoutInMilliseconds = 5000;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	KinectFusion();

	/// <summary>
	/// Destructor
	/// </summary>
	~KinectFusion();



	//int WINAPI CKinectFusionExplorer::ThreadMain(HWND parent);
	//static LRESULT CALLBACK CKinectFusionExplorer::RenderWindowProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

	//LRESULT CALLBACK FusionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK     MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>

	LRESULT CALLBACK            StartProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	LRESULT CALLBACK            DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	int                         Run(HWND parent, HINSTANCE hInstance, int nCmdShow, HWND &fusionHandle);

	FusionState GetWindowState();

	void SetWindowState(FusionState fState);

	void MoveUIOnResize();

	/// <summary>
	/// Initialize the UI controls
	/// </summary>
	void                        InitializeUIControls();
	
	/// <summary>
	/// Handle new UI interaction
	/// </summary>
	void                        ProcessUI(WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Update the internal variable values from the UI Horizontal sliders.
	/// </summary>
	void                        UpdateHSliders();

	void ResumeScan();
	void Hide();

	int GetVoxelsPerMeter();
	void StartScan();
	void FinishScan(int testMode);

	void HideAllUIElements();
	Matrix4 GetWorldToCameraTransform();

private:
	HWND                        m_hWnd;
	NuiSensorChooserUI*         m_pSensorChooserUI;

	FusionState state;
	/// <summary>
	/// Direct2D
	/// </summary>
	ImageRenderer*              m_pDrawReconstruction;
	ImageRenderer*              m_pDrawTrackingResiduals;
	ImageRenderer*              m_pDrawDepth;
	ID2D1Factory*               m_pD2DFactory;

	/// <summary>
	/// Main processing function
	/// </summary>
	void                        Update();

	void InitializeFusionUI();

	void HandleKeyInput();

	bool DrawButton(LPARAM lParam);

	bool DrawButtonSlider(LPARAM lParam);

	bool DrawSliderBackground(LPARAM lParam);
	void UpdateButtonSlider();
	void UpdateButtonSliderValue();
	void MoveButtonSlider(int pos);
	bool IsMouseInHandle(HWND handle);
	/// <summary>
	/// Save a mesh
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     SaveMeshFile(INuiFusionColorMesh *mesh, KinectFusionMeshTypes saveMeshType);

	/// <summary>
	/// Handle a completed frame from the Kinect Fusion processor.
	/// </summary>
	void                        HandleCompletedFrame();

	/// <summary>
	/// Set the status bar message
	/// </summary>
	/// <param name="szMessage">message to display</param>
	void                        SetStatusMessage(const WCHAR* szMessage);

	/// <summary>
	/// Set the frames-per-second message
	/// </summary>
	/// <param name="fFramesPerSecond">current frame rate</param>
	void                        SetFramesPerSecond(float fFramesPerSecond);

	/// <summary>
	/// Set the index of the GPU processor device initialized, or -1 for CPU
	/// </summary>
	/// <param name="gpuIndex">The index of the initialized GPU processor device, or -1 fo CPU.</param>
	void                        SetDeviceIndexInitialized(int deviceIndex);

	/// <summary>
	/// The reconstruction parameters passed to the processor
	/// </summary>
	KinectFusionParams          m_params;
	KinectFusionProcessor       m_processor;
	/// <summary>
	/// The reconstruction processor
	/// </summary>

	bool                        m_bUIUpdated;

	bool                        m_bInitializeError;
	bool                        m_bSavingMesh;
	KinectFusionMeshTypes       m_saveMeshFormat;
	bool                        m_bColorCaptured;

	/// <summary>
	/// Most recently reported frame rate
	/// </summary>
	float                       m_fFramesPerSecond;

	/// <summary>
	/// Time since last status message update
	/// </summary>
	DWORD                       m_tickLastStatus;
};

void StartKinectFusion(HWND parent, HINSTANCE hInstance, KinectFusion*& expl, HWND &fusionHandle);