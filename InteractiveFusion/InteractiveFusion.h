#pragma once
#include "OpenGLControl.h"
#include "common.h"
#include <KinectFusionExplorer.h>
enum WindowMode { IF_MODE_PREPARE_SCAN, IF_MODE_SCAN, IF_MODE_SEGMENTATION, IF_MODE_PROCESSING, IF_MODE_INTERACTION };

enum Answer {ANSWER_NOTAVAILABLE, ANSWER_YES, ANSWER_NO};

enum WindowBusyState {IF_BUSYSTATE_BUSY, IF_BUSYSTATE_DEFAULT};

enum Reset {IF_RESET, IF_NO_RESET};


enum MeshQuality {QUALITY_VERYLOW, QUALITY_LOW, QUALITY_MEDIUM, QUALITY_HIGH, QUALITY_VERYHIGH};
class InteractiveFusion
{
public:
	
	MeshQuality meshQuality;
	HWND parent;
	HWND fusionHandle;
	HWND glWindowHandle;
	HWND glWindowParent;
	OpenGLControl glControl;
	CKinectFusionExplorer* fusionExplorer;
	HANDLE interactionThread;
	
	//background color
	float bgRed = 30.0f / 255.0f;
	float bgGreen = 30.0f / 255.0f;
	float bgBlue = 30.0f / 255.0f;

	//test mode flag
	int testMode = 0;

	bool previewMode = false;
	bool wallSelection = false;
	bool duplicationMode = false;
	//segmentation values
	int kSearchValue = 20;
	int minClusterSize = 1000;
	int maxClusterSize = 1000;
	int numberOfNeighbors = 20;
	double smoothnessThreshold = 100;
	double curvatureThreshold = 10;
	int maxComponentSize = 100;
	int carryDistance = 5;
	int removeClusterSize = 100;
	float wallThickness = 0.1f;
	float wallSmoothness = 0.05f;
	float clusterTolerance = 0.02f;
	bool showBB = false;
	bool wireFrameMode = false;
	//flag indicating whether a segmentation value has been changed or not
	bool segmentValuesChanged = false;
	

	//hole filling values
	int holeSize = 1000;

	//checkbox values
	bool helpingVisuals = false;
	bool estimateNormals = false;
	//bool freeCameraControls = false;
	bool colorSelection = false;
	bool snapToVertex = false;
	bool placeWithRaycast = false;
	//resolution
	int height = 768;
	int width = 1024;

	//mouse related variables
	short wheelDelta = 0;

	//calculate fps methods
	void ResetTimer();
	void UpdateTimer();
	float SpeedOptimizedFloat(float fVal);
	//window related methods
	bool CreateOpenGLWindow();
	void ReleaseOpenGL();
	void ShutdownWindow();
	LPCWSTR GetLastErrorStdStr();

	//application in general
	HINSTANCE GetInstance();

	WindowMode GetWindowMode();
	void SetWindowMode(WindowMode wMode);
	//WindowState GetWindowState();
	//void SetWindowState(WindowState wState);
	WindowBusyState GetWindowBusyState();
	void SetWindowBusyState(WindowBusyState bState);
	void SetAnswer(Answer ans);
	Answer GetAnswer();
	void SetReset(Reset res);
	Reset GetReset();
	bool DrawButton(WPARAM wParam, LPARAM lParam);
	//cursor and/or mouse related methods
	bool IsMouseInHandle();
	bool IsMouseInUI(std::vector<HWND> handles);
	bool IsMouseInOpenGLWindow();
	void MoveModeButtonsOnResize();
	void MoveButtonsOnResize();
	void InitWallConfirmation();

	void ShowStatusBarMessage(string message);
	void ShowStatusBarMessage(wstring message);
	
	void ToggleDebugControls();

	void ResumeScanning();

	void SetBackgroundColor(int redValue, int greenValue, int blueValue);
	void UpdateWallSelectionValues();
	void UpdateSegmentationPreviewValues();
	void UpdateProcessingValues();
	void HideUI(std::vector<HWND> handles);
	void ShowUI(std::vector<HWND> handles);
	bool IsHandleInUI(HWND handle, std::vector<HWND> handles);
	void HideWholeUI();
	void DeactivateUI(std::vector<HWND> handles);
	void DeactivateWholeUI();
	void ActivateUI(std::vector<HWND> handles);
	void ActivateWholeUI();
	void RedrawManipulationButtons();

	void SetProgressionText(wstring text);
	void SetProgressionPercent(wstring percent);

	void ProcessOpenGLUI(WPARAM wParam, LPARAM lParam);
	void ProcessDebugUI(WPARAM wParam, LPARAM lParam);
	void ProcessParentUI(WPARAM wParam, LPARAM lParam);
	void UpdateSegmentationUI();
	//thread related methods
	//bool StartOpenGLThread(HWND parentWin, HINSTANCE currHInstance, KinectFusionProcessor* proc);
	
	void TerminateThread();
	DWORD GetThreadID();
	HINSTANCE appInstance;
	DWORD threadId;
private:
	//application variables
	
	Reset reset = IF_NO_RESET;
	Answer answer = ANSWER_NOTAVAILABLE;
	WindowMode mode;
	//WindowState state;
	WindowBusyState busyState = IF_BUSYSTATE_DEFAULT;
	//fps related variables
	clock_t tLastFrame;
	float fFrameInterval;

	
	void DetermineMeshQuality();

	//thread related variables
	

	
};

extern InteractiveFusion openGLWin;

//open gl threads
int WINAPI GLViewportThreadMain();
LRESULT CALLBACK GLViewportProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK DebugDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK GLDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK SubEditProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam);
//process UI


void SetViewportStatusMessage(wstring message);
void SetViewportPercentMsg(wstring percent);

void InitializeGLUIControls();
void UpdateSliderText();
void UpdateGLHSliders();
void ResetEditControls();
void ResetSliders();

void ResetForResume();
void InitialLoading();

void ResetCameraPosition();
void ToggleCameraMode();

void RemoveSelectionColor();

void SelectWallObject();
void ResetWallObject();

//main render functions, defined in openGLRendering.cpp
void Initialize(LPVOID);
void Render(LPVOID);
void Release(LPVOID);

void StartOpenGLThread(int testMode);
void SetWindowMode(int wMode);