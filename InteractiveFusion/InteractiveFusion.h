#pragma once
#include "OpenGLControl.h"
#include "common.h"
#include <KinectFusionExplorer.h>
enum WindowMode { MODE_PREPARE_SCANNING, MODE_SCANNING, MODE_SEGMENTATION, MODE_INTERACTION };

enum WindowState {INITIALIZING, BUFFERS, DEFAULT, SEGMENTATION, SEGMENTATION_FINISHED, WALL_SELECTION, SEGMENTATION_PREVIEW, SHOWSTATUS, SELECTION };

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

	//segmentation mode flags
	int segmentationMode = -1;
	bool previewMode = false;
	bool wallSelection = false;
	bool isWall = false;
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

	float wallThickness = 0.2f;
	float wallSmoothness = 0.5f;
	float clusterTolerance = 0.02f;
	bool showBB = false;
	bool wireFrameMode = false;
	//flag indicating whether a segmentation value has been changed or not
	bool segmentValuesChanged = false;
	

	//hole filling values
	int holeSize = 1;

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
	WindowState GetWindowState();
	void SetWindowState(WindowState wState);

	//cursor and/or mouse related methods
	bool IsMouseInHandle();
	bool IsMouseInUI(std::vector<HWND> handles);
	bool IsMouseInOpenGLWindow();

	void InitWallConfirmation();

	void ShowStatusBarMessage(string message);
	void ShowStatusBarMessage(wstring message);
	
	void ToggleDebugControls();

	void ResumeScanning();

	void SetBackgroundColor(int redValue, int greenValue, int blueValue);
	void UpdateWallSelectionValues();
	void UpdateSegmentationPreviewValues();
	void HideUI(std::vector<HWND> handles);
	void ShowUI(std::vector<HWND> handles);
	bool IsHandleInUI(HWND handle, std::vector<HWND> handles);
	void HideWholeUI();
	void RedrawManipulationButtons();

	void SetProgressionText(wstring text);
	void SetProgressionPercent(wstring percent);

	//thread related methods
	//bool StartOpenGLThread(HWND parentWin, HINSTANCE currHInstance, KinectFusionProcessor* proc);
	
	void TerminateThread();
	DWORD GetThreadID();
	HINSTANCE appInstance;
	DWORD threadId;
private:
	//application variables
	
	
	WindowMode mode;
	WindowState state;
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
void GLProcessUI(WPARAM wParam, LPARAM lParam);


void SetViewportStatusMessage(wstring message);
void SetViewportPercentMsg(wstring percent);

void InitializeGLUIControls();
void UpdateSliderText();
void UpdateGLHSliders();
void ResetEditControls();
void ResetSliders();
void MoveModeButtonsOnResize();
void MoveButtonsOnResize();

void ResetForResume();
void InitialLoading();

void ResetCameraPosition();
void ToggleCameraMode();

void RemoveSelectionColor();

void StartSegmentation();

void SelectWallObject();
void ResetWallObject();

//main render functions, defined in openGLRendering.cpp
void Initialize(LPVOID);
void Render(LPVOID);
void Release(LPVOID);

void StartOpenGLThread(int testMode);
void SetWindowMode(int wMode);