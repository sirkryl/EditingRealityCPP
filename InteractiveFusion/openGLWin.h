#pragma once
#include "openGLControl.h"

enum WindowMode { SCANNING, INTERACTION };

enum WindowState {INITIALIZING, BUFFERS, DEFAULT, SEGMENTATION, SEGMENTATION_FINISHED, WALL_SELECTION, SEGMENTATION_PREVIEW, SHOWSTATUS, SELECTION };

class OpenGLWin
{
public:
	

	WindowMode mode;
	WindowState state;
	HWND parent;
	HWND fusionHandle;
	HWND glWindowHandle;
	HWND glWindowParent;
	OpenGLControl glControl;
	KinectFusionProcessor* processor;
	HANDLE interactionThread;

	//background color
	float bgRed = 0.0f;
	float bgGreen = 0.0f;
	float bgBlue = 0.0f;

	//test mode flag
	int testMode = 0;

	//segmentation mode flags
	int segmentationMode = -1;
	bool previewMode = false;
	bool wallSelection = false;
	bool isWall = false;
	//segmentation values
	int kSearchValue = 20;
	int minClusterSize = 1000;
	int maxClusterSize = 1000;
	int numberOfNeighbors = 20;
	double smoothnessThreshold = 100;
	double curvatureThreshold = 10;
	int maxComponentSize = 100;
	int carryDistance = 5;


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
	void SetViewportStatusMessage(wstring message);
	//window related methods
	bool CreateOpenGLWindow();
	void ReleaseOpenGL();
	void ShutdownWindow();
	LPCWSTR GetLastErrorStdStr();

	//application in general
	HINSTANCE GetInstance();

	//cursor and/or mouse related methods
	bool IsMouseInHandle();
	bool IsMouseInOpenGLWindow();

	void ShowConfirmationButtons(bool flag);
	void InitWallConfirmation();

	void ShowStatusBarMessage(string message);
	void ShowStatusBarMessage(wstring message);
	
	void ToggleDebugControls();

	void SetBackgroundColor(int redValue, int greenValue, int blueValue);

	//thread related methods
	//bool StartOpenGLThread(HWND parentWin, HINSTANCE currHInstance, KinectFusionProcessor* proc);
	
	void TerminateThread();
	DWORD GetThreadID();
	HINSTANCE appInstance;
	DWORD threadId;
private:
	//application variables
	

	//fps related variables
	clock_t tLastFrame;
	float fFrameInterval;

	//thread related variables
	

	
};

namespace Keys
{
	int GetKeyState(int key);
	int GetKeyStateOnce(int key);
	extern TCHAR kp[256];
}

extern OpenGLWin openGLWin;

//open gl threads
int WINAPI GLViewportThreadMain();
LRESULT CALLBACK GLViewportProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK DebugDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK GLDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK SubEditProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam);
//process UI
void GLProcessUI(WPARAM wParam, LPARAM lParam);

void InitializeGLUIControls();
void UpdateSliderText();
void UpdateGLHSliders();
void ResetEditControls();
void ResetSliders();
void MoveButtonsOnResize();

void LoadInput();
void CombineAndExport();

void ResetCameraPosition();
void ToggleCameraMode();

void RemoveSelectionColor();

void FillHoles(int holeSize);
void RemoveSmallComponents(int size);
void CleanMesh();
void StartSegmentation();

void SelectWallObject();
void ResetWallObject();

//main render functions, defined in openGLRendering.cpp
void Initialize(LPVOID);
void Render(LPVOID);
void Release(LPVOID);

void StartOpenGLThread(KinectFusionProcessor* proc, int testMode);