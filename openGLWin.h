#pragma once
#include "openGLControl.h"

class OpenGLWin
{
public:
	HWND parent;
	HWND glWindowHandle;
	HWND glWindowParent;
	OpenGLControl glControl;
	KinectFusionProcessor* processor;
	HANDLE interactionThread;

	//test mode flag
	int testMode = 0;

	//segmentation mode flags
	int segmentationMode = -1;
	bool previewMode = false;

	//segmentation values
	int kSearchValue = 20;
	int minClusterSize = 100;
	int maxClusterSize = 1000;
	int numberOfNeighbors = 30;
	double smoothnessThreshold = 100;
	double curvatureThreshold = 10;
	int maxComponentSize = 100;

	//flag indicating whether a segmentation value has been changed or not
	bool segmentValuesChanged = false;

	//hole filling values
	int holeSize = 1;

	//checkbox values
	bool helpingVisuals = false;
	bool estimateNormals = false;
	bool freeCameraControls = false;
	bool colorSelection = false;

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
	void ShutdownWindow();
	LPCWSTR GetLastErrorStdStr();

	//application in general
	HINSTANCE GetInstance();

	//cursor and/or mouse related methods
	bool IsMouseInOpenGLWindow();

	//thread related methods
	bool StartOpenGLThread(HWND parentWin, HINSTANCE currHInstance, KinectFusionProcessor* proc);
	void TerminateThread();
	DWORD GetThreadID();

private:
	//application variables
	HINSTANCE appInstance;

	//fps related variables
	clock_t tLastFrame;
	float fFrameInterval;

	//thread related variables
	DWORD threadId;

	
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
LRESULT CALLBACK GLDlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK SubEditProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam);
//process UI
void GLProcessUI(WPARAM wParam, LPARAM lParam);
void SetBackgroundColor(int redValue, int greenValue, int blueValue);
void InitializeGLUIControls();
void UpdateSliderText();
void UpdateGLHSliders();
void ResetEditControls();
void ResetSliders();

//all defined in openGLRendering.cpp
void CombineAndExport();
void ResetCameraPosition();
void ToggleWireFrame();
void ToggleBoundingBoxes();
void FillHoles();
//void MLS();
void RemoveSmallComponents(int size);
void StartSegmentation();
//void ShowPCLViewer();
void CleanMesh();

void ToggleColorSelectedObject();
void SelectWallObject();
void ResetWallObject();

//main render functions, defined in openGLRendering.cpp
void Initialize(LPVOID);
void Render(LPVOID);
void Release(LPVOID);

