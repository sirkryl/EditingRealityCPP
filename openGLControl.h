#pragma once
#include "common.h"
#include "KinectFusionProcessor.h"

class OpenGLControl
{
public:
	bool InitOpenGL(HINSTANCE hInstance, HWND* a_hWnd, int iMajorVersion, int iMinorVersion, void(*a_ptrInitScene)(LPVOID), void(*a_ptrRenderScene)(LPVOID), void(*a_ptrReleaseScene)(LPVOID), KinectFusionProcessor* proc, LPVOID lpParam);
	
	void ResizeOpenGLViewportFull(int width, int height);
	void SetProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar);
	void SetOrtho2D(int width, int height);

	glm::mat4* GetProjectionMatrix();
	glm::mat4* GetOrthoMatrix();
	glm::mat4 GetKinectViewMatrix();

	void Render(LPVOID lpParam);
	void ReleaseOpenGLControl(LPVOID lpParam);

	static void RegisterSimpleOpenGLClass(HINSTANCE hInstance);
	static void UnregisterSimpleOpenGLClass(HINSTANCE hInstance);

	void SetOffSetWidth(int offsetX);
	void SetOffSetHeight(int offsetY);

	void MakeCurrent();
	void SwapBuffers();

	bool SetVerticalSynchronization(bool bEnabled);

	int GetFPS();

	int GetViewportWidth();
	int GetViewportHeight();

	OpenGLControl();

private:
	bool InitGLEW(HINSTANCE hInstance);

	int segmentationMode = -1;
	HDC hDC;
	HWND* hWnd;
	HGLRC hRC;
	KinectFusionProcessor* processor;
	static bool bClassRegistered;
	static bool bGlewInitialized;
	int iMajorVersion, iMinorVersion;
	int offSetWidth, offSetHeight = 0;
	// Used for FPS calculation
	int iFPSCount, iCurrentFPS;
	clock_t tLastSecond;

	// Matrix for perspective projection
	glm::mat4 mProjection;
	// Matrix for orthographic 2D projection
	glm::mat4 mOrtho;
	glm::mat4 mView;
	// Viewport parameters
	int iViewportWidth, iViewportHeight;

	void(*ptrInitScene)(LPVOID lpParam), (*ptrRenderScene)(LPVOID lpParam), (*ptrReleaseScene)(LPVOID lpParam), (*ptrSegmentation)();
};

LRESULT CALLBACK msgHandlerSimpleOpenGLClass(HWND, UINT, WPARAM, LPARAM);