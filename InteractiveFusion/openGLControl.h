#pragma once
#include "common.h"
#include "KinectFusion.h"

class OpenGLControl
{
public:
	bool InitOpenGL(HINSTANCE hInstance, HWND a_hWnd, int iMajorVersion, int iMinorVersion, void(*a_ptrInitScene)(LPVOID), void
		(*a_ptrRenderScene)(LPVOID), void(*a_ptrReleaseScene)(LPVOID), KinectFusion* explo, LPVOID lpParam);
	//(*a_ptrRenderScene)(LPVOID), void(*a_ptrReleaseScene)(LPVOID), LPVOID lpParam);

	void ResizeOpenGLViewportFull(int width, int height);
	void ResizeOpenGLViewportFull();
	void SetProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar);
	void SetOrtho2D(int width, int height);

	glm::mat4 GetProjectionMatrix();
	glm::mat4 GetOrthoMatrix();
	glm::mat4 GetKinectViewMatrix();
	void SetCameraMatrix(glm::mat4 viewMatrix);

	void Render(LPVOID lpParam);
	void ReleaseOpenGLControl(LPVOID lpParam);

	static void RegisterSimpleOpenGLClass(HINSTANCE hInstance);
	static void UnregisterSimpleOpenGLClass(HINSTANCE hInstance);

	int GetOffSetRight();
	int GetOffSetBottom();

	void SetOffSetRight(int offset);
	void SetOffSetBottom(int offset);

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
	HWND hWnd;
	HGLRC hRC;
	KinectFusion* fusionExplorer;
	static bool bClassRegistered;
	static bool bGlewInitialized;
	int iMajorVersion, iMinorVersion;
	int offSetRight, offSetBottom = 0;
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