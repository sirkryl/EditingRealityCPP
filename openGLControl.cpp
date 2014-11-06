#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")

#include "common.h"
#include "openGLControl.h"
#include <gl/wglew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
bool OpenGLControl::bClassRegistered = false, OpenGLControl::bGlewInitialized = false;

OpenGLControl::OpenGLControl()
{
	iFPSCount = 0;
	iCurrentFPS = 0;
}

/*-----------------------------------------------

Name:	InitGLEW

Params:	none

Result:	Creates fake window and OpenGL rendering
		context, within which GLEW is initialized.

/*---------------------------------------------*/

bool OpenGLControl::InitGLEW(HINSTANCE hInstance)
{
	if(bGlewInitialized)return true;

	RegisterSimpleOpenGLClass(hInstance);

	HWND hWndFake = CreateWindow(_T("OPENGL"), _T("FAKE"), WS_OVERLAPPEDWINDOW | WS_MAXIMIZE | WS_CLIPCHILDREN,
		0, 0, CW_USEDEFAULT, CW_USEDEFAULT, NULL,
		NULL, hInstance, NULL);

	hDC = GetDC(hWndFake);

	// First, choose false pixel format
	
	PIXELFORMATDESCRIPTOR pfd;
	memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
	pfd.nSize		= sizeof(PIXELFORMATDESCRIPTOR);
	pfd.nVersion   = 1;
	pfd.dwFlags    = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 32;
	pfd.cDepthBits = 32;
	pfd.iLayerType = PFD_MAIN_PLANE;
 
	int iPixelFormat = ChoosePixelFormat(hDC, &pfd);
	if (iPixelFormat == 0)return false;

	if(!SetPixelFormat(hDC, iPixelFormat, &pfd))return false;

	// Create the false, old style context (OpenGL 2.1 and before)

	HGLRC hRCFake = wglCreateContext(hDC);
	wglMakeCurrent(hDC, hRCFake);

	bool bResult = true;

	if(!bGlewInitialized)
	{
		if(glewInit() != GLEW_OK)
		{
			MessageBox(*hWnd, _T("Couldn't initialize GLEW!"), _T("Fatal Error"), MB_ICONERROR);
			bResult = false;
		}
		bGlewInitialized = true;
	}

	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRCFake);
	DestroyWindow(hWndFake);

	return bResult;
}

/*-----------------------------------------------

Name:	InitOpenGL

Params:	hInstance - application instance
		a_hWnd - window to init OpenGL into
		a_iMajorVersion - Major version of OpenGL
		a_iMinorVersion - Minor version of OpenGL
		a_initScene - pointer to init function
		a_openGLRendering - pointer to render function
		a_releaseScene - optional parameter of release
						function

Result:	Initializes OpenGL rendering context
		of specified version. If succeeds,
		returns true.

/*---------------------------------------------*/

bool OpenGLControl::InitOpenGL(HINSTANCE hInstance, HWND* a_hWnd, int iMajorVersion, int iMinorVersion, void(*a_ptrInitScene)(LPVOID), void(*a_ptrRenderScene)(LPVOID), void(*a_ptrReleaseScene)(LPVOID), KinectFusionProcessor* proc, LPVOID lpParam)
{
	if(!InitGLEW(hInstance))return false;
	processor = proc;
	hWnd = a_hWnd;
	hDC = GetDC(*hWnd);
	bool bError = false;
	PIXELFORMATDESCRIPTOR pfd;

	if(iMajorVersion <= 2)
	{
		memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
		pfd.nSize		= sizeof(PIXELFORMATDESCRIPTOR);
		pfd.nVersion   = 1;
		pfd.dwFlags    = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
		pfd.iPixelType = PFD_TYPE_RGBA;
		pfd.cColorBits = 32;
		pfd.cDepthBits = 32;
		pfd.iLayerType = PFD_MAIN_PLANE;
 
		int iPixelFormat = ChoosePixelFormat(hDC, &pfd);
		if (iPixelFormat == 0)return false;

		if(!SetPixelFormat(hDC, iPixelFormat, &pfd))return false;

		// Create the old style context (OpenGL 2.1 and before)
		hRC = wglCreateContext(hDC);
		if(hRC)wglMakeCurrent(hDC, hRC);
		else bError = true;
	}
	else if(WGLEW_ARB_create_context && WGLEW_ARB_pixel_format)
	{
		const int iPixelFormatAttribList[] =
		{
			WGL_DRAW_TO_WINDOW_ARB, GL_TRUE,
			WGL_SUPPORT_OPENGL_ARB, GL_TRUE,
			WGL_DOUBLE_BUFFER_ARB, GL_TRUE,
			WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB,
			WGL_COLOR_BITS_ARB, 32,
			WGL_DEPTH_BITS_ARB, 24,
			WGL_STENCIL_BITS_ARB, 8,
			0 // End of attributes list
		};
		int iContextAttribs[] =
		{
			WGL_CONTEXT_MAJOR_VERSION_ARB, iMajorVersion,
			WGL_CONTEXT_MINOR_VERSION_ARB, iMinorVersion,
			WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
			0 // End of attributes list
		};

		int iPixelFormat, iNumFormats;
		wglChoosePixelFormatARB(hDC, iPixelFormatAttribList, NULL, 1, &iPixelFormat, (UINT*)&iNumFormats);

		// PFD seems to be only redundant parameter now
		if(!SetPixelFormat(hDC, iPixelFormat, &pfd))return false;

		hRC = wglCreateContextAttribsARB(hDC, 0, iContextAttribs);
		// If everything went OK
		if(hRC) wglMakeCurrent(hDC, hRC);
		else bError = true;

	}
	else bError = true;
	
	if(bError)
	{
		// Generate error messages
		TCHAR sErrorMessage[255], sErrorTitle[255];
		wsprintf(sErrorMessage, _T("OpenGL %d.%d is not supported! Please download latest GPU drivers!"), iMajorVersion, iMinorVersion);
		wsprintf(sErrorTitle, _T("OpenGL %d.%d Not Supported"), iMajorVersion, iMinorVersion);
		MessageBox(*hWnd, sErrorMessage, sErrorTitle, MB_ICONINFORMATION);
		return false;
	}
	ptrRenderScene = a_ptrRenderScene;
	ptrInitScene = a_ptrInitScene;
	ptrReleaseScene = a_ptrReleaseScene;

	if(ptrInitScene != NULL)ptrInitScene(lpParam);
	return true;
}

/*-----------------------------------------------

Name:	ResizeOpenGLViewportFull

Params:	none

Result:	Resizes viewport to full window.

/*---------------------------------------------*/

void OpenGLControl::ResizeOpenGLViewportFull(int width, int height)
{
	if(hWnd == NULL)return;
	RECT rRect; GetClientRect(*hWnd, &rRect);
	glViewport(0, 0, rRect.right, rRect.bottom);
	//glViewport(0, 0, width, height);
	//iViewportWidth = width;
	//iViewportHeight = height;
	iViewportWidth = rRect.right;
	iViewportHeight = rRect.bottom;
}

/*-----------------------------------------------

Name:	SetProjection3D

Params:	fFOV - field of view angle
		fAspectRatio - aspect ration of width/height
		fNear, fFar - distance of near and far clipping plane

Result:	Calculates projection matrix and stores it.

/*---------------------------------------------*/

void OpenGLControl::SetProjection3D(float fFOV, float fAspectRatio, float fNear, float fFar)
{
	mProjection = glm::perspective(fFOV, fAspectRatio, fNear, fFar);
}

/*-----------------------------------------------

Name:	SetOrtho2D

Params:	width - width of window
				height - height of window

Result:	Calculates ortho 2D projection matrix and stores it.

/*---------------------------------------------*/

void OpenGLControl::SetOrtho2D(int width, int height)
{
	mOrtho = glm::ortho(0.0f, float(width), 0.0f, float(height));
}

/*-----------------------------------------------

Name:	GetProjectionMatrix()

Params:	none

Result:	Retrieves pointer to projection matrix.

/*---------------------------------------------*/

glm::mat4* OpenGLControl::GetProjectionMatrix()
{
	return &mProjection;
}

/*-----------------------------------------------

Name:	GetOrthoMatrix()

Params:	none

Result:	Retrieves pointer to ortho matrix.

/*---------------------------------------------*/

glm::mat4* OpenGLControl::GetOrthoMatrix()
{
	return &mOrtho;
}

glm::mat4 OpenGLControl::GetKinectViewMatrix()
{
	glm::mat4 glmLeftHandedCamTransform(1.0f);

	Matrix4 leftHandedCamTransform = processor->GetWorldToCameraTransform();
	glmLeftHandedCamTransform[0] = glm::vec4(leftHandedCamTransform.M11, leftHandedCamTransform.M12, leftHandedCamTransform.M13, leftHandedCamTransform.M14);
	glmLeftHandedCamTransform[1] = glm::vec4(leftHandedCamTransform.M21, leftHandedCamTransform.M22, leftHandedCamTransform.M23, leftHandedCamTransform.M24);
	glmLeftHandedCamTransform[2] = glm::vec4(leftHandedCamTransform.M31, leftHandedCamTransform.M32, leftHandedCamTransform.M33, leftHandedCamTransform.M34);
	glmLeftHandedCamTransform[3] = glm::vec4(leftHandedCamTransform.M41, leftHandedCamTransform.M42, leftHandedCamTransform.M43, leftHandedCamTransform.M44);

	/*glm::mat4 conversionMatrix(1.0f);
	conversionMatrix[1] = glm::vec4(0.0f, -1.0f, 0.0f, 0.0f);
	conversionMatrix[2] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);

	glm::mat4 glmRightHandedCamTransform(1.0f);
	glmRightHandedCamTransform = glm::inverse(conversionMatrix) * glmLeftHandedCamTransform * conversionMatrix;*/

	/* this is for flipped y,z axis
	Matrix4 leftHandedCamTransform = processor->GetWorldToCameraTransform();
	glmLeftHandedCamTransform[0] = glm::vec4(leftHandedCamTransform.M11, -leftHandedCamTransform.M12, -leftHandedCamTransform.M13, leftHandedCamTransform.M14);
	glmLeftHandedCamTransform[1] = glm::vec4(leftHandedCamTransform.M21, -leftHandedCamTransform.M22, -leftHandedCamTransform.M23, leftHandedCamTransform.M24);
	glmLeftHandedCamTransform[2] = glm::vec4(leftHandedCamTransform.M31, -leftHandedCamTransform.M32, -leftHandedCamTransform.M33, leftHandedCamTransform.M34);
	glmLeftHandedCamTransform[3] = glm::vec4(leftHandedCamTransform.M41, -leftHandedCamTransform.M42, -leftHandedCamTransform.M43, leftHandedCamTransform.M44);
	return glmRightHandedCamTransform;*/

	mView = glmLeftHandedCamTransform;
	return mView;
}

/*-----------------------------------------------

Name:	RegisterSimpleOpenGLClass

Params:	hInstance - application instance

Result:	Registers simple OpenGL window class.

/*---------------------------------------------*/
void OpenGLControl::RegisterSimpleOpenGLClass(HINSTANCE hInstance)
{
	if(bClassRegistered)return;
	WNDCLASSEX wc;

	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style =  CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
	wc.lpfnWndProc = (WNDPROC) msgHandlerSimpleOpenGLClass;
	wc.cbClsExtra = 0; wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_APPLICATION));
	wc.hIconSm = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_APPLICATION));
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)(COLOR_MENUBAR+1);
	wc.lpszMenuName = NULL;
	wc.lpszClassName = _T("OPENGL");

	RegisterClassEx(&wc);

	bClassRegistered = true;
}

/*-----------------------------------------------

Name:	UnregisterSimpleOpenGLClass

Params:	hInstance - application instance

Result:	Unregisters simple OpenGL window class.

/*---------------------------------------------*/
void OpenGLControl::UnregisterSimpleOpenGLClass(HINSTANCE hInstance)
{
	if(bClassRegistered)
	{
		UnregisterClass(_T("OPENGL"), hInstance);
		bClassRegistered = false;
	}
}

/*-----------------------------------------------

Name:	msgHandlerSimpleOpenGLClass

Params:	whatever

Result:	Handles messages from windows that use
		simple OpenGL class.

/*---------------------------------------------*/

LRESULT CALLBACK msgHandlerSimpleOpenGLClass(HWND hWnd, UINT uiMsg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;
	switch(uiMsg)
	{
		case WM_PAINT:									
			BeginPaint(hWnd, &ps);							
			EndPaint(hWnd, &ps);					
			break;

		default:
			return DefWindowProc(hWnd, uiMsg, wParam, lParam); // Default window procedure
	}
	return 0;
}

/*-----------------------------------------------

Name:	SwapBuffers

Params:	none

Result:	Swaps back and front buffer.

/*---------------------------------------------*/

void OpenGLControl::SwapBuffers()
{
	::SwapBuffers(hDC);
}

/*-----------------------------------------------

Name:	MakeCurrent

Params:	none

Result:	Makes current device and OpenGL rendering
		context to those associated with OpenGL
		control.

/*---------------------------------------------*/

void OpenGLControl::MakeCurrent()
{
	wglMakeCurrent(hDC, hRC);
}

/*-----------------------------------------------

Name:	Render

Params:	lpParam - pointer to whatever you want

Result:	Calls previously set render function.

/*---------------------------------------------*/

void OpenGLControl::Render(LPVOID lpParam)
{
	clock_t tCurrent = clock();
	if( (tCurrent-tLastSecond) >= CLOCKS_PER_SEC)
	{
		tLastSecond += CLOCKS_PER_SEC;
		iFPSCount = iCurrentFPS;
		iCurrentFPS = 0;
	}
	if(ptrRenderScene)ptrRenderScene(lpParam);
	iCurrentFPS++;
}


/*-----------------------------------------------

Name:	ReleaseOpenGLControl

Params:	lpParam - pointer to whatever you want

Result:	Calls previously set release function
		and deletes rendering context.

/*---------------------------------------------*/

void OpenGLControl::ReleaseOpenGLControl(LPVOID lpParam)
{
	if(ptrReleaseScene)ptrReleaseScene(lpParam);

	wglMakeCurrent(NULL, NULL);
	wglDeleteContext(hRC);
	ReleaseDC(*hWnd, hDC);

	hWnd = NULL;
}

/*-----------------------------------------------

Name:	SetVerticalSynchronization

Params: bEnabled - whether to enable V-Sync

Result:	Guess what it does :)

/*---------------------------------------------*/

bool OpenGLControl::SetVerticalSynchronization(bool bEnabled)
{
	if(!wglSwapIntervalEXT)return false;

	if(bEnabled)wglSwapIntervalEXT(1);
	else wglSwapIntervalEXT(0);

	return true;
}

/*-----------------------------------------------

Name:	Getters

Params:	none

Result:	... They get something :D

/*---------------------------------------------*/

int OpenGLControl::GetFPS()
{
	return iFPSCount;
}

int OpenGLControl::GetViewportWidth()
{
	return iViewportWidth;
}

int OpenGLControl::GetViewportHeight()
{
	return iViewportHeight;
}