#include "OpenGLContext.h"

#include <gl/glew.h>
#include <gl/wglew.h>

namespace InteractiveFusion {

	bool OpenGLContext::bClassRegistered = false, OpenGLContext::bGlewInitialized = false;

	OpenGLContext::OpenGLContext()
	{
	}


	OpenGLContext::~OpenGLContext()
	{
	}

	bool OpenGLContext::InitGLEW(HINSTANCE _instance)
	{
		if (bGlewInitialized)
			return true;

		RegisterSimpleOpenGLClass(_instance);

		HWND hWndFake = CreateWindow(L"OPENGL", L"FAKE", WS_OVERLAPPEDWINDOW | WS_MAXIMIZE | WS_CLIPCHILDREN,
			0, 0, CW_USEDEFAULT, CW_USEDEFAULT, NULL,
			NULL, _instance, NULL);

		hDC = GetDC(hWndFake);

		// First, choose false pixel format

		PIXELFORMATDESCRIPTOR pfd;
		memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
		pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
		pfd.nVersion = 1;
		pfd.dwFlags = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
		pfd.iPixelType = PFD_TYPE_RGBA;
		pfd.cColorBits = 32;
		pfd.cDepthBits = 32;
		pfd.iLayerType = PFD_MAIN_PLANE;

		int pixelFormat = ChoosePixelFormat(hDC, &pfd);

		if (pixelFormat == 0)
			return false;

		if (!SetPixelFormat(hDC, pixelFormat, &pfd))return false;

		// Create the false, old style context (OpenGL 2.1 and before)

		HGLRC hRCFake = wglCreateContext(hDC);
		wglMakeCurrent(hDC, hRCFake);

		bool result = true;

		if (!bGlewInitialized)
		{
			if (glewInit() != GLEW_OK)
			{
				MessageBox(parentWindow, L"Couldn't initialize GLEW!", L"Fatal Error", MB_ICONERROR);
				result = false;
			}
			bGlewInitialized = true;
		}

		wglMakeCurrent(NULL, NULL);
		wglDeleteContext(hRCFake);
		DestroyWindow(hWndFake);

		return result;
	}

	bool OpenGLContext::InitOpenGL(HINSTANCE _instance, HWND _hWnd, int _majorVersion, int _minorVersion)
	{
		if (!InitGLEW(_instance))
			return false;

		parentWindow = _hWnd;
		parentInstance = _instance;

		hDC = GetDC(parentWindow);

		bool errorHappened = false;

		PIXELFORMATDESCRIPTOR pfd;

		if (_majorVersion <= 2)
		{
			memset(&pfd, 0, sizeof(PIXELFORMATDESCRIPTOR));
			pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
			pfd.nVersion = 1;
			pfd.dwFlags = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW;
			pfd.iPixelType = PFD_TYPE_RGBA;
			pfd.cColorBits = 32;
			pfd.cDepthBits = 32;
			pfd.iLayerType = PFD_MAIN_PLANE;

			int pixelFormat = ChoosePixelFormat(hDC, &pfd);
			if (pixelFormat == 0)return false;

			if (!SetPixelFormat(hDC, pixelFormat, &pfd))
				return false;

			// Create the old style context (OpenGL 2.1 and before)
			hRC = wglCreateContext(hDC);

			if (hRC)
				wglMakeCurrent(hDC, hRC);
			else 
				errorHappened = true;
		}
		else if (WGLEW_ARB_create_context && WGLEW_ARB_pixel_format)
		{
			const int pixelFormatAttributeList[] =
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
			int contextAttributes[] =
			{
				WGL_CONTEXT_MAJOR_VERSION_ARB, _majorVersion,
				WGL_CONTEXT_MINOR_VERSION_ARB, _minorVersion,
				WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
				0 // End of attributes list
			};

			int pixelFormat, numFormats;
			wglChoosePixelFormatARB(hDC, pixelFormatAttributeList, NULL, 1, &pixelFormat, (UINT*)&numFormats);

			// PFD seems to be only redundant parameter now
			if (!SetPixelFormat(hDC, pixelFormat, &pfd))
				return false;

			hRC = wglCreateContextAttribsARB(hDC, 0, contextAttributes);
			// If everything went OK
			if (hRC) 
				wglMakeCurrent(hDC, hRC);
			else 
				errorHappened = true;

		}
		else errorHappened = true;

		if (errorHappened)
		{
			// Generate error messages
			TCHAR errorMessage[255], errorTitle[255];
			wsprintf(errorMessage, L"OpenGL %d.%d is not supported! Please download latest GPU drivers!"), _majorVersion, _minorVersion;
			wsprintf(errorTitle, L"OpenGL %d.%d Not Supported"), _majorVersion, _minorVersion;
			MessageBox(parentWindow, errorMessage, errorTitle, MB_ICONINFORMATION);
			return false;
		}
		return true;
	}

	LRESULT CALLBACK msgHandlerSimpleOpenGLClass(HWND _hWnd, UINT _msg, WPARAM _wParam, LPARAM _lParam)
	{
		PAINTSTRUCT ps;
		switch (_msg)
		{
		case WM_PAINT:
			BeginPaint(_hWnd, &ps);
			EndPaint(_hWnd, &ps);
			break;

		default:
			return DefWindowProc(_hWnd, _msg, _wParam, _lParam);
		}
		return 0;
	}

	void OpenGLContext::RegisterSimpleOpenGLClass(HINSTANCE _instance)
	{
		if (bClassRegistered)
			return;
		
		WNDCLASSEX wc;

		wc.cbSize = sizeof(WNDCLASSEX);
		wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
		wc.lpfnWndProc = (WNDPROC)msgHandlerSimpleOpenGLClass;
		wc.cbClsExtra = 0; wc.cbWndExtra = 0;
		wc.hInstance = _instance;
		wc.hIcon = LoadIcon(_instance, MAKEINTRESOURCE(IDI_APPLICATION));
		wc.hIconSm = LoadIcon(_instance, MAKEINTRESOURCE(IDI_APPLICATION));
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)(COLOR_MENUBAR + 1);
		wc.lpszMenuName = NULL;
		wc.lpszClassName = L"OPENGL";

		RegisterClassEx(&wc);

		bClassRegistered = true;
	}

	bool OpenGLContext::SetVerticalSynchronization(bool _flag)
	{
		if (!wglSwapIntervalEXT)
			return false;

		if (_flag)
			wglSwapIntervalEXT(1);
		else 
			wglSwapIntervalEXT(0);

		return true;
	}

	void OpenGLContext::SwapBuffers()
	{
		::SwapBuffers(hDC);
	}

	void OpenGLContext::CleanUp()
	{
		wglMakeCurrent(NULL, NULL);
		wglDeleteContext(hRC);
		ReleaseDC(parentWindow, hDC);
		UnregisterSimpleOpenGLClass(parentInstance);
	}

	void OpenGLContext::UnregisterSimpleOpenGLClass(HINSTANCE _instance)
	{
		if (bClassRegistered)
		{
			UnregisterClass(L"OPENGL", _instance);
			bClassRegistered = false;
		}
	}
}
