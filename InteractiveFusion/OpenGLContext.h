#pragma once
#include <Windows.h>
namespace InteractiveFusion {
	class OpenGLContext
	{
	public:
		OpenGLContext();
		~OpenGLContext();

		bool InitOpenGL(HINSTANCE _instance, HWND _hWnd, int _majorVersion, int _minorVersion);
		
		void SwapBuffers();
		bool SetVerticalSynchronization(bool _flag);

		void CleanUp();

	private:
		HDC hDC;
		HGLRC hRC;
		HWND parentWindow;
		HINSTANCE parentInstance;

		static bool bClassRegistered;
		static bool bGlewInitialized;

		bool InitGLEW(HINSTANCE _instance);
		void RegisterSimpleOpenGLClass(HINSTANCE _instance);
		void UnregisterSimpleOpenGLClass(HINSTANCE _instance);
	};

	LRESULT CALLBACK msgHandlerSimpleOpenGLClass(HWND _hWnd, UINT _msg, WPARAM _wParam, LPARAM _lParam);
}
