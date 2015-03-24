#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")

#include "KeyState.h"
#include "OpenGLWindow.h"
#include "MainWindow.h"
#include "GraphicsControl.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "IFResources.h"
namespace InteractiveFusion {

	int mouseWheelDelta;

	HWND resetCameraButton;

	OpenGLWindow::OpenGLWindow()
	{
	}


	OpenGLWindow::~OpenGLWindow()
	{
	}

	void OpenGLWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		parentHandle = _parentHandle;
		hInstance = _hInstance;
		marginTop = _marginTop;
		marginBottom = _marginBottom;
		marginRight = _marginRight;
		marginLeft = _marginLeft;
		className = _className;

		WNDCLASSEX wc = { 0 };
		wc.cbSize = sizeof(WNDCLASSEX);
		wc.style = CS_HREDRAW | CS_VREDRAW;
		wc.lpfnWndProc = (WNDPROC)OpenGLWindow::MessageRouter;
		wc.cbClsExtra = 0;
		wc.cbWndExtra = 0;
		wc.hInstance = hInstance;
		wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
		wc.lpszMenuName = NULL;
		wc.lpszClassName = (LPCWSTR)className.c_str();
		wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
		ATOM ClassAtom = RegisterClassExW(&wc);



		RECT rRect;
		GetClientRect(parentHandle, &rRect);

		windowHandle = CreateWindowExW(0, (LPCTSTR)MAKELONG(ClassAtom, 0), 0, WS_CHILD | WS_CLIPCHILDREN | WS_CLIPSIBLINGS,
			(int)marginLeft, (int)marginTop, rRect.right - (int)marginRight, rRect.bottom - (int)(marginTop + marginBottom), parentHandle,
			NULL, hInstance, this);

		resetCameraButton = CreateWindowEx(0, L"BUTTON", L"Reset", WS_CHILD | WS_CLIPSIBLINGS | WS_VISIBLE | BS_ICON | BS_OWNERDRAW, 250, 50, 150, 50, windowHandle, (HMENU)IDC_OPENGL_RESETCAMERA, hInstance, 0);
		SetWindowPos(resetCameraButton, HWND_TOP, 0, 0, 0, 0, 0);
		buttonLayoutMap[resetCameraButton] = StyleSheet::GetInstance()->GetButtonLayoutParams(ButtonLayoutType::GlobalDefault);
		buttonLayoutMap[resetCameraButton].SetIcon(L"camera_icon.ico");


		Hide();

		

		//trashBmp = LoadBitmap(appInstance, MAKEINTRESOURCE(IDB_MODE_ARROW));
		//trashBmp_mask = LoadBitmap(appInstance, MAKEINTRESOURCE(IDB_MODE_ARROW_MASK));
	}

	HWND OpenGLWindow::GetHandle()
	{
		return windowHandle;
	}


	bool OpenGLWindow::IsCursorInWindow()
	{
		POINT pCur;
		GetCursorPos(&pCur);
		if (IsWindowVisible(windowHandle))
		{
			RECT rRect; GetWindowRect(windowHandle, &rRect);
			if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
				pCur.y >= rRect.top && pCur.y <= rRect.bottom)
				return true;
		}
		return false;
	}

	void OpenGLWindow::SetMargins(float _marginTop, float _marginBottom, float _marginRight, float _marginLeft)
	{
		marginTop = _marginTop;
		marginBottom = _marginBottom;
		marginRight = _marginRight;
		marginLeft = _marginLeft;
	}

	int OpenGLWindow::GetWidth()
	{
		return width;
	}

	int OpenGLWindow::GetHeight()
	{
		return height;
	}


	LRESULT CALLBACK OpenGLWindow::SubWindowProc(HWND windowHandle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		PAINTSTRUCT ps;
		switch (message)
		{
		case WM_PAINT:
			BeginPaint(windowHandle, &ps);
			EndPaint(windowHandle, &ps);
			break;
		case WM_LBUTTONDOWN:
			KeyState::SetMouseDown(true);
			
			break;
		case WM_LBUTTONUP:
			//KeyState::SetMouseDown(false);

			break;
		case WM_MOUSEWHEEL:
			mouseWheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
			break;
		}
		return SubWindow::SubWindowProc(windowHandle, message, wParam, lParam);
	}

	void OpenGLWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{
		if (IDC_OPENGL_RESETCAMERA == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
		{
			eventQueue.push(OpenGLWindowEvent::ResetCamera);
		}
	}

	void OpenGLWindow::HandleEvents(GraphicsControl& _glControl)
	{
		while (!eventQueue.empty())
		{
			int event = eventQueue.front();

			switch (event)
			{
			case OpenGLWindowEvent::ResetCamera:
				DebugUtility::DbgOut(L"OpenGLWindow::HandleEvents::ResetCamera");
				_glControl.ResetCamera();
				break;
			}

			eventQueue.pop();
		}
	}

	int OpenGLWindow::GetWheelDelta()
	{
		return mouseWheelDelta;
	}

	void OpenGLWindow::ResetWheelDeltaToZero()
	{
		mouseWheelDelta = 0;
	}

	void OpenGLWindow::Resize(int parentWidth, int parentHeight)
	{
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE");
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::parentWidth:", parentWidth);
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::parentHeight:", parentHeight);
		//RECT rRect;
		//GetClientRect(parentHandle, &rRect);

		SubWindow::Resize(parentWidth, parentHeight);
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::width:", width);
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::height:", height);
		/*xPosition = marginLeft;
		yPosition = marginTop;
		width = parentWidth - (marginLeft + marginRight);
		height = parentHeight - (marginTop + marginBottom);*/
		
		
		SetWindowPos(windowHandle, 0, xPosition, yPosition, width, height, SWP_NOZORDER | SWP_NOACTIVATE);
		glViewport(0, 0, width, height);

		MoveWindow(resetCameraButton, width - (int)(0.12f*height), (int)(0.88f*height), (int)(0.1f*height), (int)(0.1f*height), true);
		
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::lower left X:", xPosition);
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::lower left Y:", marginBottom);
		//DebugUtility::DbgOut(L"OpenGLWindow::RESIZE::lower left Y:", marginBottom);
		
		float ratio = (float)width / (float)height;

		//DebugUtility::DbgOut(L"OpenGLWindow::Resize::Ratio: ", ratio);
		
	}

	void OpenGLWindow::HideButtons()
	{
		ShowWindow(resetCameraButton, SW_HIDE);
	}

	void OpenGLWindow::ShowButtons()
	{
		ShowWindow(resetCameraButton, SW_SHOW);
		SetWindowPos(resetCameraButton, HWND_TOP, 0, 0, 0, 0, 0);
	}

	void OpenGLWindow::CleanUp()
	{
		DebugUtility::DbgOut(L"OpenGLWindow::CleanUp");
		SubWindow::CleanUp();
	}
}