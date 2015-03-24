#include "SubWindow.h"
#include "MainWindow.h"
#include "StyleSheet.h"
#include "DebugUtility.h"

namespace InteractiveFusion {

	

	SubWindow::SubWindow()
	{
	}


	SubWindow::~SubWindow()
	{
	}

	void SubWindow::Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor)
	{
		parentHandle = _parentHandle;
		hInstance = _hInstance;
		marginTop = _marginTop;
		marginBottom = _marginBottom;
		marginRight = _marginRight;
		marginLeft = _marginLeft;
		className = _className;
		//parentWindow = reinterpret_cast<MainWindow*>(::GetWindowLongPtr(parentHandle, GWLP_USERDATA));

		backgroundBrush = CreateSolidBrush(RGB(_backgroundColor.r, _backgroundColor.g, _backgroundColor.b));
		uiFontBig = CreateFont(40, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());
		uiFontMedium = CreateFont(25, 0, 0, 0, FW_EXTRALIGHT, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());
		uiFontSmall = CreateFont(20, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());


		WNDCLASSEX wc = { 0 };
		wc.cbSize = sizeof(WNDCLASSEX);
		wc.lpfnWndProc = (WNDPROC)SubWindow::MessageRouter;
		wc.style = 0;
		wc.hInstance = hInstance;
		wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = backgroundBrush;
		wc.lpszMenuName = NULL;
		wc.lpszClassName = (LPCWSTR)className.c_str();
		//wc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
		classAtom = RegisterClassExW(&wc);

		RECT rRect;
		GetClientRect(parentHandle, &rRect);

		xPosition = (int)(marginLeft*rRect.right);
		yPosition = (int)(marginTop*rRect.bottom);
		width = rRect.right - ((int)(marginLeft*rRect.right) + (int)(marginRight*rRect.right));
		height = rRect.bottom - ((int)(marginTop*rRect.bottom) + (int)(marginBottom*rRect.bottom));

		windowHandle = CreateWindowExW(0, (LPCTSTR)MAKELONG(classAtom, 0), 0, WS_CHILD | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
			xPosition, yPosition, width, height, parentHandle,
			NULL, hInstance, this);
		Hide();
		//windowHandle = CreateWindowEx(0, L"", L"", WS_CHILD | WS_VISIBLE, marginTop, y, width, height, parentHandle, 0, hInstance, 0);
	}

	void SubWindow::Show()
	{
		isVisible = true;
		ShowWindow(windowHandle, SW_SHOW);
	}

	void SubWindow::Hide()
	{
		isVisible = false;
		ShowWindow(windowHandle, SW_HIDE);
	}

	bool SubWindow::IsVisible()
	{
		return isVisible;
		return IsWindowVisible(windowHandle);
	}

	void SubWindow::Activate()
	{
		isActive = true;
		EnableWindow(windowHandle, true);
	}

	void SubWindow::Deactivate()
	{
		isActive = false;
		EnableWindow(windowHandle, false);
	}

	bool SubWindow::IsActive()
	{
		return isActive;
		//return IsWindowEnabled(windowHandle);
	}

	void SubWindow::Resize(int parentWidth, int parentHeight)
	{
		xPosition = (int)(marginLeft*parentWidth);
		yPosition = (int)(marginTop*parentHeight);
		width = parentWidth - ((int)(marginLeft*parentWidth) + (int)(marginRight*parentWidth));
		height = parentHeight - ((int)(marginTop*parentHeight) + (int)(marginBottom*parentHeight));
		MoveWindow(windowHandle, xPosition, yPosition, width, height, true);
	}

	void SubWindow::ProcessUI(WPARAM wParam, LPARAM lParam)
	{

	}

	void SubWindow::HandleEvents(MainWindow& _parentWindow)
	{

	}

	void SubWindow::CleanUp()
	{
		//if (windowHandle)
			//DestroyWindow(windowHandle);
		//if (parentWindow)
		//	parentWindow = NULL;
		DebugUtility::DbgOut(L"SubWindow::CleanUp()");
		for (auto style : buttonLayoutMap)
			style.second.CleanUp();
		buttonLayoutMap.clear();
		DeleteObject(uiFontSmall);
		DeleteObject(uiFontMedium);
		DeleteObject(uiFontBig);
		DeleteObject(backgroundBrush);
		UnregisterClass((LPCWSTR)className.c_str(), hInstance);
	}

	LRESULT CALLBACK    SubWindow::MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		SubWindow* pThis = nullptr;

		if (WM_CREATE == message)
		{
			pThis = (SubWindow*)((LPCREATESTRUCT)lParam)->lpCreateParams;
			SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)pThis);
		}
		else
		{
			pThis = reinterpret_cast<SubWindow*>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
		}

		if (pThis)
		{
			return pThis->SubWindowProc(hWnd, message, wParam, lParam);
		}
		else return DefWindowProc(hWnd, message, wParam, lParam);

		return 0;
	}

	LRESULT CALLBACK SubWindow::SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
	{
		PAINTSTRUCT ps;
		switch (message)
		{
		case WM_DESTROY:
			DebugUtility::DbgOut(L"SubWindowProc::WM_DESTROY");
			CleanUp();
			return DefWindowProc(hWnd, message, wParam, lParam);
			break;
		case WM_COMMAND:
			ProcessUI(wParam, lParam);
			break;
		case WM_CTLCOLORSTATIC:
		{
			HDC hdc = reinterpret_cast<HDC>(wParam);
			SetBkMode((HDC)wParam, TRANSPARENT);
			SetTextColor(hdc, RGB(StyleSheet::GetInstance()->GetDefaultTextColor().r, StyleSheet::GetInstance()->GetDefaultTextColor().g, StyleSheet::GetInstance()->GetDefaultTextColor().b));

			return (LRESULT)backgroundBrush;
		}
		break;
		case WM_SIZE:
			//Resize();
			return DefWindowProc(hWnd, message, wParam, lParam);
			break;
		case WM_DRAWITEM:
		{
			std::unordered_map<HWND, ButtonLayout>::iterator it = buttonLayoutMap.find(((LPDRAWITEMSTRUCT)lParam)->hwndItem);
			if (it != buttonLayoutMap.end()) {
				return it->second.Draw(lParam);
			}
		}
		break;
		case WM_LBUTTONDOWN:
			mouseDown = true;
			break;
		case WM_LBUTTONUP:
			mouseDown = false;
			break;
		case WM_MOUSEMOVE:
			if (mouseDown)
			{
				//if (IsMouseInHandle(hButtonSlider))
				//	UpdateButtonSlider();
			}
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		return 0;
	}

	ATOM SubWindow::GetClassAtom()
	{
		return classAtom;
	}

	bool SubWindow::IsMouseInHandle(HWND handle)
	{
		POINT pCur;
		GetCursorPos(&pCur);

		RECT rRect; GetWindowRect(handle, &rRect);
		if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
			pCur.y >= rRect.top && pCur.y <= rRect.bottom)
			return true;

		return false;
	}


}