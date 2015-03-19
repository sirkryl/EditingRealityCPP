#pragma once
#include "SubWindow.h"

namespace InteractiveFusion {

	class PrepareWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			ChangeSize = SubWindowEvent::Last,
			Last
		};
	};

	class PrepareWindow :
		public SubWindow
	{
	public:
		PrepareWindow();
		virtual ~PrepareWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow* _parentWindow);

		virtual void Resize(int parentWidth, int parentHeight);
		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		

	private:

		int voxelsPerMeter = 256;
		//const SubWindowEvent ChangeSize;

		HFONT countdownFont;

		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);

	//	bool IsMouseInHandle(HWND handle);

		void UpdateScanVolumeSize();

		int CountdownThread(MainWindow* _parentWindow);

	};
}

