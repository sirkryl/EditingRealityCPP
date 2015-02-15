#pragma once
#include "SubWindow.h"

namespace InteractiveFusion {

	class PlaneSelectionWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			PlaneConfirmed = SubWindowEvent::Last,
			PlaneRejected,
			UpdateSegmentation,
			Last
		};
	};

	class PlaneSelectionWindow :
		public SubWindow
	{
	public:
		PlaneSelectionWindow();
		virtual ~PlaneSelectionWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow* _parentWindow);
		virtual void Activate();
		virtual void Deactivate();
		virtual void Show();
		virtual void Hide();

		virtual void Resize(int parentWidth, int parentHeight);
		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);


	private:

		

		void UpdatePlaneSelectionValues();
		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);

	};
}

