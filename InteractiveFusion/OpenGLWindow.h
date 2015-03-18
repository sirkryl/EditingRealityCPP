#pragma once
#include "SubWindow.h"

namespace InteractiveFusion {

	class OpenGLWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			ResetCamera = SubWindowEvent::Last,
			Last
		};
	};

	class GraphicsControl;
	class OpenGLWindow : 
		public SubWindow
	{
	public:
		OpenGLWindow();
		virtual ~OpenGLWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);


		virtual void Resize(int parentWidth, int parentHeight);

		void SetMargins(float _marginTop, float _marginBottom, float _marginRight, float _marginLeft);
		
		int GetWidth();
		int GetHeight();
		int GetWheelDelta();
		void ResetWheelDeltaToZero();
		HWND GetHandle();

		bool IsCursorInWindow();

		void ShowButtons();
		void HideButtons();

		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);
		void HandleEvents(GraphicsControl* _glControl);
	};
}