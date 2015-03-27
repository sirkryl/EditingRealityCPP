#pragma once
#include "SubWindow.h"
#include "EnumDeclarations.h"
namespace InteractiveFusion {

	class HelpWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			OK = SubWindowEvent::Last,
			Last
		};
	};

	class HelpWindow :
		public SubWindow
	{
	public:
		HelpWindow();
		virtual ~HelpWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow& _parentWindow);

		virtual void Show();
		virtual void Hide();

		virtual void Resize(int parentWidth, int parentHeight);

		void SetHelpState(HelpMessage state);
		void SetDefaultMessage(WindowState state);
		void UpdateLineCount();
		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	protected:
		void NextHelpMessage();
		void UpdateMessageCount();
		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);

	private:

	};
}
