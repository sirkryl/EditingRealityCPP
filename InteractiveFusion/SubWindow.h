#pragma once
#include "ButtonLayout.h"
#include <queue>
#include <Windows.h>
#include <string>
#include <unordered_map>
namespace InteractiveFusion {

	class SubWindowEvent {
	public:
		enum Type {
			StateChange,
			Start,
			HelpChanged,
			Last
		};
	};
	
	class MainWindow;
	class SubWindow
	{
	public:
		SubWindow();
		virtual ~SubWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);
		virtual void Show();
		virtual void Hide();
		bool IsVisible();
		virtual void Activate();
		virtual void Deactivate();
		bool IsActive();

		virtual void HandleEvents(MainWindow& _parentWindow);
		virtual void Resize(int parentWidth, int parentHeight);
		virtual void CleanUp();

		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	protected:

		bool isVisible = false;
		bool isActive = true;
		bool helpActive = true;
		HWND parentHandle;
		HWND windowHandle;

		HBRUSH backgroundBrush;
		
		//enum SubWindowEvent { StateChange, LastEvent };

		HINSTANCE hInstance;
		std::wstring className;
		ATOM classAtom;
		HFONT countdownFont;
		HFONT uiFontBig, uiFontMedium, uiFontSmall;

		std::queue<int> eventQueue;
		//MainWindow* parentWindow;
		float marginTop, marginBottom, marginRight, marginLeft;
		int xPosition, yPosition, width, height;
		std::unordered_map<HWND, ButtonLayout> buttonLayoutMap;
		bool mouseDown = false;

		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);

		bool IsMouseInHandle(HWND handle);

		void MoveButtonSlider(int pos);

		static LRESULT CALLBACK    MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		
		ATOM GetClassAtom();
	};
}

