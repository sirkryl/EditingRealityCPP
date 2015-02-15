#pragma once
#include <vector>
#include <Windows.h>
namespace InteractiveFusion {
	class GUIContainer
	{
	public:
		void Show();
		void Hide();
		void Activate();
		void Deactivate();
		void Add(HWND _element);
		void CleanUp();
		void Redraw();
		bool IsMouseInUI();
		bool ContainsHandle(HWND _handle);
	private:
		std::vector<HWND> elements;
	};

}