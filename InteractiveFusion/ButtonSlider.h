#pragma once
#include <Windows.h>
#include <string>
#include "ButtonLayout.h"

namespace InteractiveFusion {
	class ButtonSlider
	{
	public:
		ButtonSlider();
		~ButtonSlider();

		void HandleLeftMouseButtonDown();
		void HandleLeftMouseButtonUp();
		bool HandleMouseMove();

		void SetValue(int _value);
		void SetLimits(int _lowerBound, int _upperBound);
		int GetValue();
		void SetStep(int _step);

		void Resize(int _x, int _y, int _width, int _height);
		bool Draw(HWND _handle, LPARAM _lParam);
		bool HasHandle(HWND _handle);
		void Initialize(HWND _parentHandle, HINSTANCE _hInstance);

		void SetLayout(ButtonLayoutParams _sliderButtonLayout, ButtonLayoutParams _sliderBackgroundLayout);

		void Show();
		void Hide();
		void CleanUp();

	protected:
		HWND parentWindow;
		HWND hSliderButton;
		HWND hSliderBackground;
		int sliderPos = -1;
		POINT sliderAnchor;
		bool sliderDown = false;
		int lowerBound = 0;
		int upperBound = 100;
		int sliderValue;
		int step = 1;
		int x, y, width, height;
		bool repositioningNecessary = false;

		Gradient defaultGradient;
		Gradient pressedGradient;
		ButtonLayout sliderButtonLayout;
		ButtonLayout sliderBackgroundLayout;

		void Reposition();
		void MoveHandle();
		void Redraw();

		bool IsMouseInHandle();
	};
}