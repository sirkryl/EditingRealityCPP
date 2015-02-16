#include "ButtonSlider.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "GUIContainer.h"
namespace InteractiveFusion {

	GUIContainer sliderGui;

	ButtonSlider::ButtonSlider()
	{
	}


	ButtonSlider::~ButtonSlider()
	{
	}

	void ButtonSlider::Initialize(HWND _parentHandle, HINSTANCE _hInstance)
	{
		parentWindow = _parentHandle;

		hSliderBackground = CreateWindowEx(0, L"Button", L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 50, 500, 400, 50, _parentHandle, (HMENU)0, _hInstance, 0);

		hSliderButton = CreateWindowEx(0, L"Button", L"", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | BS_OWNERDRAW, 50, 500, 150, 50, _parentHandle, (HMENU)0, _hInstance, 0);
	
		//buttonLayoutMap[hButtonSlider].SetInactiveGradient(Gradient{ { 50, 50, 50 }, { 30, 30, 30 } });
		
		SetWindowPos(hSliderButton, HWND_TOP, 0, 0, 0, 0, 0);
		SetWindowPos(hSliderBackground, hSliderButton, 0, 0, 0, 0, 0);

		EnableWindow(hSliderButton, false);
		EnableWindow(hSliderBackground, false);

		sliderGui.Add(hSliderBackground);
		sliderGui.Add(hSliderButton);

		SetLayout(StyleSheet::GetInstance()->GetButtonLayoutParams(InnerDefault), StyleSheet::GetInstance()->GetButtonLayoutParams(InnerDefault));
	}

	void ButtonSlider::SetLayout(ButtonLayoutParams _sliderButtonLayout, ButtonLayoutParams _sliderBackgroundLayout)
	{
		_sliderButtonLayout.inactiveGradient = _sliderButtonLayout.activeGradient;
		sliderButtonLayout = ButtonLayout();
		sliderButtonLayout.SetLayoutParams(_sliderButtonLayout);
		defaultGradient = _sliderButtonLayout.activeGradient;
		pressedGradient = _sliderButtonLayout.pressedGradient;
		sliderBackgroundLayout = ButtonLayout();
		sliderBackgroundLayout.SetLayoutParams(_sliderBackgroundLayout);

	}

	bool ButtonSlider::HandleMouseMove()
	{
		if (sliderDown)
		{
			if (IsMouseInHandle())
			{
				DebugUtility::DbgOut(L"ButtonSlider::HandleMouseMove()::sliderValue::", sliderValue);
				POINT p;
				GetCursorPos(&p);
				RECT borderRect; GetClientRect(hSliderBackground, &borderRect);
				RECT sliderRect; GetClientRect(hSliderButton, &sliderRect);
				MapWindowPoints(hSliderBackground, parentWindow, (POINT *)&borderRect, 2);

				ScreenToClient(parentWindow, &p);
				if (p.x != sliderAnchor.x && p.x > borderRect.left + (sliderRect.right / 2) && p.x < borderRect.right - (sliderRect.right / 2))
				{
					sliderAnchor.x = p.x;
					RECT rect; GetClientRect(parentWindow, &rect);

					sliderPos = p.x - (int)(sliderRect.right / 2.0f) - borderRect.left;
					MoveHandle();

					//32 <--> 256
					float channelWidth = borderRect.right - (sliderRect.right / 2.0f) - borderRect.left - (sliderRect.right / 2.0f);
					float percent = sliderPos / channelWidth;
					sliderValue = lowerBound + (int)(percent * (float)(upperBound-lowerBound)) - 1;
					//voxelsPerMeter = (416 - value);

					return true;
				}
			}
		}
		return false;
	}

	void ButtonSlider::Reposition()
	{
		RECT borderRect; GetClientRect(hSliderBackground, &borderRect);
		RECT sliderRect; GetClientRect(hSliderButton, &sliderRect);
		MapWindowPoints(hSliderBackground, parentWindow, (POINT *)&borderRect, 2);

		//int value = upperBound - _value;
		float percent = (float)(sliderValue - lowerBound) / (float)(upperBound - lowerBound);
		float channelWidth = borderRect.right - (sliderRect.right / 2.0f) - borderRect.left - (sliderRect.right / 2.0f);
		sliderPos = (int)(percent * channelWidth);

		DebugUtility::DbgOut(L"ButtonSlider::SetPositionToValue::percent: ", percent);
		DebugUtility::DbgOut(L"ButtonSlider::SetPositionToValue::sliderValue: ", sliderValue);
		DebugUtility::DbgOut(L"ButtonSlider::SetPositionToValue::sliderPos: ", sliderPos);
	}

	void ButtonSlider::SetValue(int _value)
	{
		if (_value < lowerBound || _value > upperBound)
			return;
		sliderValue = _value;
		repositioningNecessary = true;
	}

	void ButtonSlider::SetStep(int _step)
	{
		if (_step > (upperBound - lowerBound))
			return;
		step = _step;
	}

	void ButtonSlider::SetLimits(int _lowerBound, int _upperBound)
	{
		if (lowerBound < upperBound)
		{
			lowerBound = _lowerBound;
			upperBound = _upperBound;
		}
	}

	int ButtonSlider::GetValue()
	{
		return sliderValue - (sliderValue % step);
	}

	void ButtonSlider::HandleLeftMouseButtonDown()
	{
		DebugUtility::DbgOut(L"ButtonSlider::HandleLeftClickDown()::Before");
		if (!sliderDown)
		{
			if (IsMouseInHandle())
			{
				DebugUtility::DbgOut(L"ButtonSlider::HandleLeftClickDown()");
				sliderDown = true;
				sliderButtonLayout.SetInactiveGradient(pressedGradient);
				Redraw();
			}
		}
	}

	void ButtonSlider::HandleLeftMouseButtonUp()
	{
		if (sliderDown)
		{
			DebugUtility::DbgOut(L"ButtonSlider::HandleLeftClickUp()");
			sliderDown = false;
			sliderButtonLayout.SetInactiveGradient(defaultGradient);
			Redraw();
		}
	}

	bool ButtonSlider::HasHandle(HWND _handle)
	{
		if (_handle == hSliderButton || _handle == hSliderBackground)
			return true;
		return false;
	}

	bool ButtonSlider::Draw(HWND _handle, LPARAM _lParam)
	{
		if (_handle == hSliderButton)
		{
			return sliderButtonLayout.Draw(_lParam);
		}
		else if (_handle == hSliderBackground)
			return sliderBackgroundLayout.Draw(_lParam);
		return false;
	}

	void ButtonSlider::MoveHandle()
	{
		if (repositioningNecessary)
		{
			RECT rect;
			GetClientRect(hSliderButton, &rect);
			if (rect.bottom != 0)
			{
				Reposition();
				repositioningNecessary = false;
			}
		}
		MoveWindow(hSliderButton, x+sliderPos, y, 50, height, true);
	}
	void ButtonSlider::Resize(int _x, int _y, int _width, int _height)
	{
		x = _x;
		y = _y;
		width = _width;
		height = _height;
		MoveHandle();
		MoveWindow(hSliderBackground, _x, _y, _width, _height, true);
	}

	void ButtonSlider::Redraw()
	{
		RECT rect;
		GetClientRect(hSliderButton, &rect);
		InvalidateRect(hSliderButton, &rect, TRUE);
	}

	bool ButtonSlider::IsMouseInHandle()
	{
		POINT pCur;
		GetCursorPos(&pCur);

		RECT rRect; GetWindowRect(hSliderButton, &rRect);
		if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
			pCur.y >= rRect.top && pCur.y <= rRect.bottom)
			return true;

		return false;
	}

	void ButtonSlider::Show()
	{
		sliderGui.Show();
	}

	void ButtonSlider::Hide()
	{
		sliderGui.Hide();
	}

	void ButtonSlider::CleanUp()
	{
		sliderGui.CleanUp();
		sliderButtonLayout.CleanUp();
		sliderBackgroundLayout.CleanUp();
	}

}