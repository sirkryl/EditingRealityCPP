#pragma once
#include "CommonStructs.h"
#include <Windows.h>
#include <string>
namespace InteractiveFusion {
	class ButtonLayout
	{
	public:
		ButtonLayout();
		ButtonLayout(ButtonLayoutParams _params);

		void SetLayoutParams(ButtonLayoutParams _params);

		bool Draw(LPARAM lParam);

		void SetEdgeRounding(int _rounding);
		void SetFontSize(int _fontSize);
		void SetBackgroundColor(ColorInt _backGroundColor);
		void SetActiveGradient(Gradient _activeGradient);
		void SetInactiveGradient(Gradient _inactiveGradient);
		void SetPressedGradient(Gradient _pressedGradient);
		void SetActivePenColor(ColorInt _activePenColor);
		void SetInactivePenColor(ColorInt _inactivePenColor);
		void SetPressedPenColor(ColorInt _pressedPenColor);
		void SetActiveTextColor(ColorInt _textColor);
		void SetInactiveTextColor(ColorInt _textColor);
		void SetIcon(std::wstring _iconPath);
		void CleanUp();
	private:

		bool paramsInitialized = false;

		ButtonLayoutParams layoutParams;

		HBRUSH CreateGradientBrush(COLORREF top, COLORREF bottom, LPDRAWITEMSTRUCT item);
		HFONT textFont;
		HBRUSH backgroundBrush;
		HBRUSH pressedBrush;
		HBRUSH activeBrush;
		HBRUSH inactiveBrush;

		bool drawIcon = false;
		std::wstring iconPath;
		HICON icon;
		HPEN activePen;
		HPEN inactivePen;
		HPEN pressedPen;
		Gradient activeGradient;
		Gradient inactiveGradient;
		Gradient pressedGradient;

		bool DrawIcon(LPDRAWITEMSTRUCT lParam);

		int iconWidth;
		int iconHeight;

	};

}