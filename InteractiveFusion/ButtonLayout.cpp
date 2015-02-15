#include "ButtonLayout.h"
#include "DebugUtility.h"
#include "StyleSheet.h"

namespace InteractiveFusion {
	
	ButtonLayout::ButtonLayout()
	{
	}

	ButtonLayout::ButtonLayout(ButtonLayoutParams _params)
	{
		SetLayoutParams(_params);
	}

	void ButtonLayout::SetLayoutParams(ButtonLayoutParams _params)
	{
		if (!_params.backgroundColor.Equals(layoutParams.backgroundColor))
			SetBackgroundColor(_params.backgroundColor);

		if (_params.fontSize != layoutParams.fontSize)
			SetFontSize(_params.fontSize);;

		if (!_params.activeGradient.Equals(layoutParams.activeGradient))
			SetActiveGradient(_params.activeGradient);

		if (!_params.inactiveGradient.Equals(layoutParams.inactiveGradient))
			SetInactiveGradient(_params.inactiveGradient);

		if (!_params.pressedGradient.Equals(layoutParams.pressedGradient))
			SetPressedGradient(_params.pressedGradient);

		if (!_params.activeTextColor.Equals(layoutParams.activeTextColor))
			SetActiveTextColor(_params.activeTextColor);

		if (!_params.inactiveTextColor.Equals(layoutParams.inactiveTextColor))
			SetInactiveTextColor(_params.inactiveTextColor);

		if (!_params.activePenColor.Equals(layoutParams.activePenColor))
			SetActivePenColor(_params.activePenColor);

		if (!_params.inactivePenColor.Equals(layoutParams.inactivePenColor))
			SetInactivePenColor(_params.inactivePenColor);

		if (!_params.pressedPenColor.Equals(layoutParams.pressedPenColor))
			SetPressedPenColor(_params.pressedPenColor);

		if (_params.edgeRounding != layoutParams.edgeRounding)
			SetEdgeRounding(_params.edgeRounding);

		paramsInitialized = true;
		
	}

	void ButtonLayout::SetEdgeRounding(int _rounding)
	{
		layoutParams.edgeRounding = _rounding;
	}

	bool ButtonLayout::DrawIcon(LPDRAWITEMSTRUCT _itemStruct)
	{
		int width = _itemStruct->rcItem.right;
		int height = _itemStruct->rcItem.bottom;

		DrawIconEx(
			_itemStruct->hDC,
			(int) 0.5 * (_itemStruct->rcItem.right - _itemStruct->rcItem.left - width),
			(int) 0.5 * (_itemStruct->rcItem.bottom - _itemStruct->rcItem.top - height),
			(HICON)icon,
			width,
			height,
			0, NULL, DI_NORMAL);
		
		return true;
	}

	bool ButtonLayout::Draw(LPARAM lParam)
	{
		if (!paramsInitialized)
			return false;
		LPDRAWITEMSTRUCT itemStruct = (LPDRAWITEMSTRUCT)lParam;
		SelectObject(itemStruct->hDC, textFont);
		FillRect(itemStruct->hDC, &itemStruct->rcItem, backgroundBrush);
		//SelectObject(itemStruct->hDC, activeBrush);
		SetTextColor(itemStruct->hDC, 
			RGB(layoutParams.activeTextColor.r, layoutParams.activeTextColor.g, layoutParams.activeTextColor.b));

		if (!IsWindowEnabled(itemStruct->hwndItem))
		{
			SetTextColor(itemStruct->hDC, 
				RGB(layoutParams.inactiveTextColor.r, layoutParams.inactiveTextColor.g, layoutParams.inactiveTextColor.b));
			if (inactiveBrush == NULL)
			{
				inactiveBrush = CreateGradientBrush(
					RGB(layoutParams.inactiveGradient.topColor.r, layoutParams.inactiveGradient.topColor.g, layoutParams.inactiveGradient.topColor.b),
					RGB(layoutParams.inactiveGradient.bottomColor.r, layoutParams.inactiveGradient.bottomColor.g, layoutParams.inactiveGradient.bottomColor.b),
					itemStruct);
			}
			SelectObject(itemStruct->hDC, inactiveBrush);
			SelectObject(itemStruct->hDC, inactivePen);
		}
		else if (itemStruct->itemState & ODS_SELECTED)
		{
			if (pressedBrush == NULL)
			{
				pressedBrush = CreateGradientBrush(
					RGB(layoutParams.pressedGradient.topColor.r, layoutParams.pressedGradient.topColor.g, layoutParams.pressedGradient.topColor.b),
					RGB(layoutParams.pressedGradient.bottomColor.r, layoutParams.pressedGradient.bottomColor.g, layoutParams.pressedGradient.bottomColor.b),
					itemStruct);
			}
			SelectObject(itemStruct->hDC, pressedBrush);
			SelectObject(itemStruct->hDC, pressedPen);
		}
		else
		{
			if (activeBrush == NULL)
			{
				activeBrush = CreateGradientBrush(
					RGB(layoutParams.activeGradient.topColor.r, layoutParams.activeGradient.topColor.g, layoutParams.activeGradient.topColor.b),
					RGB(layoutParams.activeGradient.bottomColor.r, layoutParams.activeGradient.bottomColor.g, layoutParams.activeGradient.bottomColor.b),
					itemStruct);
			}
			SelectObject(itemStruct->hDC, activeBrush);
			SelectObject(itemStruct->hDC, activePen);
		}

		SetBkMode(itemStruct->hDC, TRANSPARENT);

		RoundRect(itemStruct->hDC, itemStruct->rcItem.left, itemStruct->rcItem.top, itemStruct->rcItem.right, itemStruct->rcItem.bottom, layoutParams.edgeRounding, layoutParams.edgeRounding);
		
		
		if (drawIcon)
			return DrawIcon(itemStruct);

		int len;
		len = GetWindowTextLength(itemStruct->hwndItem);
		LPSTR lpBuff = new char[len + 1];
		GetWindowTextA(itemStruct->hwndItem, lpBuff, len + 1);

		DrawTextA(itemStruct->hDC, lpBuff, len, &itemStruct->rcItem, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
		DeleteObject(lpBuff);

		return TRUE;
	}


	HBRUSH ButtonLayout::CreateGradientBrush(COLORREF top, COLORREF bottom, LPDRAWITEMSTRUCT item)
	{
		HBRUSH Brush = NULL;
		HDC hdcmem = CreateCompatibleDC(item->hDC);
		HBITMAP hbitmap = CreateCompatibleBitmap(item->hDC, item->rcItem.right - item->rcItem.left, item->rcItem.bottom - item->rcItem.top);
		SelectObject(hdcmem, hbitmap);

		int r1 = GetRValue(top), r2 = GetRValue(bottom), g1 = GetGValue(top), g2 = GetGValue(bottom), b1 = GetBValue(top), b2 = GetBValue(bottom);
		for (int i = 0; i < item->rcItem.bottom - item->rcItem.top; i++)
		{
			RECT temp;
			int r, g, b;
			r = int(r1 + double(i * (r2 - r1) / item->rcItem.bottom - item->rcItem.top));
			g = int(g1 + double(i * (g2 - g1) / item->rcItem.bottom - item->rcItem.top));
			b = int(b1 + double(i * (b2 - b1) / item->rcItem.bottom - item->rcItem.top));
			Brush = CreateSolidBrush(RGB(r, g, b));
			temp.left = 0;
			temp.top = i;
			temp.right = item->rcItem.right - item->rcItem.left;
			temp.bottom = i + 1;
			FillRect(hdcmem, &temp, Brush);
			DeleteObject(Brush);
		}
		HBRUSH pattern = CreatePatternBrush(hbitmap);

		DeleteDC(hdcmem);
		DeleteObject(Brush);
		DeleteObject(hbitmap);

		return pattern;
	}

	void ButtonLayout::SetFontSize(int _fontSize)
	{
		if (textFont != NULL)
		{
			DeleteObject(textFont);
			textFont = NULL;
		}
		layoutParams.fontSize = _fontSize;
		textFont = CreateFont(_fontSize, 0, 0, 0, StyleSheet::GetInstance()->GetGlobalFontWeight(), 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH, StyleSheet::GetInstance()->GetGlobalFontName().c_str());
	}

	void ButtonLayout::SetBackgroundColor(ColorInt _backGroundColor)
	{
		if (backgroundBrush != NULL)
			DeleteObject(backgroundBrush);
		layoutParams.backgroundColor = _backGroundColor;
		backgroundBrush = CreateSolidBrush(RGB(_backGroundColor.r, _backGroundColor.g, _backGroundColor.b));
	}

	void ButtonLayout::SetActiveGradient(Gradient _activeGradient)
	{
		if (activeBrush != NULL)
		{
			DeleteObject(activeBrush);
			activeBrush = NULL;
		}
		layoutParams.activeGradient = _activeGradient;
	}

	void ButtonLayout::SetInactiveGradient(Gradient _inactiveGradient)
	{
		if (inactiveBrush != NULL)
		{
			DeleteObject(inactiveBrush);
			inactiveBrush = NULL;
		}
		layoutParams.inactiveGradient = _inactiveGradient;
	}

	void ButtonLayout::SetPressedGradient(Gradient _pressedGradient)
	{
		if (pressedBrush != NULL)
		{
			DeleteObject(pressedBrush);
			pressedBrush = NULL;
		}
		layoutParams.pressedGradient = _pressedGradient;
	}

	void ButtonLayout::SetActivePenColor(ColorInt _activePenColor)
	{
		if (activePen != NULL)
			DeleteObject(activePen);
		layoutParams.activePenColor = _activePenColor;
		activePen = CreatePen(PS_INSIDEFRAME, 2, RGB(_activePenColor.r, _activePenColor.g, _activePenColor.b));
	}

	void ButtonLayout::SetInactivePenColor(ColorInt _inactivePenColor)
	{
		if (inactivePen != NULL)
			DeleteObject(inactivePen);
		layoutParams.inactivePenColor = _inactivePenColor;
		inactivePen = CreatePen(PS_INSIDEFRAME, 2, RGB(_inactivePenColor.r, _inactivePenColor.g, _inactivePenColor.b));
	}

	void ButtonLayout::SetPressedPenColor(ColorInt _pressedPenColor)
	{
		if (pressedPen != NULL)
			DeleteObject(pressedPen);
		layoutParams.pressedPenColor = _pressedPenColor;
		pressedPen = CreatePen(PS_INSIDEFRAME, 2, RGB(_pressedPenColor.r, _pressedPenColor.g, _pressedPenColor.b));
		
	}

	void ButtonLayout::SetActiveTextColor(ColorInt _textColor)
	{
		layoutParams.activeTextColor = _textColor;
	}

	void ButtonLayout::SetInactiveTextColor(ColorInt _textColor)
	{
		layoutParams.inactiveTextColor = _textColor;
	}

	void ButtonLayout::SetIcon(std::wstring _iconPath)
	{
		drawIcon = true;
		icon = (HICON)LoadImage(NULL, _iconPath.c_str(), IMAGE_ICON, 256, 256, LR_LOADFROMFILE);
	}

	void ButtonLayout::CleanUp()
	{
		DeleteObject(textFont);
		DeleteObject(backgroundBrush);
		DeleteObject(activeBrush);
		DeleteObject(inactiveBrush);
		DeleteObject(activePen);
		DeleteObject(inactivePen);
		DeleteObject(pressedPen);
		DeleteObject(pressedBrush);
		DeleteObject(icon);
		paramsInitialized = false;
	}
}