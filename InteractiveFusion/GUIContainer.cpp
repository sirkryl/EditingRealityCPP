#include "GUIContainer.h"

namespace InteractiveFusion {

	void GUIContainer::Show()
	{
		for (auto& handle : elements)
		{
			ShowWindow(handle, SW_SHOW);
		}
	}

	void GUIContainer::Hide()
	{
		for (auto& handle : elements)
		{
			ShowWindow(handle, SW_HIDE);
		}
	}

	void GUIContainer::Activate()
	{
		for (auto& handle : elements)
		{
			EnableWindow(handle, true);
		}
	}

	void GUIContainer::Deactivate()
	{
		for (auto& handle: elements)
		{
			EnableWindow(handle, false);
		}
	}

	void GUIContainer::Add(HWND _element)
	{
		elements.push_back(_element);
	}

	void GUIContainer::Add(ButtonSlider _element)
	{
		std::vector<HWND> sliderHandles = _element.GetHandles();
		elements.insert(elements.end(), sliderHandles.begin(), sliderHandles.end());
	}

	void GUIContainer::CleanUp()
	{
		elements.clear();
	}

	bool GUIContainer::IsMouseInUI()
	{
		POINT pCur;
		GetCursorPos(&pCur);
		for (auto& handle : elements)
		{
			if (IsWindowVisible(handle) && handle)
			{
				RECT rRect; GetWindowRect(handle, &rRect);
				if (pCur.x >= rRect.left && pCur.x <= rRect.right &&
					pCur.y >= rRect.top && pCur.y <= rRect.bottom)
					return true;
			}
		}
		return false;
	}

	void GUIContainer::Redraw()
	{
		RECT rect;
		for (auto& handle : elements)
		{
			GetClientRect(handle, &rect);
			InvalidateRect(handle, &rect, TRUE);
		}
	}

	bool GUIContainer::ContainsHandle(HWND _handle)
	{
		for (auto& handle : elements)
		{
			if (handle == _handle)
				return true;
		}
		return false;
	}

}