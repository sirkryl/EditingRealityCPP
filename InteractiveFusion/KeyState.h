#pragma once
#include <tchar.h>

namespace InteractiveFusion {
	namespace KeyState
	{
		void SetMouseDown(bool _flag);
		int GetKeyState(int _key);
		int GetKeyStateOnce(int _key);
		bool LeftMouseDownTouchCheck();
		extern TCHAR keysPressed[256];
		extern bool leftMouseDown;
	}
}