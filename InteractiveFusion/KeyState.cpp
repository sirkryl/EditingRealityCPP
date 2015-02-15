#include "KeyState.h"
#include <windows.h>
namespace InteractiveFusion {

	void KeyState::SetMouseDown(bool _flag)
	{
		KeyState::leftMouseDown = _flag;
	}

	int KeyState::GetKeyState(int _key)
	{
		return (GetAsyncKeyState(_key) >> 15) & 1;
	}

	int KeyState::GetKeyStateOnce(int _key)
	{
		if (GetKeyState(_key) && !keysPressed[_key])
		{
			keysPressed[_key] = 1;
			return 1;
		}
		else if (!GetKeyState(_key))
		{
			keysPressed[_key] = 0;
		}
		return 0;
	}

	bool KeyState::LeftMouseDownTouchCheck()
	{
		if (KeyState::leftMouseDown)
		{
			KeyState::leftMouseDown = false;
			return true;
		}
		else
			return false;
	}
}