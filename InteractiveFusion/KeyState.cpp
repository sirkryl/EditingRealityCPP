#include "KeyState.h"
#include <windows.h>
#include <boost/thread.hpp>
namespace InteractiveFusion {

	void KeyState::SetMouseDown(bool _flag)
	{
		KeyState::leftMouseDown = _flag;
		KeyState::firstDown = _flag;
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

	bool KeyState::LeftMouseFirstDownTouchCheck()
	{
		if (KeyState::leftMouseDown && KeyState::firstDown)
		{
			KeyState::firstDown = false;
			//KeyState::leftMouseDown = false;
			return true;
		}
		else
			return false;
	}

	bool KeyState::LeftMouseDownTouchCheck()
	{
		return KeyState::leftMouseDown;
	}
}