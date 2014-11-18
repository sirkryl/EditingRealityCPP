#include "Keys.h"

int Keys::GetKeyState(int key)
{
	return (GetAsyncKeyState(key) >> 15) & 1;
}

int Keys::GetKeyStateOnce(int key)
{
	if (GetKeyState(key) && !kp[key])
	{
		kp[key] = 1;
		return 1;
	}
	else if (!GetKeyState(key))
	{
		kp[key] = 0;
	}
	return 0;
}
