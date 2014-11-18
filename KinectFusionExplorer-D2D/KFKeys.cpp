#include "KFKeys.h"

int KFKeys::GetKeyState(int key)
{
	return (GetAsyncKeyState(key) >> 15) & 1;
}

int KFKeys::GetKeyStateOnce(int key)
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
