#include <ctime>
#include <windows.h>
#include <tchar.h>

namespace KFKeys
{
	int GetKeyState(int key);
	int GetKeyStateOnce(int key);
	extern TCHAR kp[256];
}