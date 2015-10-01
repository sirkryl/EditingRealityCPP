#include "MainWindow.h"
#include <Windows.h>

int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{

	InteractiveFusion::MainWindow mW;
	mW.InitApplication(hInstance, lpCmdLine);
}