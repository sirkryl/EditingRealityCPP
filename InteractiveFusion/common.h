#pragma once
#include <ctime>
#include <windows.h>
#include <tchar.h>
#include <vector>
#include <sstream>
#include <sys/stat.h>
#include <unordered_map>
#include "openGLResources.h"
#include <boost/smart_ptr/shared_ptr.hpp>
using namespace std;

#include <gl/glew.h>
//#include <GL/glut.h>
#include <glm/glm.hpp>

namespace cDebug {
	static void DbgOut(wstring label, int value) 
	{
		wstringstream strs;
		strs << value;
		wstring concLabel;
		concLabel.append(L"DEBUG::");
		concLabel.append(label);
		concLabel.append(strs.str());
		concLabel.append(L"\n");
		const TCHAR *c_str = concLabel.c_str();
		OutputDebugString(c_str);
	}

	static void DbgOut(wstring label, float value) 
	{
		wstringstream strs;
		strs << value;
		wstring concLabel;
		concLabel.append(L"DEBUG::");
		concLabel.append(label);
		concLabel.append(strs.str());
		concLabel.append(L"\n");
		const TCHAR *c_str = concLabel.c_str();
		OutputDebugString((c_str));
	}

	static void DbgOut(wstring label, double value) 
	{
		wstringstream strs;
		strs << value;
		wstring concLabel;
		concLabel.append(L"DEBUG::");
		concLabel.append(label);
		concLabel.append(strs.str());
		concLabel.append(L"\n");
		const TCHAR *c_str = concLabel.c_str();
		OutputDebugString((c_str));
	}

	static void DbgOut(wstring label)
	{
		wstring concLabel;
		concLabel.append(L"DEBUG::");
		concLabel.append(label);
		concLabel.append(L"\n");
		const TCHAR *c_str = concLabel.c_str();
		OutputDebugString(c_str);
	}
}
	extern wstring statusMsg;