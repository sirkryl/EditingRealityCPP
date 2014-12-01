#pragma once
#include <ctime>
#include <windows.h>
#include <tchar.h>
#include <vector>
#include <Shlobj.h>
#include <sstream>
#include <sys/stat.h>
#include <unordered_map>
#include "IFResources.h"
#include <boost/shared_ptr.hpp>
using namespace std;

#include <gl/glew.h>
//#include <GL/glut.h>
#include <glm/glm.hpp>

struct ColorIF
{
	float r, g, b;
};

struct Vertex
{
	float x, y, z;
	float r, g, b;
	float normal_x, normal_y, normal_z;

};

struct Triangle
{
	GLuint v1, v2, v3;
};

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

namespace Util
{
	static std::wstring StringToWString(const std::string& s)
	{
		int len;
		int slength = (int)s.length() + 1;
		len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
		wchar_t* buf = new wchar_t[len];
		MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
		std::wstring r(buf);
		delete[] buf;
		return r;
	}
}
extern wstring statusMsg;