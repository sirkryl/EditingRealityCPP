#pragma once
#define NOMINMAX
#include <string>
#include <sstream>
#include <Windows.h>
namespace InteractiveFusion {

	namespace DebugUtility {
		inline void DbgOut(std::wstring label, int value)
		{
			std::wstring output = L"DEBUG::" + label + std::to_wstring(value) + L"\n";
			OutputDebugStringW(output.c_str());
			/*std::wstringstream strs;
			strs << value;
			std::wstring concLabel;
			concLabel.append(L"DEBUG::");
			concLabel.append(label);
			concLabel.append(strs.str());
			concLabel.append(L"\n");
			OutputDebugStringW(concLabel.c_str());*/
		}

		inline void DbgOut(std::wstring label, float value)
		{
			std::wstring output = L"DEBUG::" + label + std::to_wstring(value) + L"\n";
			OutputDebugStringW(output.c_str());
			/*std::wstringstream strs;
			strs << value;
			std::wstring concLabel;
			concLabel.append(L"DEBUG::");
			concLabel.append(label);
			concLabel.append(strs.str());
			concLabel.append(L"\n");
			OutputDebugStringW((concLabel.c_str()));*/
		}

		inline void DbgOut(std::wstring label, double value)
		{
			std::wstring output = L"DEBUG::" + label + std::to_wstring(value) + L"\n";
			OutputDebugStringW(output.c_str());
			//std::wstringstream strs;
			//strs << value;
			//std::wstring concLabel;
			//concLabel.append(L"DEBUG::");
			//concLabel.append(label);
			//concLabel.append(strs.str());
			//concLabel.append(L"\n");
			//OutputDebugStringW((concLabel.c_str()));
		}

		inline void DbgOut(std::wstring label)
		{
			std::wstring output = L"DEBUG::" + label + L"\n";
			OutputDebugStringW(output.c_str());
			//std::wstring conclabel;
			//conclabel.append(L"DEBUG::");
			//conclabel.append(label);
			//conclabel.append(L"\n");
			//OutputDebugStringW(conclabel.c_str());
		}
	}
}