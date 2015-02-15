#include "DebugUtility.h"
#include <sstream>
#include <Windows.h>
namespace InteractiveFusion {
	namespace DebugUtility {
		void DbgOut(std::wstring label, int value)
		{
			std::wstringstream strs;
			strs << value;
			std::wstring concLabel;
			concLabel.append(L"DEBUG::");
			concLabel.append(label);
			concLabel.append(strs.str());
			concLabel.append(L"\n");
			const TCHAR *c_str = concLabel.c_str();
			OutputDebugString(c_str);
		}

		void DbgOut(std::wstring label, float value)
		{
			std::wstringstream strs;
			strs << value;
			std::wstring concLabel;
			concLabel.append(L"DEBUG::");
			concLabel.append(label);
			concLabel.append(strs.str());
			concLabel.append(L"\n");
			const TCHAR *c_str = concLabel.c_str();
			OutputDebugString((c_str));
		}

		void DbgOut(std::wstring label, double value)
		{
			std::wstringstream strs;
			strs << value;
			std::wstring concLabel;
			concLabel.append(L"DEBUG::");
			concLabel.append(label);
			concLabel.append(strs.str());
			concLabel.append(L"\n");
			const TCHAR *c_str = concLabel.c_str();
			OutputDebugString((c_str));
		}

		void DbgOut(std::wstring label)
		{
			std::wstring concLabel;
			concLabel.append(L"DEBUG::");
			concLabel.append(label);
			concLabel.append(L"\n");
			const TCHAR *c_str = concLabel.c_str();
			OutputDebugString(c_str);
		}
	}
}