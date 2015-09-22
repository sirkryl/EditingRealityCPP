#pragma once
#define NOMINMAX
#include <string>
#include <sstream>
#include <Windows.h>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/logger.hpp>
#include "Logger.h"
#include "StringConverter.h"
namespace InteractiveFusion {
	namespace DebugUtility {
		inline void DbgOut(std::wstring label, int value)
		{
			std::wstring output = L"DEBUG::" + label + std::to_wstring(value) + L"\n";
			OutputDebugStringW(output.c_str());
			Logger::WriteToLog(L"DEBUG::" + label + std::to_wstring(value), Logger::debug);
		}

		inline void DbgOut(std::wstring label, float value)
		{
			std::wstring output = L"DEBUG::" + label + std::to_wstring(value) + L"\n";
			OutputDebugStringW(output.c_str());
			Logger::WriteToLog(L"DEBUG::" + label + std::to_wstring(value), Logger::debug);
		}

		inline void DbgOut(std::wstring label, double value)
		{
			std::wstring output = L"DEBUG::" + label + std::to_wstring(value) + L"\n";
			OutputDebugStringW(output.c_str());
			Logger::WriteToLog(L"DEBUG::" + label + std::to_wstring(value), Logger::debug);
		}

		inline void DbgOut(std::wstring label)
		{
			std::wstring output = L"DEBUG::" + label + L"\n";
			OutputDebugStringW(output.c_str());
			Logger::WriteToLog(L"DEBUG::" + label, Logger::debug);
		}
	}
}