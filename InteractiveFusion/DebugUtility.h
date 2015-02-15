#pragma once
#include <string>

namespace InteractiveFusion {

	namespace DebugUtility {
		void DbgOut(std::wstring label, int value);

		void DbgOut(std::wstring label, float value);

		void DbgOut(std::wstring label, double value);

		void DbgOut(std::wstring label);
	}
}