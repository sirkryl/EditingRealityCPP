#pragma once
#include <string>
#include "EnumDeclarations.h"

namespace InteractiveFusion {

	class Logger
	{
	public:
		enum severity_level
		{
			debug,
			info,
			scenario,
			warning,
			error,
			critical
		};

		Logger();
		~Logger();

		static void Initialize();
		static void WriteToLog(std::wstring label);
		static void WriteToLog(std::wstring label, severity_level level);
		static void CloseLog();
		static void SetState(WindowState state);
	private:
		static WindowState currentState;
	};
}