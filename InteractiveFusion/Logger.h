#pragma once
#include <string>


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
	};
}