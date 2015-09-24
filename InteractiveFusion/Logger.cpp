#include "Logger.h"
#include <fstream>
#include <iomanip>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>

namespace InteractiveFusion {

#define NUM_SEVERITY_LEVELS 6

	WindowState Logger::currentState;

	const char* severity_level_str[NUM_SEVERITY_LEVELS] = {
		"debug",
		"info",
		"scenario",
		"warning",
		"error",
		"critical"
	};

	#define NUM_MODES 7

	const char* window_state_str[NUM_MODES] = {
		"Prepare",
		"Scan",
		"PlaneSelection",
		"Segmentation",
		"PlaneCut",
		"Processing",
		"Interaction"
	};

	template< typename CharT, typename TraitsT >
	std::basic_ostream< CharT, TraitsT >&
		operator<< (
		std::basic_ostream< CharT, TraitsT >& strm,
		Logger::severity_level lvl
		)
	{
		const char* str = severity_level_str[lvl];
		if (lvl < NUM_SEVERITY_LEVELS && lvl >= 0)
			strm << str;
		else
			strm << static_cast< int >(lvl);
		return strm;
	}

	BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(my_logger,
		boost::log::sources::wseverity_logger_mt<Logger::severity_level>)
	namespace logging = boost::log;
	namespace src = boost::log::sources;
	namespace sinks = boost::log::sinks;
	namespace keywords = boost::log::keywords;

	Logger::Logger()
	{
	}


	Logger::~Logger()
	{
	}

	void Logger::Initialize()
	{
		boost::log::register_simple_formatter_factory< severity_level, char >("Severity");
		logging::add_file_log
			(
			keywords::file_name = "data/log/sample_%Y-%m-%d_%H-%M-%S.%N.log",                                        /*< file name pattern >*/
			keywords::rotation_size = 10 * 1024 * 1024,                                   /*< rotate files every 10 MiB... >*/
			keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0), /*< ...or at midnight >*/
			keywords::format = "[%TimeStamp%] (%Severity%): %Message%"                                 /*< log record format >*/
			);

		boost::log::add_common_attributes();
	}

	void Logger::SetState(WindowState state)
	{
		Logger::currentState = state;
		WriteToLog(L"Changed state", Logger::info);
	}

	void Logger::WriteToLog(std::wstring label)
	{
		WriteToLog(label, debug);
	}

	void Logger::WriteToLog(std::wstring label, severity_level level)
	{
		boost::log::sources::wseverity_logger_mt<severity_level>& lg = my_logger::get();
		BOOST_LOG_SEV(lg, level) << "[" << window_state_str[(int)currentState] << "] " << label;
	}

	void Logger::CloseLog()
	{
		logging::core::get()->remove_all_sinks();
	}
}