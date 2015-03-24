//#include "DebugUtility.h"
//#include <sstream>
//#include <Windows.h>
//#include <boost/thread/mutex.hpp>
//namespace InteractiveFusion {
//
//	#ifdef _DEBUG
//	#define LOGMESSAGE( str ) OutputDebugString( str );
//	#else
//	#define LOGMESSAGE( str )
//	#endif
//
//	
//
//	namespace DebugUtility {
//		boost::mutex mtx_;
//		inline void DbgOut(std::wstring label, int value)
//		{
//			/*mtx_.lock();
//			OutputDebugStringW(L"DbgOut Start\n");
//			std::wstringstream strs;
//			strs << value;
//			std::wstring concLabel;
//			concLabel.append(L"DEBUG::");
//			concLabel.append(label);
//			concLabel.append(strs.str());
//			concLabel.append(L"\n");
//			OutputDebugStringW(concLabel.c_str());
//			OutputDebugStringW(L"DbgOut End\n");
//			mtx_.unlock();*/
//		}
//
//		inline void DbgOut(std::wstring label, float value)
//		{
//			/*mtx_.lock();
//			OutputDebugStringW(L"DbgOut Start\n");
//			std::wstringstream strs;
//			strs << value;
//			std::wstring concLabel;
//			concLabel.append(L"DEBUG::");
//			concLabel.append(label);
//			concLabel.append(strs.str());
//			concLabel.append(L"\n");
//			OutputDebugStringW((concLabel.c_str()));
//			OutputDebugStringW(L"DbgOut End\n");
//			mtx_.unlock();*/
//		}
//
//		inline void DbgOut(std::wstring label, double value)
//		{
//			/*mtx_.lock();
//			OutputDebugStringW(L"DbgOut Start\n");
//			std::wstringstream strs;
//			strs << value;
//			std::wstring concLabel;
//			concLabel.append(L"DEBUG::");
//			concLabel.append(label);
//			concLabel.append(strs.str());
//			concLabel.append(L"\n");
//			OutputDebugStringW((concLabel.c_str()));
//			OutputDebugStringW(L"DbgOut End\n");
//			mtx_.unlock();*/
//		}
//
//		inline void DbgOut(std::wstring label)
//		{
//			/*mtx_.lock();
//			OutputDebugStringW(L"DbgOut Start\n");
//			std::wstring conclabel;
//			conclabel.append(L"DEBUG::");
//			conclabel.append(label);
//			conclabel.append(L"\n");
//			OutputDebugStringW(conclabel.c_str());
//			OutputDebugStringW(L"DbgOut End\n");
//			mtx_.unlock();*/
//		}
//	}
//}