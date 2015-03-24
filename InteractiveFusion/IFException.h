#pragma once
#include <iostream>
#include <exception>
namespace InteractiveFusion {

	struct IFException : public std::exception
	{
		IFException(const char* pStr = "Exception occurred") : pMessage(pStr) { }
		const char * what() const throw ()
		{
			return pMessage;
		}

	private:
		const char* pMessage;
	};
}