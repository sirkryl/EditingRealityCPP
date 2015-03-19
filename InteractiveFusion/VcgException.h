#pragma once
#include "IFException.h"

namespace InteractiveFusion {

	struct VcgException : public IFException
	{
		VcgException(const char* pStr = "An exception occurred while calling a vcg function.") : IFException(pStr) { }



	};
}