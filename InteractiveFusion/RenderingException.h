#pragma once
#include "IFException.h"

namespace InteractiveFusion {

	struct RenderingException : public IFException
	{
		RenderingException(const char* pStr = "An exception occurred while rendering.") : IFException(pStr) { }

		
		
	};
}