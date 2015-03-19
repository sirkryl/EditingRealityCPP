#pragma once
#include "IFException.h"

namespace InteractiveFusion {

	struct BufferException : public IFException
	{
		BufferException(const char* pStr = "An exception occurred while dealing with buffers.") : IFException(pStr) { }
	};
}