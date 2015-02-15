#include "StopWatch.h"

#include <windows.h>


namespace InteractiveFusion {

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;

	StopWatch::StopWatch()
	{

	}

	StopWatch::~StopWatch()
	{

	}

	void StopWatch::Start()
	{
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
	}

	double StopWatch::Stop()
	{
		QueryPerformanceCounter(&t2);
		return (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	}
}