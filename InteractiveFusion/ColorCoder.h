#pragma once
#include <glm/glm.hpp>
namespace InteractiveFusion
{
	namespace ColorCoder
	{
		static int ColorToInt(int r, int g, int b)
		{
			return (r) | (g << 8) | (b << 16);
		}

		static glm::vec4 IntToColor(int index)
		{
			int r = index & 0xFF;
			int g = (index >> 8) & 0xFF;
			int b = (index >> 16) & 0xFF;

			return glm::vec4(float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f, 1.0f);
		}

		
	}
}