#pragma once
#include <glm/glm.hpp>
#include <gl/glew.h>

namespace InteractiveFusion {

#define NOT_INITIALIZED -99999.0f

	struct PlaneParameters
	{
		PlaneParameters() :
			x(NOT_INITIALIZED),
			y(NOT_INITIALIZED),
			z(NOT_INITIALIZED),
			d(NOT_INITIALIZED) {}
		PlaneParameters(float _x, float _y, float _z, float _d) :
			x(_x), y(_y), z(_z), d(_d) {}

		bool IsInitialized() { 
			return x != NOT_INITIALIZED 
				|| y != NOT_INITIALIZED 
				|| z != NOT_INITIALIZED 
				|| d != NOT_INITIALIZED; }
		float x, y, z, d;
	};

	struct ColorIF
	{
		float r, g, b, a;

		bool Equals(const ColorIF& _otherColor)
		{
			if (_otherColor.r == r
				&& _otherColor.g == g
				&& _otherColor.b == b
				&& _otherColor.a == a)
				return true;
			return false;
		}
	};

	struct ColorInt
	{
		int r, g, b;

		bool Equals(const ColorInt& _otherColor)
		{
			if (_otherColor.r == r
				&& _otherColor.g == g
				&& _otherColor.b == b)
				return true;
			return false;
		}
	};

	struct Gradient
	{
		ColorInt topColor, bottomColor;

		bool Equals(const Gradient& _otherGradient)
		{
			if (topColor.Equals(_otherGradient.topColor)
				&& bottomColor.Equals(_otherGradient.bottomColor))
				return true;
			return false;
		}
	};

	struct Vertex
	{
		Vertex() : x(NOT_INITIALIZED), y(NOT_INITIALIZED), z(NOT_INITIALIZED){}
		Vertex(float _x, float _y, float _z, float _r, float _g, float _b,
			float _normal_x, float _normal_y, float _normal_z) :
			x(_x), y(_y), z(_z), r(_r), g(_g), b(_b),
			normal_x{ _normal_x }, normal_y(_normal_y), normal_z(_normal_z) {};
		float x, y, z;
		float r, g, b;
		float normal_x, normal_y, normal_z;

		void Clear() { x = NOT_INITIALIZED; y = NOT_INITIALIZED; z = NOT_INITIALIZED; }

	};

	struct Triangle
	{
		GLuint v1, v2, v3;
	};

	struct Ray
	{
		glm::vec3 startPoint;
		glm::vec3 endPoint;
	};

	struct ButtonLayoutParams
	{
		int fontSize;
		ColorInt backgroundColor;
		Gradient activeGradient, inactiveGradient, pressedGradient;
		ColorInt activeTextColor, inactiveTextColor;
		ColorInt activePenColor, inactivePenColor, pressedPenColor;
		int edgeRounding;
	};
}