#pragma once
#include "CommonStructs.h"
#include <string>
namespace InteractiveFusion {

	enum ButtonLayoutType { GlobalDefault, InnerDefault, InactiveMode, ActiveMode, Green, Red, Blue, AlternativeDefault };

	class StyleSheet
	{
	public: 
		static StyleSheet* GetInstance();

		void CreateStyles();

		ColorInt GetGlobalBackgroundColor();
		ColorInt GetInnerBackgroundColor();
		ColorInt GetPropertyBackgroundColor();
		ColorInt GetDefaultTextColor();

		std::wstring GetGlobalFontName();
		int GetGlobalFontWeight();
		ButtonLayoutParams GetButtonLayoutParams(ButtonLayoutType _type);
	private:

		static StyleSheet* instance;
		StyleSheet() {};
		StyleSheet(StyleSheet const&) = delete;
		void operator=(StyleSheet const&) = delete;
	};
}