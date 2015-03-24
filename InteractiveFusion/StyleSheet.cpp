#include "StyleSheet.h"
#include "ButtonLayout.h"
#include "DebugUtility.h"
#include "StopWatch.h"
#include <map>
namespace InteractiveFusion {


	StyleSheet* StyleSheet::instance;
	ColorInt globalBackgroundColor{ 66, 66, 66 };
	ColorInt innerElementBackgroundColor{ 33, 33, 33 };
	ColorInt propertyBackgroundColor{ 66, 66, 66 };// { 79, 79, 79 };
	ColorInt defaultTextColor{ 202, 225, 225 };

	std::wstring globalFontName = L"Open Sans";
	int globalFontWeight = FW_REGULAR;
	std::map<ButtonLayoutType, ButtonLayoutParams> styleSheetMap;

	StyleSheet* StyleSheet::GetInstance()
	{
		if (!instance) instance = new StyleSheet;
		assert(instance != NULL);
		return instance;
	}


	ColorInt StyleSheet::GetGlobalBackgroundColor()
	{
		return globalBackgroundColor;
	}

	ColorInt StyleSheet::GetInnerBackgroundColor()
	{
		return innerElementBackgroundColor;
	}

	ColorInt StyleSheet::GetPropertyBackgroundColor()
	{
		return propertyBackgroundColor;
	}

	ColorInt StyleSheet::GetDefaultTextColor()
	{
		return defaultTextColor;
	}

	std::wstring StyleSheet::GetGlobalFontName()
	{
		return globalFontName;
	}
	int StyleSheet::GetGlobalFontWeight()
	{
		return globalFontWeight;
	}
	ButtonLayoutParams StyleSheet::GetButtonLayoutParams(ButtonLayoutType _type)
	{
		return styleSheetMap[_type];
	}

	void StyleSheet::CreateStyles()
	{
		StopWatch stopWatch;
		stopWatch.Start();

		//default button

		int defaultTextSize = 20;

		//other default possibility
		Gradient alternativeGradient{
			{ 40, 40, 40 },
			{ 20, 20, 20 } };
		// good alternative::
		Gradient defaultGradient{
		{ 113, 113, 113 },
		{ 79, 79, 79 } };


		Gradient inactiveGradient{
			{ 40, 40, 40 },
			{ 40, 40, 40 } };

		Gradient pressedGradient{
			{ 65, 65, 65 },
			{ 60, 60, 60 } };


		ColorInt inactiveTextColor{ 50, 50, 50 };

		ColorInt defaultPenColor{ 40, 40, 40 };
		ColorInt pressedPenColor{ 40, 40, 40 };
		ColorInt inactivePenColor{ 40, 40, 40 };

		//inner button
		Gradient innerGradient{
			{ 30, 30, 30 },
			{ 20, 20, 20 } };

		Gradient innerPressedGradient{
			{ 50, 50, 50 },
			{ 40, 40, 40 } };

		Gradient innerInactiveGradient{
			{ 15, 15, 15 },
			{ 5, 5, 5 } };

		ColorInt innerPenColor = globalBackgroundColor;

		//active mode button
		int modeTextSize = 22;

		Gradient activeModeGradient{
			globalBackgroundColor,
			globalBackgroundColor };

		ColorInt activeModePenColor = globalBackgroundColor;
		ColorInt activeModePressedPenColor = globalBackgroundColor;

		//inactive mode button
		Gradient inactiveModeGradient{
			{ 60, 60, 60 },
			{ 55, 55, 55 } };

		ColorInt inactiveModeTextColor{ 149, 149, 149 };

		//red button
		int bigTextSize = 40;

		Gradient redGradient{
			{ 110, 0, 0 },
			{ 80, 0, 0 } };

		Gradient redPressedGradient{
			{ 130, 0, 0 },
			{ 110, 0, 0 } };

		//green button
		Gradient greenGradient{
			{ 0, 110, 0 },
			{ 0, 80, 0 } };

		Gradient greenPressedGradient{
			{ 0, 130, 0 },
			{ 0, 110, 0 } };

		//blue button
		Gradient blueGradient{
			{ 0, 0, 110 },
			{ 0, 0, 80 } };

		Gradient bluePressedGradient{
			{ 0, 0, 130 },
			{ 0, 0, 110 } };


		styleSheetMap[ButtonLayoutType::InactiveMode].fontSize = modeTextSize;
		styleSheetMap[ButtonLayoutType::InactiveMode].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::InactiveMode].activeGradient = inactiveModeGradient;
		styleSheetMap[ButtonLayoutType::InactiveMode].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::InactiveMode].pressedGradient = pressedGradient;
		styleSheetMap[ButtonLayoutType::InactiveMode].activeTextColor = inactiveModeTextColor;
		styleSheetMap[ButtonLayoutType::InactiveMode].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::InactiveMode].activePenColor = defaultPenColor;
		styleSheetMap[ButtonLayoutType::InactiveMode].inactivePenColor = inactivePenColor;
		styleSheetMap[ButtonLayoutType::InactiveMode].pressedPenColor = pressedPenColor;
		styleSheetMap[ButtonLayoutType::InactiveMode].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::InactiveMode] = inactiveModeParams;


		styleSheetMap[ButtonLayoutType::ActiveMode].fontSize = modeTextSize;
		styleSheetMap[ButtonLayoutType::ActiveMode].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::ActiveMode].activeGradient = activeModeGradient;
		styleSheetMap[ButtonLayoutType::ActiveMode].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::ActiveMode].pressedGradient = pressedGradient;
		styleSheetMap[ButtonLayoutType::ActiveMode].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::ActiveMode].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::ActiveMode].activePenColor = activeModePenColor;
		styleSheetMap[ButtonLayoutType::ActiveMode].inactivePenColor = activeModePenColor;
		styleSheetMap[ButtonLayoutType::ActiveMode].pressedPenColor = activeModePressedPenColor;
		styleSheetMap[ButtonLayoutType::ActiveMode].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::ActiveMode] = activeModeParams;

		styleSheetMap[ButtonLayoutType::GlobalDefault].fontSize = defaultTextSize;
		styleSheetMap[ButtonLayoutType::GlobalDefault].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::GlobalDefault].activeGradient = defaultGradient;
		styleSheetMap[ButtonLayoutType::GlobalDefault].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::GlobalDefault].pressedGradient = pressedGradient;
		styleSheetMap[ButtonLayoutType::GlobalDefault].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::GlobalDefault].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::GlobalDefault].activePenColor = defaultPenColor;
		styleSheetMap[ButtonLayoutType::GlobalDefault].inactivePenColor = inactivePenColor;
		styleSheetMap[ButtonLayoutType::GlobalDefault].pressedPenColor = pressedPenColor;
		styleSheetMap[ButtonLayoutType::GlobalDefault].edgeRounding = 0;


		styleSheetMap[ButtonLayoutType::AlternativeDefault].fontSize = defaultTextSize;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].activeGradient = alternativeGradient;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].pressedGradient = pressedGradient;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].activePenColor = defaultPenColor;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].inactivePenColor = inactivePenColor;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].pressedPenColor = pressedPenColor;
		styleSheetMap[ButtonLayoutType::AlternativeDefault].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::GlobalDefault] = defaultParams;

		styleSheetMap[ButtonLayoutType::InnerDefault].fontSize = bigTextSize;
		styleSheetMap[ButtonLayoutType::InnerDefault].backgroundColor = innerElementBackgroundColor;
		styleSheetMap[ButtonLayoutType::InnerDefault].activeGradient = innerGradient;
		styleSheetMap[ButtonLayoutType::InnerDefault].inactiveGradient = innerInactiveGradient;
		styleSheetMap[ButtonLayoutType::InnerDefault].pressedGradient = innerPressedGradient;
		styleSheetMap[ButtonLayoutType::InnerDefault].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::InnerDefault].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::InnerDefault].activePenColor = innerPenColor;
		styleSheetMap[ButtonLayoutType::InnerDefault].inactivePenColor = innerPenColor;
		styleSheetMap[ButtonLayoutType::InnerDefault].pressedPenColor = innerPenColor;
		styleSheetMap[ButtonLayoutType::InnerDefault].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::InnerDefault] = innerDefaultParams;

		styleSheetMap[ButtonLayoutType::Green].fontSize = bigTextSize;
		styleSheetMap[ButtonLayoutType::Green].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::Green].activeGradient = greenGradient;
		styleSheetMap[ButtonLayoutType::Green].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::Green].pressedGradient = greenPressedGradient;
		styleSheetMap[ButtonLayoutType::Green].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::Green].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::Green].activePenColor = defaultPenColor;
		styleSheetMap[ButtonLayoutType::Green].inactivePenColor = inactivePenColor;
		styleSheetMap[ButtonLayoutType::Green].pressedPenColor = pressedPenColor;
		styleSheetMap[ButtonLayoutType::Green].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::Green] = greenParams;

		styleSheetMap[ButtonLayoutType::Red].fontSize = bigTextSize;
		styleSheetMap[ButtonLayoutType::Red].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::Red].activeGradient = redGradient;
		styleSheetMap[ButtonLayoutType::Red].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::Red].pressedGradient = redPressedGradient;
		styleSheetMap[ButtonLayoutType::Red].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::Red].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::Red].activePenColor = defaultPenColor;
		styleSheetMap[ButtonLayoutType::Red].inactivePenColor = inactivePenColor;
		styleSheetMap[ButtonLayoutType::Red].pressedPenColor = pressedPenColor;
		styleSheetMap[ButtonLayoutType::Red].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::Red] = redParams;

		styleSheetMap[ButtonLayoutType::Blue].fontSize = defaultTextSize;
		styleSheetMap[ButtonLayoutType::Blue].backgroundColor = globalBackgroundColor;
		styleSheetMap[ButtonLayoutType::Blue].activeGradient = blueGradient;
		styleSheetMap[ButtonLayoutType::Blue].inactiveGradient = inactiveGradient;
		styleSheetMap[ButtonLayoutType::Blue].pressedGradient = bluePressedGradient;
		styleSheetMap[ButtonLayoutType::Blue].activeTextColor = defaultTextColor;
		styleSheetMap[ButtonLayoutType::Blue].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ButtonLayoutType::Blue].activePenColor = defaultPenColor;
		styleSheetMap[ButtonLayoutType::Blue].inactivePenColor = inactivePenColor;
		styleSheetMap[ButtonLayoutType::Blue].pressedPenColor = pressedPenColor;
		styleSheetMap[ButtonLayoutType::Blue].edgeRounding = 0;

		//styleSheetMap[ButtonLayoutType::Blue] = blueParams;

		DebugUtility::DbgOut(L"layout duration: ", stopWatch.Stop());
	}
}