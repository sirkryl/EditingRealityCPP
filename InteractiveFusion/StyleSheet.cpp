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


		styleSheetMap[InactiveMode].fontSize = modeTextSize;
		styleSheetMap[InactiveMode].backgroundColor = globalBackgroundColor;
		styleSheetMap[InactiveMode].activeGradient = inactiveModeGradient;
		styleSheetMap[InactiveMode].inactiveGradient = inactiveGradient;
		styleSheetMap[InactiveMode].pressedGradient = pressedGradient;
		styleSheetMap[InactiveMode].activeTextColor = inactiveModeTextColor;
		styleSheetMap[InactiveMode].inactiveTextColor = inactiveTextColor;
		styleSheetMap[InactiveMode].activePenColor = defaultPenColor;
		styleSheetMap[InactiveMode].inactivePenColor = inactivePenColor;
		styleSheetMap[InactiveMode].pressedPenColor = pressedPenColor;
		styleSheetMap[InactiveMode].edgeRounding = 0;

		//styleSheetMap[InactiveMode] = inactiveModeParams;


		styleSheetMap[ActiveMode].fontSize = modeTextSize;
		styleSheetMap[ActiveMode].backgroundColor = globalBackgroundColor;
		styleSheetMap[ActiveMode].activeGradient = activeModeGradient;
		styleSheetMap[ActiveMode].inactiveGradient = inactiveGradient;
		styleSheetMap[ActiveMode].pressedGradient = pressedGradient;
		styleSheetMap[ActiveMode].activeTextColor = defaultTextColor;
		styleSheetMap[ActiveMode].inactiveTextColor = inactiveTextColor;
		styleSheetMap[ActiveMode].activePenColor = activeModePenColor;
		styleSheetMap[ActiveMode].inactivePenColor = activeModePenColor;
		styleSheetMap[ActiveMode].pressedPenColor = activeModePressedPenColor;
		styleSheetMap[ActiveMode].edgeRounding = 0;

		//styleSheetMap[ActiveMode] = activeModeParams;

		styleSheetMap[GlobalDefault].fontSize = defaultTextSize;
		styleSheetMap[GlobalDefault].backgroundColor = globalBackgroundColor;
		styleSheetMap[GlobalDefault].activeGradient = defaultGradient;
		styleSheetMap[GlobalDefault].inactiveGradient = inactiveGradient;
		styleSheetMap[GlobalDefault].pressedGradient = pressedGradient;
		styleSheetMap[GlobalDefault].activeTextColor = defaultTextColor;
		styleSheetMap[GlobalDefault].inactiveTextColor = inactiveTextColor;
		styleSheetMap[GlobalDefault].activePenColor = defaultPenColor;
		styleSheetMap[GlobalDefault].inactivePenColor = inactivePenColor;
		styleSheetMap[GlobalDefault].pressedPenColor = pressedPenColor;
		styleSheetMap[GlobalDefault].edgeRounding = 0;


		styleSheetMap[AlternativeDefault].fontSize = defaultTextSize;
		styleSheetMap[AlternativeDefault].backgroundColor = globalBackgroundColor;
		styleSheetMap[AlternativeDefault].activeGradient = alternativeGradient;
		styleSheetMap[AlternativeDefault].inactiveGradient = inactiveGradient;
		styleSheetMap[AlternativeDefault].pressedGradient = pressedGradient;
		styleSheetMap[AlternativeDefault].activeTextColor = defaultTextColor;
		styleSheetMap[AlternativeDefault].inactiveTextColor = inactiveTextColor;
		styleSheetMap[AlternativeDefault].activePenColor = defaultPenColor;
		styleSheetMap[AlternativeDefault].inactivePenColor = inactivePenColor;
		styleSheetMap[AlternativeDefault].pressedPenColor = pressedPenColor;
		styleSheetMap[AlternativeDefault].edgeRounding = 0;

		//styleSheetMap[GlobalDefault] = defaultParams;

		styleSheetMap[InnerDefault].fontSize = bigTextSize;
		styleSheetMap[InnerDefault].backgroundColor = innerElementBackgroundColor;
		styleSheetMap[InnerDefault].activeGradient = innerGradient;
		styleSheetMap[InnerDefault].inactiveGradient = innerInactiveGradient;
		styleSheetMap[InnerDefault].pressedGradient = innerPressedGradient;
		styleSheetMap[InnerDefault].activeTextColor = defaultTextColor;
		styleSheetMap[InnerDefault].inactiveTextColor = inactiveTextColor;
		styleSheetMap[InnerDefault].activePenColor = innerPenColor;
		styleSheetMap[InnerDefault].inactivePenColor = innerPenColor;
		styleSheetMap[InnerDefault].pressedPenColor = innerPenColor;
		styleSheetMap[InnerDefault].edgeRounding = 0;

		//styleSheetMap[InnerDefault] = innerDefaultParams;

		styleSheetMap[Green].fontSize = bigTextSize;
		styleSheetMap[Green].backgroundColor = globalBackgroundColor;
		styleSheetMap[Green].activeGradient = greenGradient;
		styleSheetMap[Green].inactiveGradient = inactiveGradient;
		styleSheetMap[Green].pressedGradient = greenPressedGradient;
		styleSheetMap[Green].activeTextColor = defaultTextColor;
		styleSheetMap[Green].inactiveTextColor = inactiveTextColor;
		styleSheetMap[Green].activePenColor = defaultPenColor;
		styleSheetMap[Green].inactivePenColor = inactivePenColor;
		styleSheetMap[Green].pressedPenColor = pressedPenColor;
		styleSheetMap[Green].edgeRounding = 0;

		//styleSheetMap[Green] = greenParams;

		styleSheetMap[Red].fontSize = bigTextSize;
		styleSheetMap[Red].backgroundColor = globalBackgroundColor;
		styleSheetMap[Red].activeGradient = redGradient;
		styleSheetMap[Red].inactiveGradient = inactiveGradient;
		styleSheetMap[Red].pressedGradient = redPressedGradient;
		styleSheetMap[Red].activeTextColor = defaultTextColor;
		styleSheetMap[Red].inactiveTextColor = inactiveTextColor;
		styleSheetMap[Red].activePenColor = defaultPenColor;
		styleSheetMap[Red].inactivePenColor = inactivePenColor;
		styleSheetMap[Red].pressedPenColor = pressedPenColor;
		styleSheetMap[Red].edgeRounding = 0;

		//styleSheetMap[Red] = redParams;

		styleSheetMap[Blue].fontSize = defaultTextSize;
		styleSheetMap[Blue].backgroundColor = globalBackgroundColor;
		styleSheetMap[Blue].activeGradient = blueGradient;
		styleSheetMap[Blue].inactiveGradient = inactiveGradient;
		styleSheetMap[Blue].pressedGradient = bluePressedGradient;
		styleSheetMap[Blue].activeTextColor = defaultTextColor;
		styleSheetMap[Blue].inactiveTextColor = inactiveTextColor;
		styleSheetMap[Blue].activePenColor = defaultPenColor;
		styleSheetMap[Blue].inactivePenColor = inactivePenColor;
		styleSheetMap[Blue].pressedPenColor = pressedPenColor;
		styleSheetMap[Blue].edgeRounding = 0;

		//styleSheetMap[Blue] = blueParams;

		DebugUtility::DbgOut(L"layout duration: ", stopWatch.Stop());
	}
}