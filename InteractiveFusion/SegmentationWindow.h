#pragma once
#include "SubWindow.h"
#include "ButtonSlider.h"
namespace InteractiveFusion {

	class SegmentationWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			SegmentationFinished = SubWindowEvent::Last,
			UpdateSegmentation,
			Last
		};
	};

	class SegmentationWindow :
		public SubWindow
	{
	public:
		SegmentationWindow();
		virtual ~SegmentationWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow& _parentWindow);

		virtual void Show();
		virtual void Resize(int parentWidth, int parentHeight);
		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);


	private:
		
		enum SliderType { Tolerance, Smoothness, Curvature, Neighbors, RGMinSize, EuclideanMinSize };
		std::unordered_map<SliderType, ButtonSlider> sliderMap;

		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);
		void UpdateSegmentationPreviewValues();
		void UpdateSegmentationUI();
	};
}

