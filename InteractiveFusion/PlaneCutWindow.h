#pragma once
#include "SubWindow.h"
#include "EnumDeclarations.h"
namespace InteractiveFusion {

	class PlaneCutWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			ChangePlaneTransformation = SubWindowEvent::Last,
			ExecutePlaneCut,
			ChangePlaneCutAxis,
			Reset,
			ChangeCameraMovement,
			Last
		};
	};

	class PlaneCutWindow :
		public SubWindow
	{
	public:
		PlaneCutWindow();
		virtual ~PlaneCutWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow& _parentWindow);

		virtual void Resize(int parentWidth, int parentHeight);
		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);


	private:
		PlaneCutAxis planeCutAxis;
		PlaneCutTransformation planeCutTransformation;
		bool cameraMovementEnabled = false;
		void ToggleCameraMovementEnabled();
		void ChangePlaneCutAxis(PlaneCutAxis _axis);
		void ChangePlaneTransformation(PlaneCutTransformation _transformationMode);
		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);
	};
}

