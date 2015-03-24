#pragma once
#include "SubWindow.h"
#include "EnumDeclarations.h"
namespace InteractiveFusion {

	class InteractionWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			ChangeCameraMode = SubWindowEvent::Last,
			ChangeManipulationMode,
			Reset,
			ExportModel,
			Last
		};
	};

	class InteractionWindow :
		public SubWindow
	{
	public:
		InteractionWindow();
		virtual ~InteractionWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow& _parentWindow);

		virtual void Resize(int parentWidth, int parentHeight);
		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);


	private:

		InteractionMode interactionMode = None;
		OpenGLCameraMode cameraMode = Sensor;
		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);

		void ChangeInteractionMode(InteractionMode _interactionMode);
	};
}

