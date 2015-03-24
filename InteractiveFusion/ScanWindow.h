#pragma once
#include "SubWindow.h"

namespace InteractiveFusion {

	class ScanWindowEvent : public SubWindowEvent{
	public:
		enum Type {
			Reset = SubWindowEvent::Last,
			Last
		};
	};

	class ScanWindow :
		public SubWindow
	{
	public:
		ScanWindow();
		virtual ~ScanWindow();

		virtual void Initialize(HWND _parentHandle, HINSTANCE _hInstance, float _marginTop, float _marginBottom, float _marginRight, float _marginLeft, std::wstring _className, ColorInt _backgroundColor);

		virtual void HandleEvents(MainWindow& _parentWindow);

		virtual void Resize(int parentWidth, int parentHeight);

		virtual void Show();
		virtual void Hide();
		void PauseScan();
		void UnpauseScan();
		

		glm::mat4 GetCameraMatrix();

		virtual void CleanUp();
		virtual LRESULT CALLBACK SubWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

	protected:
		void FinishFusionScan(MainWindow& _parentWindow);

		virtual void ProcessUI(WPARAM wParam, LPARAM lParam);

	private:
		

		

		bool HasVolumeSizeChanged(int _voxelsPerMeter);


	};
}
