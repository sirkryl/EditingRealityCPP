#pragma once
#include "KinectFusionParams.h"
#include "KinectFusionProcessor.h"
#include <memory>
#include <Windows.h>
namespace InteractiveFusion {
	class FusionDebugDialog {
	public:
		void InitializeDialog(HINSTANCE _hInstance, HWND _parent, shared_ptr<KinectFusionParams> _params, shared_ptr<KinectFusionProcessor> _processor);

		void Show();
		void Hide();
		void MoveOnResize();
		bool IsVisible();
		void CleanUp();
	private:
		HWND debugHandle;
		HINSTANCE hInstance;
		HWND parentHandle;

		std::shared_ptr<KinectFusionParams>        fusionParams;
		std::shared_ptr<KinectFusionProcessor>       fusionProcessor;


		void InitializeUIControls();
		static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		LRESULT CALLBACK FusionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		void ProcessDebugUI(WPARAM wParam, LPARAM lParam);
		void UpdateHSliders();
	};
}