#pragma once
#include "SegmentationHelper.h"
#include "SelectionHelper.h"
#include "MeshHelper.h"
#include "GraphicsCamera.h"
namespace InteractiveFusion {
	class InteractionDebugDialog {
	public:
		void InitializeDialog(HINSTANCE _hInstance, HWND _parent, shared_ptr<Parameters> _params, shared_ptr<SegmentationHelper> _segmentationHelper, shared_ptr<SelectionHelper> _selectionHelper, shared_ptr<MeshHelper> _meshHelper, shared_ptr<GraphicsCamera> _openGLCamera);

		void Show();
		void Hide();
		void MoveOnResize();
		bool IsVisible();
		void CleanUp();
	private:
		HWND debugHandle;
		HINSTANCE hInstance;
		HWND parentHandle;
		WNDPROC oldEditProc;
		
		shared_ptr<Parameters>        interactionParams;
		shared_ptr<SegmentationHelper>       segmentationHelper;
		shared_ptr<SelectionHelper> selectionHelper;
		shared_ptr<MeshHelper> meshHelper;
		shared_ptr<GraphicsCamera> openGLCamera;
		void SetBackgroundColor(int redValue, int greenValue, int blueValue);
		void ResetEditControls();
		void ResetSliders();
		void UpdateSliderText();
		void UpdateSliders();

		void InitializeUIControls();
		static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		LRESULT CALLBACK InteractionDebugRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		static LRESULT CALLBACK EditMessageRouter(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam);
		LRESULT CALLBACK SubEditProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam);
		void ProcessDebugUI(WPARAM wParam, LPARAM lParam);
		
	};
}