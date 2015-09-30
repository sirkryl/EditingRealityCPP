#pragma once
#define NOMINMAX

#include "CommonStructs.h"
#include <string>
#include <Windows.h>
#include <vector>
namespace InteractiveFusion {

	class GraphicsControl;
	class ModelData;
	class IconData;
	class Selector
	{
	public:
		Selector();
		virtual ~Selector();

		virtual void HandleSelection(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		
		virtual void CleanUp();
	protected:

		virtual void HandleLeftMouseClick(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleMouseScroll(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		int GetIndexOfMeshUnderCursor(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, HWND _windowHandle);
		virtual void DrawForColorPicking(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		Ray GetRayCastFromCursor(HWND _windowHandle, glm::mat4& _viewMatrix, glm::mat4& _projectionMatrix);

		std::vector<int> GetVertexOrientationFromRayPerspective(Vertex _vertex, Ray _ray);

		int GetColorCodeUnderCursor(HWND _windowHandle);
		
	private:
		bool releaseHandled = true;
	};
}

