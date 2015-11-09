#pragma once
#define NOMINMAX

#include "CommonStructs.h"
#include <string>
#include <Windows.h>
#include <vector>
namespace InteractiveFusion {

	class GraphicsController;
	class ModelData;
	class IconData;
	class Selector
	{
	public:
		Selector();
		virtual ~Selector();

		virtual void HandleSelection(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		
		virtual void CleanUp();
	protected:

		virtual void HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleMouseScroll(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		int GetIndexOfMeshUnderCursor(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, HWND _windowHandle);
		virtual void DrawForColorPicking(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		Ray GetRayCastFromCursor(HWND _windowHandle, glm::mat4& _viewMatrix, glm::mat4& _projectionMatrix);

		std::vector<int> GetVertexOrientationFromRayPerspective(Vertex _vertex, Ray _ray);

		int GetColorCodeUnderCursor(HWND _windowHandle);
		
	private:
		bool releaseHandled = true;
	};
}

