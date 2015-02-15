#pragma once
#define NOMINMAX

#include "CommonStructs.h"
#include <string>
#include <Windows.h>
#include <vector>
namespace InteractiveFusion {

	class OpenGLControl;
	class ModelData;
	class IconData;
	class Selector
	{
	public:
		Selector();
		virtual ~Selector();

		virtual void HandleSelection(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);
		
		virtual void CleanUp();
	protected:

		virtual void HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);

		int GetIndexOfMeshUnderCursor(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, HWND _windowHandle);
		virtual void DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);
		Ray GetRayCastFromCursor(HWND _windowHandle, glm::mat4* _viewMatrix, glm::mat4* _projectionMatrix);

		std::vector<int> GetVertexOrientationFromRayPerspective(Vertex _vertex, Ray _ray);

		int GetColorCodeUnderCursor(HWND _windowHandle);
		
	};
}

