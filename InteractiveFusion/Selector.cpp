#include "Selector.h"
#include "KeyState.h"
#include "StyleSheet.h"
#include "ColorCoder.h"
#include <glm/gtc/matrix_transform.hpp>
#include "DebugUtility.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	Selector::Selector()
	{
	}


	Selector::~Selector()
	{
	}

	void Selector::HandleSelection(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper)
	{
		int currentlySelectedMeshIndex = _modelData.GetCurrentlySelectedMeshIndex();
		if (KeyState::LeftMouseDownTouchCheck())
		{
			HandleLeftMouseClick(_glControl, _modelData, _overlayHelper, currentlySelectedMeshIndex);
		}
		else if (KeyState::GetKeyState(VK_LBUTTON))
		{
			HandleLeftMouseDown(_glControl, _modelData, _overlayHelper, currentlySelectedMeshIndex);
		}
		else if (currentlySelectedMeshIndex != -1)
		{
			HandleLeftMouseRelease(_glControl, _modelData, _overlayHelper, currentlySelectedMeshIndex);
		}
	}

	void Selector::HandleLeftMouseClick(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
	}

	void Selector::HandleLeftMouseDown(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
	}

	void Selector::HandleLeftMouseRelease(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
	}

	void Selector::DrawForColorPicking(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper)
	{
		_modelData.DrawAllButIndexWithAssignedColorCodes(_modelData.GetCurrentlySelectedMeshIndex(), _glControl.GetProjectionMatrix(), _glControl.GetViewMatrix());
		_overlayHelper.DrawForColorPicking(_glControl.GetViewportWidth(), _glControl.GetViewportHeight());
	}

	int Selector::GetIndexOfMeshUnderCursor(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, HWND _windowHandle)
	{
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClearDepth(1.0);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		DrawForColorPicking(_glControl, _modelData, _overlayHelper);

		int _colorCodeUnderCursor = GetColorCodeUnderCursor(_windowHandle);

		//DebugUtility::DbgOut(L"SelectionHelperNew::GetIndexOfMeshUnderCursor::ColorCode:", _colorCodeUnderCursor);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (_overlayHelper.IsValidKey(_colorCodeUnderCursor))
			return _colorCodeUnderCursor;

		return _modelData.ReturnIndexOfMeshWithColorCode(_colorCodeUnderCursor);
	}

	Ray Selector::GetRayCastFromCursor(HWND _windowHandle, glm::mat4& _viewMatrix, glm::mat4& _projectionMatrix)
	{
		POINT cursorPos;
		GetCursorPos(&cursorPos);
		ScreenToClient(_windowHandle, &cursorPos);
		RECT rect;
		GetClientRect(_windowHandle, &rect);
		cursorPos.y = rect.bottom - cursorPos.y;

		glm::vec4 viewport = glm::vec4(0.0f, 0.0f, rect.right, rect.bottom);

		//DebugUtility::DbgOut(L"SelectionHelperNew::GetRayCastFromCursor::Viewport W: ", rect.right);
		//DebugUtility::DbgOut(L"SelectionHelperNew::GetRayCastFromCursor::Viewport G: ", rect.bottom);

		Ray outputRay;
		outputRay.startPoint = glm::unProject(glm::vec3(float(cursorPos.x), float(cursorPos.y), 0.0f), _viewMatrix, _projectionMatrix, viewport);
		outputRay.endPoint = glm::unProject(glm::vec3(float(cursorPos.x), float(cursorPos.y), 1.0f), _viewMatrix, _projectionMatrix, viewport);
		DebugUtility::DbgOut(L"outputRay start", outputRay.startPoint.x);
		DebugUtility::DbgOut(L"outputRay end", outputRay.endPoint.x);
		return outputRay;
	}

	std::vector<int> Selector::GetVertexOrientationFromRayPerspective(Vertex _vertex, Ray _ray)
	{
		std::vector<int> orientation{ 0, 0, 0 };

		if (abs(_vertex.normal_z) > 0.5f)
		{
			if (abs(_ray.endPoint.z - _ray.startPoint.z) > 700)
			{
				if (_ray.endPoint.z < _ray.startPoint.z)
					orientation[2] = 1;
				else
					orientation[2] = -1;
			}
		}
		if (_vertex.normal_y > 0.5f)
			orientation[1] = 1;
		//else if (normal.y < -0.5f)
		//	orientation[1] = -1;
		//else
		orientation[1] = 1;

		if (abs(_vertex.normal_x) > 0.5f)
		{
			if (abs(_ray.endPoint.x - _ray.startPoint.x) > 900)
			{
				if (_ray.endPoint.x < _ray.startPoint.x)
					orientation[0] = 1;
				else
					orientation[0] = -1;
			}
		}
		return orientation;
	}

	int Selector::GetColorCodeUnderCursor(HWND _windowHandle)
	{
		POINT cursorPos;
		GetCursorPos(&cursorPos);
		ScreenToClient(_windowHandle, &cursorPos);

		RECT rect;
		GetClientRect(_windowHandle, &rect);
		cursorPos.y = rect.bottom - cursorPos.y;

		BYTE colorByte[4];
		glReadPixels(cursorPos.x, cursorPos.y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, colorByte);

		int output = ColorCoder::ColorToInt(colorByte[0], colorByte[1], colorByte[2]);

		ColorIF backgroundColor{ (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().r / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().g / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().b / 255.0f };

		if (colorByte[0] == backgroundColor.r && colorByte[1] == backgroundColor.g && colorByte[2] == backgroundColor.b)
			return -1;
		return output;
	}

	void Selector::CleanUp()
	{

	}
}
