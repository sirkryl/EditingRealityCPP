#include "TransformSelector.h"
#include "KeyState.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {

	bool transformationBasePointInitialized = false;
	glm::vec3 transformationBasePoint;
	int oldPosX;
	int oldPosY;
	bool firstClick = false;

	TransformSelector::TransformSelector()
	{
	}


	TransformSelector::~TransformSelector()
	{
	}

	void TransformSelector::HandleLeftMouseClick(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		ResetTransformationBasePoint();
		ColorSelector::HandleLeftMouseClick(_glControl, _modelData, _overlayHelper, _selectedIndex);
	}

	void TransformSelector::HandleLeftMouseDown(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1)
		{
			if (!transformationBasePointInitialized)
			{
				InitializeTransformationBasePoint(_modelData, _selectedIndex);
			}
			_modelData.TranslateMeshToPoint(_selectedIndex, transformationBasePoint, { 0, 1, 0 });

			HandleRotation(_glControl, _modelData, _selectedIndex);
			HandleScale(_glControl, _modelData, _selectedIndex);
			
		}
	}

	void TransformSelector::HandleLeftMouseRelease(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		ResetTransformationBasePoint();
	}

	bool TransformSelector::InitializeTransformationBasePoint(ModelData& _modelData, int _selectedIndex)
	{
		transformationBasePointInitialized = true;
		if (_modelData.GetBasePoint(_selectedIndex) == nullptr)
			return false;
		transformationBasePoint = *_modelData.GetBasePoint(_selectedIndex);
		return true;
	}

	void TransformSelector::ResetTransformationBasePoint()
	{
		transformationBasePointInitialized = false;
		firstClick = false;
	}

	void TransformSelector::HandleRotation(GraphicsControl& _glControl, ModelData& _modelData, int _selectedIndex)
	{
		POINT pCur;
		GetCursorPos(&pCur);
		float offSetX = (float)((pCur.y - oldPosY)*0.2f);
		float offSetY = (float)((pCur.x - oldPosX)*0.2f);

		if (firstClick)
		{
			glm::mat4 cameraMatrix = _glControl.GetViewMatrix();
			glm::vec3 horizontalRotation = glm::vec3(cameraMatrix[0][0], cameraMatrix[0][1], cameraMatrix[0][2]);
			glm::vec3 verticalRotation = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);

			_modelData.RotateMeshAroundAxis(_selectedIndex, offSetX, horizontalRotation);
			_modelData.RotateMeshAroundAxis(_selectedIndex, offSetY, verticalRotation);
		}

		oldPosX = pCur.x;
		oldPosY = pCur.y;
		firstClick = true;
	}

	void TransformSelector::HandleScale(GraphicsControl& _glControl, ModelData& _modelData, int _selectedIndex)
	{
		if (_glControl.GetMouseWheelDelta() > 0)
		{
			_modelData.ScaleMeshDown(_selectedIndex);
		}
		else if (_glControl.GetMouseWheelDelta() < 0)
		{
			_modelData.ScaleMeshUp(_selectedIndex);
		}
		_glControl.SetMouseWheelDelta(0);
	}

	void TransformSelector::DrawForColorPicking(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper)
	{
		_modelData.DrawNonStaticMeshWithAssignedColorCodes(_glControl.GetProjectionMatrix(), _glControl.GetViewMatrix());
	}



}
