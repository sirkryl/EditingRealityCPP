#include "PlaneSelector.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"
#include <glm/gtc/matrix_transform.hpp>
namespace InteractiveFusion {

	Vertex currentCursorToMeshHitpoint;
	float absoluteDegreeX = 0.0f;
	float absoluteDegreeY = 0.0f;
	glm::vec3 horizontalRotationAxis = glm::vec3(0.0f, 0.0f, 1.0f);
	glm::vec3 verticalRotationAxis = glm::vec3(0.0f, 0.0f, 1.0f);
	PlaneSelector::PlaneSelector(std::shared_ptr<SimplePlaneRenderable3D> _plane)
	{
		plane = _plane;
	}

	void PlaneSelector::SetPlane(std::shared_ptr<SimplePlaneRenderable3D> _plane)
	{
		plane = _plane;
	}

	PlaneSelector::~PlaneSelector()
	{
	}

	void PlaneSelector::HandleLeftMouseClick(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		
		int newIndex = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl.GetOpenGLWindowHandle());
		if (newIndex != -1 && newIndex != _selectedIndex)
		{
			_modelData.RemoveTemporaryTriangleColor(_selectedIndex);
			_modelData.RemoveTemporaryMeshColor(_selectedIndex);
			_glControl.PushEvent(GraphicsControlEvent::RemoveModelHighlights);
			_modelData.UnselectMesh();

			_modelData.MarkMeshAsSelected(newIndex);

			_modelData.TemporarilyColorMesh(newIndex);
			_selectedIndex = newIndex;

			ResetPlaneRotation();
		}
		if (newIndex != -1)
		{
			Ray cursorToFarPlaneRay = GetRayCastFromCursor(_glControl.GetOpenGLWindowHandle(), _glControl.GetViewMatrix(), _glControl.GetProjectionMatrix());
			Vertex hitPoint = _modelData.GetHitpoint(_selectedIndex, cursorToFarPlaneRay);
			currentCursorToMeshHitpoint = hitPoint;
			UpdatePlaneTranslation();

			UpdatePreview(_glControl, _modelData, _selectedIndex);
		}
		
	}

	void PlaneSelector::HandleLeftMouseDown(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1)
		{
			HandlePlaneTransformation(_glControl);

			firstClick = true;
		}
	}

	void PlaneSelector::HandleLeftMouseRelease(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1 && firstClick)
		{		
			UpdatePreview(_glControl, _modelData, _selectedIndex);
			//ApplyPlaneRotation();
			
			//_modelData.RemoveTemporaryMeshColor(_selectedIndex);
			//_modelData.UnselectMesh();
			//currentCursorToMeshHitpoint.Clear();
			firstClick = false;
		}
	}

	void PlaneSelector::UpdatePreview(GraphicsControl& _glControl, ModelData& _modelData, int _selectedIndex)
	{
		_modelData.RemoveTemporaryTriangleColor(_selectedIndex);
		_modelData.RemoveTemporaryMeshColor(_selectedIndex);
		_glControl.PlaneCutPreview();
	}

	void PlaneSelector::UpdatePlaneTranslation()
	{
		
		glm::vec3 centerPoint = plane->GetCenterPoint();
		switch (currentAxis)
		{
			case PlaneCutAxis::AxisX:
				plane->SetTranslation(glm::vec3(-centerPoint.x + currentCursorToMeshHitpoint.x, 0.0f, 0.0f));
				break;
			case PlaneCutAxis::AxisY:
				plane->SetTranslation(glm::vec3(0.0f, -centerPoint.y + currentCursorToMeshHitpoint.y, 0.0f));
				break;
			case PlaneCutAxis::AxisZ:
				plane->SetTranslation(glm::vec3(0.0f, 0.0f, -centerPoint.z + currentCursorToMeshHitpoint.z));
				break;
		}
	}

	void PlaneSelector::ApplyPlaneTranslation(float _offSet)
	{
		switch (currentAxis)
		{
		case PlaneCutAxis::AxisX:
				currentCursorToMeshHitpoint.x += _offSet;
				break;
		case PlaneCutAxis::AxisY:
				currentCursorToMeshHitpoint.y -= _offSet;
				break;
		case PlaneCutAxis::AxisZ:
				currentCursorToMeshHitpoint.z += _offSet;
				break;
		}	
		UpdatePlaneTranslation();
	}

	void PlaneSelector::ApplyPlaneRotation(GraphicsControl& _glControl, float _offSetX, float _offSetY)
	{
		
			glm::mat4 cameraMatrix = _glControl.GetViewMatrix();
			switch (currentAxis)
			{
			case PlaneCutAxis::AxisX:
					//horizontalRotationAxis = glm::vec3(0.0f, 1.0f, 0.0f);
					verticalRotationAxis = glm::vec3(cameraMatrix[2][0], cameraMatrix[2][1], cameraMatrix[2][2]);
					horizontalRotationAxis = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);
					//rotationAxis = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);
					break;
			case PlaneCutAxis::AxisY:
					horizontalRotationAxis = glm::vec3(cameraMatrix[2][0], cameraMatrix[2][1], cameraMatrix[2][2]);
					verticalRotationAxis = glm::vec3(cameraMatrix[0][0], cameraMatrix[0][1], cameraMatrix[0][2]);
					//_offSetX = -_offSetX;
					/*glm::vec3 horizontalRotation = glm::vec3(cameraMatrix[0][0], cameraMatrix[0][1], cameraMatrix[0][2]);
					glm::vec3 verticalRotation = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);*/
					break;
			case PlaneCutAxis::AxisZ:
					verticalRotationAxis = glm::vec3(cameraMatrix[0][0], cameraMatrix[0][1], cameraMatrix[0][2]);
					horizontalRotationAxis = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);
					//horizontalRotationAxis = glm::vec3(0.0f, 1.0f, 0.0f);
					//rotationAxis = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);
					break;
			}
			if (CheckRotationLimitsX(_offSetX))
			{
				if (currentAxis == PlaneCutAxis::AxisY)
					_offSetX = -_offSetX;
				plane->AddRotation(_offSetX, horizontalRotationAxis);
			
			if (CheckRotationLimitsY(_offSetY))
			{
				plane->AddRotation(_offSetY, verticalRotationAxis);
			}
				DebugUtility::DbgOut(L"Applying rotation");
				//plane->ApplyTransformation(glm::rotate(glm::mat4(1.0), -_offSet, rotationAxis), glm::mat4());
				
				
			}
			//plane->RotateX(_offSet, rotationAxis);
			//plane->GenerateBuffers();
	}

	bool PlaneSelector::CheckRotationLimitsX(float _offSetX)
	{
		float temporaryDegreeX = absoluteDegreeX + _offSetX;
		
		if (temporaryDegreeX < -30 ||
			temporaryDegreeX > 30)
			return false;
		else
		{
			absoluteDegreeX += _offSetX;
			return true;
		}
	}
	bool PlaneSelector::CheckRotationLimitsY(float _offSetY)
	{
		float temporaryDegreeY = absoluteDegreeY + _offSetY;

		if (temporaryDegreeY < -30 ||
			temporaryDegreeY > 30)
			return false;
		else
		{
			absoluteDegreeY += _offSetY;
			return true;
		}
	}


	void PlaneSelector::HandlePlaneTransformation(GraphicsControl& _glControl)
	{
		POINT pCur;
		GetCursorPos(&pCur);
		float translationOffSet = 0.0f;
		float rotationOffSetX = 0.0f;
		float rotationOffSetY = 0.0f;
		if (currentAxis == PlaneCutAxis::AxisX)
		{
			translationOffSet = (float)((pCur.x - oldPosX)*0.0025f);
			rotationOffSetX = (float)((pCur.x - oldPosX)*0.05f);
			rotationOffSetY = (float)((pCur.y - oldPosY)*0.05f);
		}
		else
		{
			translationOffSet = (float)((pCur.y - oldPosY)*0.0025f);
			rotationOffSetX = (float)((pCur.x - oldPosX)*0.05f);
			rotationOffSetY = (float)((pCur.y - oldPosY)*0.05f);
		}

		if (firstClick)
		{
			
			if (currentTransformation == PlaneCutTransformation::Rotate)
				ApplyPlaneRotation(_glControl, rotationOffSetX, rotationOffSetY);
			else if (currentTransformation == PlaneCutTransformation::Translate)
				ApplyPlaneTranslation(translationOffSet);
		}
		oldPosX = pCur.x;
		oldPosY = pCur.y;
	}

	void PlaneSelector::ApplyModeChange()
	{
		ResetPlaneRotation();
		plane->SetTranslation(glm::vec3(0.0f, 0.0f, 0.0f));
		if (currentAxis == PlaneCutAxis::AxisX)
			plane->ApplyTransformation(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(0.0f, 0.0f, 1.0f)), glm::mat4());
		else if (currentAxis == PlaneCutAxis::AxisZ)
			plane->ApplyTransformation(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(1.0f, 0.0f, 0.0f)), glm::mat4());

		//plane->GenerateBuffers();
	}

	void PlaneSelector::ChangeAxis(PlaneCutAxis _axis)
	{
		currentAxis = _axis;
	}

	void PlaneSelector::ChangeTransformation(PlaneCutTransformation _transformationMode)
	{
		currentTransformation = _transformationMode;
	}
	void PlaneSelector::ResetPlaneRotation()
	{
		absoluteDegreeX = 0.0f;
		absoluteDegreeY = 0.0f;
		
		plane->ResetRotation();
	}

	void PlaneSelector::DrawForColorPicking(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper)
	{
		_modelData.DrawNonStaticMeshWithAssignedColorCodes(_glControl.GetProjectionMatrix(), _glControl.GetViewMatrix());
	}

	void PlaneSelector::CleanUp()
	{
		Selector::CleanUp();
	}
}
