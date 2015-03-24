#include "PlaneSelector.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"
#include <glm/gtc/matrix_transform.hpp>
namespace InteractiveFusion {

	Vertex currentCursorToMeshHitpoint;
	float absoluteDegree = 0.0f;
	float relativeDegree = 0.0f;
	glm::vec3 rotationAxis = glm::vec3(0.0f, 0.0f, 1.0f);
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
			_glControl.PushEvent(RemoveModelHighlights);
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
			DebugUtility::DbgOut(L"ModelData...", _modelData.GetVisibleMeshCount());
			UpdatePlaneTranslation();
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
			//ApplyPlaneRotation();
			_modelData.RemoveTemporaryTriangleColor(_selectedIndex);
			_modelData.RemoveTemporaryMeshColor(_selectedIndex);
			_glControl.PlaneCutPreview();
			//_modelData.RemoveTemporaryMeshColor(_selectedIndex);
			//_modelData.UnselectMesh();
			//currentCursorToMeshHitpoint.Clear();
			firstClick = false;
		}
	}

	void PlaneSelector::UpdatePlaneTranslation()
	{
		
		glm::vec3 centerPoint = plane->GetCenterPoint();
		DebugUtility::DbgOut(L"UpdatePlaneTranslation::x::", currentCursorToMeshHitpoint.x);
		DebugUtility::DbgOut(L"UpdatePlaneTranslation::y::", currentCursorToMeshHitpoint.y);
		DebugUtility::DbgOut(L"UpdatePlaneTranslation::z::", currentCursorToMeshHitpoint.z);
		switch (currentMode)
		{
			case AxisX:
				plane->SetTranslation(glm::vec3(-centerPoint.x + currentCursorToMeshHitpoint.x, 0.0f, 0.0f));
				break;
			case AxisY:
				plane->SetTranslation(glm::vec3(0.0f, -centerPoint.y + currentCursorToMeshHitpoint.y, 0.0f));
				break;
			case AxisZ:
				plane->SetTranslation(glm::vec3(0.0f, 0.0f, -centerPoint.z + currentCursorToMeshHitpoint.z));
				break;
		}
	}

	void PlaneSelector::ApplyPlaneTranslation(float _offSet)
	{
		switch (currentMode)
		{
			case AxisX:
				currentCursorToMeshHitpoint.x += _offSet;
				break;
			case AxisY:
				currentCursorToMeshHitpoint.y -= _offSet;
				break;
			case AxisZ:
				currentCursorToMeshHitpoint.z += _offSet;
				break;
		}	
		UpdatePlaneTranslation();
	}

	void PlaneSelector::ApplyPlaneRotation(GraphicsControl& _glControl, float _offSet)
	{
		if (_offSet > 0 && absoluteDegree > -30 ||
			_offSet < 0 && absoluteDegree < 30)
		{
			glm::mat4 cameraMatrix = _glControl.GetViewMatrix();
			switch (currentMode)
			{
				case AxisX:
					rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f);
					//rotationAxis = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);
					break;
				case AxisY:
					rotationAxis = glm::vec3(cameraMatrix[2][0], cameraMatrix[2][1], cameraMatrix[2][2]);
					break;
				case AxisZ:
					rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f);
					//rotationAxis = glm::vec3(cameraMatrix[1][0], cameraMatrix[1][1], cameraMatrix[1][2]);
					break;
			}
			absoluteDegree -= _offSet;
			relativeDegree -= _offSet;
			DebugUtility::DbgOut(L"Applying rotation");
			plane->ApplyTransformation(glm::rotate(glm::mat4(1.0), -_offSet, rotationAxis), glm::mat4());


			//plane->RotateX(_offSet, rotationAxis);
			//plane->GenerateBuffers();
		}
	}

	void PlaneSelector::HandlePlaneTransformation(GraphicsControl& _glControl)
	{
		POINT pCur;
		GetCursorPos(&pCur);
		float translationOffSet = 0.0f;
		float rotationOffSet = 0.0f;
		if (currentMode == AxisX)
		{
			translationOffSet = (float)((pCur.x - oldPosX)*0.0025f);
			rotationOffSet = (float)((pCur.y - oldPosY)*0.05f);
		}
		else
		{
			translationOffSet = (float)((pCur.y - oldPosY)*0.0025f);
			rotationOffSet = (float)((pCur.x - oldPosX)*0.05f);
		}

		if (firstClick)
		{
			ApplyPlaneTranslation(translationOffSet);
			ApplyPlaneRotation(_glControl, rotationOffSet);
		}
		oldPosX = pCur.x;
		oldPosY = pCur.y;
	}

	void PlaneSelector::ApplyModeChange()
	{
		ResetPlaneRotation();
		if (currentMode == AxisX)
			plane->ApplyTransformation(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(0.0f, 0.0f, 1.0f)), glm::mat4());
		else if (currentMode == AxisZ)
			plane->ApplyTransformation(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(1.0f, 0.0f, 0.0f)), glm::mat4());

		//plane->GenerateBuffers();
	}

	void PlaneSelector::ChangeMode(PlaneCutMode _mode)
	{
		currentMode = _mode;
	}

	void PlaneSelector::ResetPlaneRotation()
	{
		absoluteDegree = 0.0f;
		relativeDegree = 0.0f;
		plane->SetTranslation(glm::vec3(0.0f, 0.0f, 0.0f));
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
