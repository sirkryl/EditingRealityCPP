#include "ManipulationSelector.h"
#include "DebugUtility.h"
#include "IFResources.h"
#include "OpenGLControl.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	ManipulationSelector::ManipulationSelector()
	{
	}


	ManipulationSelector::~ManipulationSelector()
	{
	}

	void ManipulationSelector::HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1)
		{
			_modelData->UnselectMesh();
		}
		_selectedIndex = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl->GetOpenGLWindowHandle());
		_modelData->MarkMeshAsSelected(_selectedIndex);
	}

	void ManipulationSelector::HandleLeftMouseDown(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1)
		{
			int indexOfMeshUnderCursor = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl->GetOpenGLWindowHandle());
			Ray cursorToFarPlaneRay = GetRayCastFromCursor(_glControl->GetOpenGLWindowHandle(), _glControl->GetViewMatrix(), _glControl->GetProjectionMatrix());
			if (indexOfMeshUnderCursor != -1 && indexOfMeshUnderCursor != TRASH_BIN)
			{
				Vertex hitPoint = _modelData->GetHitpoint(indexOfMeshUnderCursor, cursorToFarPlaneRay);
				if (hitPoint.x == NOT_INITIALIZED)
					return;
				std::vector<int> orientation = GetVertexOrientationFromRayPerspective(hitPoint, cursorToFarPlaneRay);
				_modelData->TranslateMeshToPoint(_selectedIndex, glm::vec3(hitPoint.x, hitPoint.y, hitPoint.z), orientation);
				_overlayHelper->SetHovered(TRASH_BIN, false);

			}
			else
			{
				if (indexOfMeshUnderCursor == TRASH_BIN)
					_overlayHelper->SetHovered(TRASH_BIN, true);
				else
					_overlayHelper->SetHovered(TRASH_BIN, false);
				_modelData->TranslateMeshToCursorRay(_selectedIndex, cursorToFarPlaneRay, 5);
				
			}
		}
	}

	void ManipulationSelector::HandleLeftMouseRelease(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex)
	{
		int indexOfMeshUnderCursor = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl->GetOpenGLWindowHandle());

		if (indexOfMeshUnderCursor == TRASH_BIN)
		{
			_modelData->SetMeshAsDeleted(_selectedIndex);
			_overlayHelper->SetHovered(TRASH_BIN, false);
		}
		else if (indexOfMeshUnderCursor != -1)
		{
			_modelData->ResetTemporaryTranslations(indexOfMeshUnderCursor);
		}
		_modelData->UnselectMesh();
	}

	void ManipulationSelector::DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper)
	{
		
		if (_modelData->GetCurrentlySelectedMeshIndex() == -1)
		{
			_modelData->DrawNonStaticMeshWithAssignedColorCodes(_glControl->GetProjectionMatrix(), _glControl->GetViewMatrix());
		}
		else
		{
			_modelData->DrawAllButIndexWithAssignedColorCodes(_modelData->GetCurrentlySelectedMeshIndex(), _glControl->GetProjectionMatrix(), _glControl->GetViewMatrix());
			_overlayHelper->DrawForColorPicking(_glControl->GetViewportWidth(), _glControl->GetViewportHeight());
		}
	}

}
