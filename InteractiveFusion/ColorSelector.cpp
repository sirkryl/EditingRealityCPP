#include "ColorSelector.h"
#include "DebugUtility.h"
#include "OpenGLControl.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	ColorSelector::ColorSelector()
	{
	}


	ColorSelector::~ColorSelector()
	{
	}

	void ColorSelector::HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex)
	{
		DebugUtility::DbgOut(L"ColorSelector::HandleLeftMouseClick");
		if (_selectedIndex != -1)
		{
			_modelData->RemoveTemporaryMeshColor(_selectedIndex);
			_modelData->UnselectMesh();
		}
		_selectedIndex = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl->GetOpenGLWindowHandle());
		_modelData->MarkMeshAsSelected(_selectedIndex);
		_modelData->TemporarilyColorMesh(_selectedIndex);
	}

	void ColorSelector::DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper)
	{
		_modelData->DrawWithAssignedColorCodes(_glControl->GetProjectionMatrix(), _glControl->GetViewMatrix());
	}
}
