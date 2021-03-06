#include "ColorSelector.h"
#include "DebugUtility.h"
#include "GraphicsController.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	ColorSelector::ColorSelector()
	{
	}


	ColorSelector::~ColorSelector()
	{
	}

	void ColorSelector::HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1)
		{
			_modelData.RemoveTemporaryMeshColor(_selectedIndex);
			_modelData.UnselectMesh();
		}
		_selectedIndex = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl.GetOpenGLWindowHandle());
		_modelData.MarkMeshAsSelected(_selectedIndex);
		_modelData.TemporarilyColorMesh(_selectedIndex);
	}

	void ColorSelector::DrawForColorPicking(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper)
	{
		_modelData.DrawWithAssignedColorCodes(_glControl.GetProjectionMatrix(), _glControl.GetViewMatrix());
	}
}
