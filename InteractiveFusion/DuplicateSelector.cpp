#include "DuplicateSelector.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	DuplicateSelector::DuplicateSelector()
	{
	}


	DuplicateSelector::~DuplicateSelector()
	{
	}

	void DuplicateSelector::HandleLeftMouseClick(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex)
	{
		if (_selectedIndex != -1)
		{
			_modelData.UnselectMesh();
		}
		_selectedIndex = GetIndexOfMeshUnderCursor(_glControl, _modelData, _overlayHelper, _glControl.GetOpenGLWindowHandle());

		_modelData.MarkMeshAsSelected(_modelData.DuplicateMeshAndGetItsIndex(_selectedIndex));
		_glControl.PushEvent(ModelDataUpdated);
	}

}
