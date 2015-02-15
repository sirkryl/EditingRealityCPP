#pragma once
#include "Selector.h"

namespace InteractiveFusion {
	class ManipulationSelector :
		public Selector
	{
	public:
		ManipulationSelector();
		~ManipulationSelector();

	protected:

		virtual void HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);

		virtual void DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);

	};
}
