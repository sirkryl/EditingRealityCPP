#pragma once
#include "Selector.h"

namespace InteractiveFusion {
	class ColorSelector :
		public Selector
	{
	public:
		ColorSelector();
		~ColorSelector();

	protected:

		virtual void DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);
		virtual void HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
	};
}
