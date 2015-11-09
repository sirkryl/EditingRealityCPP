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

		virtual void DrawForColorPicking(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		virtual void HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
	};
}
