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

		virtual void HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);

		virtual void DrawForColorPicking(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper);

	};
}
