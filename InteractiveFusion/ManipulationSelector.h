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

		virtual void HandleLeftMouseClick(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);

		virtual void DrawForColorPicking(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);

	};
}
