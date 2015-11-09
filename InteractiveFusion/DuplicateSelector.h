#pragma once
#include "ManipulationSelector.h"

namespace InteractiveFusion {
	class DuplicateSelector :
		public ManipulationSelector
	{
	public:
		DuplicateSelector();
		~DuplicateSelector();

	protected:

		virtual void HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
	};
}

