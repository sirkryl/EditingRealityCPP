#pragma once
#include "ColorSelector.h"

namespace InteractiveFusion {
	class TransformSelector :
		public ColorSelector
	{
	public:
		TransformSelector();
		~TransformSelector();

	protected:

		virtual void HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleMouseScroll(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		bool InitializeTransformationBasePoint(ModelData& _modelData, int _selectedIndex);

		void ResetTransformationBasePoint();

		void HandleRotation(GraphicsController& _glControl, ModelData& _modelData, int _selectedIndex);

		void TransformSelector::HandleScale(GraphicsController& _glControl, ModelData& _modelData, int _selectedIndex);

		virtual void DrawForColorPicking(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper);

	};
}

