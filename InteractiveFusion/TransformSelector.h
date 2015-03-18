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

		virtual void HandleLeftMouseClick(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);

		bool InitializeTransformationBasePoint(ModelData* _modelData, int _selectedIndex);

		void ResetTransformationBasePoint();

		void HandleRotation(GraphicsControl* _glControl, ModelData* _modelData, int _selectedIndex);

		void TransformSelector::HandleScale(GraphicsControl* _glControl, ModelData* _modelData, int _selectedIndex);

		virtual void DrawForColorPicking(GraphicsControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);

	};
}

