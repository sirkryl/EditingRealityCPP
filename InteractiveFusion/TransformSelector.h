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

		virtual void HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);

		bool InitializeTransformationBasePoint(ModelData* _modelData, int _selectedIndex);

		void ResetTransformationBasePoint();

		void HandleRotation(OpenGLControl* _glControl, ModelData* _modelData, int _selectedIndex);

		void TransformSelector::HandleScale(OpenGLControl* _glControl, ModelData* _modelData, int _selectedIndex);

		virtual void DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);

	};
}

