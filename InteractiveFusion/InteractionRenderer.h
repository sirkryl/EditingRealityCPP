#pragma once
#include "OpenGLRenderer.h"

namespace InteractiveFusion {
	class InteractionRenderer :
		public OpenGLRenderer
	{
	public:
		InteractionRenderer();
		InteractionRenderer(OpenGLCameraMode _cameraMode);
		virtual ~InteractionRenderer();

		virtual void Initialize(OpenGLControl* _glControl);
		virtual void Render(OpenGLControl* _glControl, ModelData* _modelData, IconData* _iconData);

		virtual void CleanUp();
	};
}

