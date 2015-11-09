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

		virtual void Initialize(GraphicsController& _glControl);
		virtual void Render(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData);

		virtual void CleanUp();
	};
}

