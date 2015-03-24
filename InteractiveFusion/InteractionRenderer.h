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

		virtual void Initialize(GraphicsControl& _glControl);
		virtual void Render(GraphicsControl& _glControl, ModelData& _modelData, IconData& _iconData);

		virtual void CleanUp();
	};
}

