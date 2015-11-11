#pragma once
#include "OpenGLRenderer.h"

namespace InteractiveFusion {
	class ProcessingRenderer :
		public OpenGLRenderer
	{
	public:
		ProcessingRenderer();
		ProcessingRenderer(OpenGLCameraMode _cameraMode);
		virtual ~ProcessingRenderer();

		virtual void Initialize(GraphicsController& _glControl);
		virtual void SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData);

		virtual void CleanUp();
	};
}

