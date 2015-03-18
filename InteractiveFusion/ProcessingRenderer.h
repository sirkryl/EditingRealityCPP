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

		virtual void Initialize(GraphicsControl* _glControl);
		virtual void Render(GraphicsControl* _glControl, ModelData* _modelData, IconData* _iconData);

		virtual void CleanUp();
	};
}

