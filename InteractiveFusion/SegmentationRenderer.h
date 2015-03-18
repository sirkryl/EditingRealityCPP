#pragma once
#include "OpenGLRenderer.h"


namespace InteractiveFusion {
	class SegmentationRenderer :
		public OpenGLRenderer
	{
	public:
		SegmentationRenderer();
		SegmentationRenderer(OpenGLCameraMode _cameraMode);
		virtual ~SegmentationRenderer();

		virtual void Initialize(GraphicsControl* _glControl);
		virtual void Render(GraphicsControl* _glControl, ModelData* _modelData, IconData* _iconData);

		virtual void CleanUp();

	protected:
		
	};
}

