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

		virtual void Initialize(GraphicsController& _glControl);
		virtual void SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData);

		virtual void CleanUp();

	protected:
		
	};
}

