#pragma once
#include "OpenGLRenderer.h"
#include "SimplePlaneRenderable3D.h"
#include <memory>
namespace InteractiveFusion {
	class PlaneCutRenderer :
		public OpenGLRenderer
	{
	public:
		PlaneCutRenderer();
		PlaneCutRenderer(OpenGLCameraMode _cameraMode, std::shared_ptr<SimplePlaneRenderable3D> _cutPlane);
		virtual ~PlaneCutRenderer();

		virtual void Initialize(GraphicsController& _glControl);
		virtual void SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData);

		virtual void CleanUp();

	protected:
		std::shared_ptr<SimplePlaneRenderable3D> plane;
	};
}

