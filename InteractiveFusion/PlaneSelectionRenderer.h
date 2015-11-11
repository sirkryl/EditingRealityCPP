#pragma once
#include "OpenGLRenderer.h"

namespace InteractiveFusion {
	class PlaneSelectionRenderer :
		public OpenGLRenderer
	{
	public:
		PlaneSelectionRenderer();
		PlaneSelectionRenderer(OpenGLCameraMode _cameraMode);
		virtual ~PlaneSelectionRenderer();

		virtual void Initialize(GraphicsController& _glControl);
		virtual void SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData);

		virtual void CleanUp();
	protected:
		std::unique_ptr<MeshContainer2D> planeSelectionQuestionOverlay;

		virtual void InitializeOverlays(GraphicsController& _glControl);

		void ShowPlaneSelectionOverlay();
	};
}

