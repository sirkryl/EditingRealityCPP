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

		virtual void Initialize(GraphicsControl* _glControl);
		virtual void Render(GraphicsControl* _glControl, ModelData* _modelData, IconData* _iconData);

		virtual void CleanUp();
	protected:
		std::unique_ptr<MeshContainer2D> planeSelectionQuestionOverlay;

		virtual void InitializeOverlays(GraphicsControl* _glControl);

		void ShowPlaneSelectionOverlay();
	};
}

