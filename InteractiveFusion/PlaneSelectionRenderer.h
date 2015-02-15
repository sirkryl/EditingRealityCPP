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

		virtual void Initialize(OpenGLControl* _glControl);
		virtual void Render(OpenGLControl* _glControl, ModelData* _modelData, IconData* _iconData);

		virtual void CleanUp();
	protected:
		std::unique_ptr<MeshContainer2D> planeSelectionQuestionOverlay;

		virtual void InitializeOverlays(OpenGLControl* _glControl);

		void ShowPlaneSelectionOverlay();
	};
}

