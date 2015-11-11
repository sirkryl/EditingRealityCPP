#include "PlaneCutRenderer.h"

#include "DebugUtility.h"
#include "GraphicsController.h"
#include "ModelData.h"
#include "IconData.h"

#include <glm/gtc/matrix_transform.hpp>
namespace InteractiveFusion {

	

	PlaneCutRenderer::PlaneCutRenderer() :
		OpenGLRenderer()
	{
	}

	PlaneCutRenderer::PlaneCutRenderer(OpenGLCameraMode _cameraMode, std::shared_ptr<SimplePlaneRenderable3D> _cutPlane)
	{
		cameraMode = _cameraMode;
		plane = _cutPlane;
	}

	PlaneCutRenderer::~PlaneCutRenderer()
	{
	}

	void PlaneCutRenderer::Initialize(GraphicsController& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);		
	}

	void PlaneCutRenderer::SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		plane->Draw(_glControl.GetProjectionMatrix(), _glControl.GetViewMatrix());

		RenderSceneInformation(_glControl, _modelData);
	}

	void PlaneCutRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"PlaneCutRenderer::CleanUp");
		//planeRenderer.CleanUp();
		plane->CleanUp();
		OpenGLRenderer::CleanUp();
	}
}