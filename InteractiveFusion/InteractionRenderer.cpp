#include "InteractionRenderer.h"
#include "GraphicsCamera.h"
#include "KeyState.h"
#include "DebugUtility.h"
#include "GraphicsController.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	InteractionRenderer::InteractionRenderer() :
		OpenGLRenderer()
	{
	}

	InteractionRenderer::InteractionRenderer(OpenGLCameraMode _cameraMode)
	{
		cameraMode = _cameraMode;
	}

	InteractionRenderer::~InteractionRenderer()
	{
	}

	void InteractionRenderer::Initialize(GraphicsController& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void InteractionRenderer::SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		RenderSceneInformation(_glControl,_modelData);
	}

	void InteractionRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"InteractionRenderer::CleanUp");
		OpenGLRenderer::CleanUp();
	}
}