#include "ProcessingRenderer.h"
#include "DebugUtility.h"
#include "GraphicsController.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	ProcessingRenderer::ProcessingRenderer() :
		OpenGLRenderer()
	{
	}

	ProcessingRenderer::ProcessingRenderer(OpenGLCameraMode _cameraMode)
	{
		cameraMode = _cameraMode;
	}

	ProcessingRenderer::~ProcessingRenderer()
	{
	}

	void ProcessingRenderer::Initialize(GraphicsController& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void ProcessingRenderer::SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		RenderSceneInformation(_glControl, _modelData);
	}

	void ProcessingRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"ProcessingRenderer::CleanUp");
		OpenGLRenderer::CleanUp();
	}
}