#include "SegmentationRenderer.h"
#include "DebugUtility.h"
#include "GraphicsController.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {


	SegmentationRenderer::SegmentationRenderer() :
		OpenGLRenderer()
	{
	}

	SegmentationRenderer::SegmentationRenderer(OpenGLCameraMode _cameraMode)
	{
		cameraMode = _cameraMode;
	}

	SegmentationRenderer::~SegmentationRenderer()
	{
	}

	void SegmentationRenderer::Initialize(GraphicsController& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void SegmentationRenderer::SubRender(GraphicsController& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		RenderSceneInformation(_glControl, _modelData);
	}

	void SegmentationRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"SegmentationRenderer::CleanUp");
		OpenGLRenderer::CleanUp();
	}
}