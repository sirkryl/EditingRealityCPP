#include "SegmentationRenderer.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
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

	void SegmentationRenderer::Initialize(GraphicsControl* _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void SegmentationRenderer::Render(GraphicsControl* _glControl, ModelData* _modelData, IconData* _iconData)
	{

		OpenGLRenderer::PrepareRender(_glControl);
		OpenGLRenderer::Render(_glControl, _modelData, _iconData);


		glText.PrepareForRender();

		glText.RenderText(L"Clusters: ", _modelData->GetVisibleMeshCount(), 13, -0.98f, 0.93f, 2.0f / _glControl->GetViewportWidth(), 2.0f / _glControl->GetViewportHeight());
		glText.RenderText(L"Vertices: ", _modelData->GetNumberOfVertices(), 13, -0.98f, 0.88f, 2.0f / _glControl->GetViewportWidth(), 2.0f / _glControl->GetViewportHeight());
		glText.RenderText(L"Triangles: ", _modelData->GetNumberOfTriangles(), 13, -0.98f, 0.83f, 2.0f / _glControl->GetViewportWidth(), 2.0f / _glControl->GetViewportHeight());
		
		

		OpenGLRenderer::FinishRender(_glControl);
	}

	void SegmentationRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"SegmentationRenderer::CleanUp");
		OpenGLRenderer::CleanUp();
	}
}