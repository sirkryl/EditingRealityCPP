#include "ProcessingRenderer.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
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

	void ProcessingRenderer::Initialize(GraphicsControl& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void ProcessingRenderer::Render(GraphicsControl& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		OpenGLRenderer::PrepareRender(_glControl);
		OpenGLRenderer::Render(_glControl, _modelData, _iconData);

		glText.PrepareForRender();

		glText.RenderText(L"Clusters: ", _modelData.GetVisibleMeshCount(), 13, -0.98f, 0.93f, 2.0f / _glControl.GetViewportWidth(), 2.0f / _glControl.GetViewportHeight());
		glText.RenderText(L"Vertices: ", _modelData.GetNumberOfVertices(), 13, -0.98f, 0.88f, 2.0f / _glControl.GetViewportWidth(), 2.0f / _glControl.GetViewportHeight());
		glText.RenderText(L"Triangles: ", _modelData.GetNumberOfTriangles(), 13, -0.98f, 0.83f, 2.0f / _glControl.GetViewportWidth(), 2.0f / _glControl.GetViewportHeight());
		glText.FinishRender();
		OpenGLRenderer::FinishRender(_glControl);
	}

	void ProcessingRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"ProcessingRenderer::CleanUp");
		OpenGLRenderer::CleanUp();
	}
}