#include "InteractionRenderer.h"
#include "GraphicsCamera.h"
#include "KeyState.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
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

	void InteractionRenderer::Initialize(GraphicsControl& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void InteractionRenderer::Render(GraphicsControl& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		OpenGLRenderer::PrepareRender(_glControl);
		OpenGLRenderer::Render(_glControl, _modelData, _iconData);

		//_iconData->Draw(TRASH_BIN);
		//gl2dHelper.DrawTrash();

		glText.PrepareForRender();

		glText.RenderText(L"Clusters: ", _modelData.GetVisibleMeshCount(), 13, -0.98f, 0.93f, 2.0f / _glControl.GetViewportWidth(), 2.0f / _glControl.GetViewportHeight());
		glText.RenderText(L"Vertices: ", _modelData.GetNumberOfVertices(), 13, -0.98f, 0.88f, 2.0f / _glControl.GetViewportWidth(), 2.0f / _glControl.GetViewportHeight());
		glText.RenderText(L"Triangles: ", _modelData.GetNumberOfTriangles(), 13, -0.98f, 0.83f, 2.0f / _glControl.GetViewportWidth(), 2.0f / _glControl.GetViewportHeight());
		glText.FinishRender();
		OpenGLRenderer::FinishRender(_glControl);
	}

	void InteractionRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"InteractionRenderer::CleanUp");
		OpenGLRenderer::CleanUp();
	}
}