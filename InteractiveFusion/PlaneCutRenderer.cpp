#include "PlaneCutRenderer.h"

#include "DebugUtility.h"
#include "OpenGLControl.h"
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

	void PlaneCutRenderer::Initialize(OpenGLControl* _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);		
	}

	void PlaneCutRenderer::Render(OpenGLControl* _glControl, ModelData* _modelData, IconData* _iconData)
	{
		//if (!plane->AreBuffersInitialized() && _modelData->GetUpperBounds().x != -999.0f && _modelData->GetLowerBounds().x != 999.0f)
		//	InitializePlane(_glControl, _modelData);
		
		OpenGLRenderer::PrepareRender(_glControl);
		OpenGLRenderer::Render(_glControl, _modelData, _iconData);

		if (_glControl->IsPlaneRendererVisible())
		{
			plane->Draw(_glControl->GetProjectionMatrix(), _glControl->GetViewMatrix());
		}

		glText.PrepareForRender();

		glText.RenderText(L"Clusters: ", _modelData->GetVisibleMeshCount(), 13, -0.98f, 0.93f, 2.0f / _glControl->GetViewportWidth(), 2.0f / _glControl->GetViewportHeight());
		glText.RenderText(L"Vertices: ", _modelData->GetNumberOfVertices(), 13, -0.98f, 0.88f, 2.0f / _glControl->GetViewportWidth(), 2.0f / _glControl->GetViewportHeight());
		glText.RenderText(L"Triangles: ", _modelData->GetNumberOfTriangles(), 13, -0.98f, 0.83f, 2.0f / _glControl->GetViewportWidth(), 2.0f / _glControl->GetViewportHeight());



		OpenGLRenderer::FinishRender(_glControl);
	}

	void PlaneCutRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"PlaneCutRenderer::CleanUp");
		//planeRenderer.CleanUp();
		plane->CleanUp();
		OpenGLRenderer::CleanUp();
	}
}