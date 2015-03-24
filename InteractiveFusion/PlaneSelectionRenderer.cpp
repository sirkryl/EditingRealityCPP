#include "PlaneSelectionRenderer.h"
#include "DebugUtility.h"
#include "StyleSheet.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"

namespace InteractiveFusion {
	PlaneSelectionRenderer::PlaneSelectionRenderer() :
		OpenGLRenderer()
	{
	}

	PlaneSelectionRenderer::PlaneSelectionRenderer(OpenGLCameraMode _cameraMode)
	{
		cameraMode = _cameraMode;
	}

	PlaneSelectionRenderer::~PlaneSelectionRenderer()
	{
	}

	void PlaneSelectionRenderer::Initialize(GraphicsControl& _glControl)
	{
		OpenGLRenderer::Initialize(_glControl);
	}

	void PlaneSelectionRenderer::InitializeOverlays(GraphicsControl& _glControl)
	{
		OpenGLRenderer::InitializeOverlays(_glControl);
		std::vector<Vertex> planeSelectionQuestionOverlayVertices;

		ColorIF backgroundColor{ (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().r / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().g / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().b / 255.0f };

		planeSelectionQuestionOverlayVertices.push_back(Vertex(-1.0f, 1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		planeSelectionQuestionOverlayVertices.push_back(Vertex(1.0f, 1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		planeSelectionQuestionOverlayVertices.push_back(Vertex(1.0f, 0.7f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		planeSelectionQuestionOverlayVertices.push_back(Vertex(-1.0f, 0.7f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		planeSelectionQuestionOverlayVertices.push_back(Vertex(-1.0f, 1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		planeSelectionQuestionOverlayVertices.push_back(Vertex(1.0f, 0.7f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));

		planeSelectionQuestionOverlay = std::unique_ptr<MeshContainer2D>(new MeshContainer2D(planeSelectionQuestionOverlayVertices));
		planeSelectionQuestionOverlay->SetAlpha(0.5f);
		planeSelectionQuestionOverlay->GenerateBuffers();
	}

	void PlaneSelectionRenderer::ShowPlaneSelectionOverlay()
	{
		//glDisable(GL_DEPTH_TEST);

		////if (stateManager.GetDeviceClass() == IF_DEVICE_PC)
		////	gl2DHelper.DrawRectangle(0.7f, 0.8f, 0.2f);
		////else if (stateManager.GetDeviceClass() == IF_DEVICE_TABLET)
		//planeSelectionQuestionOverlay->Draw(viewportWidth, viewportHeight);
		////OpenGLRenderer::gl2dHelper.DrawRectangle(0.88f, 0.8f, 0.25f);

		//const std::wstring wallMessage = L"Is the highlighted area (part of) a wall, floor or ceiling?";

		//float xPos = 0.0f - wallMessage.length() * 0.008f;

		//OpenGLRenderer::glText.PrepareForRender();
		//OpenGLRenderer::glText.RenderText(wallMessage, 28, xPos, 0.77f, 2.0f / viewportWidth, 2.0f / viewportHeight);
		//glEnable(GL_DEPTH_TEST);
	}

	void PlaneSelectionRenderer::Render(GraphicsControl& _glControl, ModelData& _modelData, IconData& _iconData)
	{
		OpenGLRenderer::PrepareRender(_glControl);
		OpenGLRenderer::Render(_glControl, _modelData, _iconData);

		ShowPlaneSelectionOverlay();

		OpenGLRenderer::FinishRender(_glControl);

		
	}

	void PlaneSelectionRenderer::CleanUp()
	{
		DebugUtility::DbgOut(L"PlaneSelectionRenderer::CleanUp");
		planeSelectionQuestionOverlay->CleanUp();
		OpenGLRenderer::CleanUp();
	}
}