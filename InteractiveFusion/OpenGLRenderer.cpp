#include "OpenGLRenderer.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include "GraphicsControl.h"
#include "ModelData.h"
#include "IconData.h"
#include "MeshContainer2D.h"

using namespace std;

namespace InteractiveFusion {

	OpenGLText OpenGLRenderer::glText;

	unique_ptr<MeshContainer2D> OpenGLRenderer::backgroundGradient;
	unique_ptr<MeshContainer2D> OpenGLRenderer::statusMessageBackground;
	unique_ptr<MeshContainer2D> OpenGLRenderer::alphaOverlay;
	bool OpenGLRenderer::initialized = false;
	

	OpenGLRenderer::OpenGLRenderer()
	{
		cameraMode = Free;
	}

	OpenGLRenderer::OpenGLRenderer(OpenGLCameraMode _cameraMode)
	{
		cameraMode = _cameraMode;
	}

	OpenGLRenderer::~OpenGLRenderer()
	{
	}


	void OpenGLRenderer::Initialize(GraphicsControl& _glControl)
	{
		viewportWidth = _glControl.GetViewportWidth();
		viewportHeight = _glControl.GetViewportHeight();
		dots.push_back(L".");
		dots.push_back(L"..");
		dots.push_back(L"...");
		dotCount = 0;
		if (OpenGLRenderer::glText.IsInitialized())
		{
			DebugUtility::DbgOut(L"OpenGLRenderer::Initialized::Already initialized");
			return;
		}
		

		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);
		glDepthFunc(GL_LESS);
		glDepthRange(-1.0f, 1.0f);

		glText.Initialize("data\\fonts\\OpenSans-Regular.ttf");

		InitializeOverlays(_glControl);
		OpenGLRenderer::initialized = true;
	}

	void OpenGLRenderer::InitializeOverlays(GraphicsControl& _glControl)
	{
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::Begin");
		vector<Vertex> statusMessageBackgroundVertices;

		ColorIF backgroundColor{ (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().r / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().g / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().b / 255.0f };
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::11");
		statusMessageBackgroundVertices.push_back(Vertex(-1.0f, -0.15f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		statusMessageBackgroundVertices.push_back(Vertex(1.0f, -0.15f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		statusMessageBackgroundVertices.push_back(Vertex(1.0f, 0.15f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		statusMessageBackgroundVertices.push_back(Vertex(-1.0f, 0.15f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		statusMessageBackgroundVertices.push_back(Vertex(-1.0f, -0.15f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		statusMessageBackgroundVertices.push_back(Vertex(1.0f, 0.15f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::22");
		statusMessageBackground = unique_ptr<MeshContainer2D>(new MeshContainer2D(statusMessageBackgroundVertices));
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::33");
		statusMessageBackground->SetShaderProgram(_glControl.GetShader(Orthographic));
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::44");
		statusMessageBackground->SetAlpha(0.5f);
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::55");
		statusMessageBackground->GenerateBuffers();
		DebugUtility::DbgOut(L"OpenGLRenderer::InitializeOverlays::66");
		vector<Vertex> overlayVertices;

		overlayVertices.push_back(Vertex(-1.0f, -1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		overlayVertices.push_back(Vertex(1.0f, -1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		overlayVertices.push_back(Vertex(1.0f, 1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		overlayVertices.push_back(Vertex(-1.0f, 1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		overlayVertices.push_back(Vertex(-1.0f, -1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));
		overlayVertices.push_back(Vertex(1.0f, 1.0f, 0.0f,
			backgroundColor.r, backgroundColor.g, backgroundColor.b,
			0.0f, 0.0f, 0.0f));

		alphaOverlay = unique_ptr<MeshContainer2D>(new MeshContainer2D(overlayVertices));
		alphaOverlay->SetShaderProgram(_glControl.GetShader(Orthographic));
		alphaOverlay->SetAlpha(0.5f);
		alphaOverlay->GenerateBuffers();

		GLfloat maxZ = 1.f - std::numeric_limits<GLfloat>::epsilon();

		vector<Vertex> backgroundGradientVertices;

		/*float bgStep = 50;

		float step = 2.0f / (float)bgStep;
		float colorStep = 0.17f / (float)bgStep;
		for (int i = 0; i < bgStep; i++)
		{
			
			float xLeft = -1.0f;
			float xRight = 1.0f;
			float yBottom = -1.0f + step*(float)i;
			float yTop = -1.0f + step*(float)(i + 1.0f);
			float cTop = 0.04f + colorStep*(float)(i + 1.0f);
			float cBottom = 0.04f + colorStep*(float)i;
			
			DebugUtility::DbgOut(L"BackgroundGradient: xLeft: ", xLeft);
			DebugUtility::DbgOut(L"BackgroundGradient: xRight: ", xRight);
			DebugUtility::DbgOut(L"BackgroundGradient: yBottom: ", yBottom);
			DebugUtility::DbgOut(L"BackgroundGradient: yTop: ", yTop);
			DebugUtility::DbgOut(L"BackgroundGradient: cTop: ", cTop);
			DebugUtility::DbgOut(L"BackgroundGradient: cBottom: ", cBottom);

			backgroundGradientVertices.push_back(Vertex(xLeft, yBottom, maxZ,
				cBottom, cBottom, cBottom,
				0.0f, 0.0f, 0.0f));
			backgroundGradientVertices.push_back(Vertex(xRight, yBottom, maxZ,
				cBottom, cBottom, cBottom,
				0.0f, 0.0f, 0.0f));
			backgroundGradientVertices.push_back(Vertex(xRight, yTop, maxZ,
				cTop, cTop, cTop,
				0.0f, 0.0f, 0.0f));
			backgroundGradientVertices.push_back(Vertex(xLeft, yTop, maxZ,
				cTop, cTop, cTop,
				0.0f, 0.0f, 0.0f));
			backgroundGradientVertices.push_back(Vertex(xLeft, yBottom, maxZ,
				cBottom, cBottom, cBottom,
				0.0f, 0.0f, 0.0f));
			backgroundGradientVertices.push_back(Vertex(xRight, yTop, maxZ,
				cTop, cTop, cTop,
				0.0f, 0.0f, 0.0f));
		}*/

		float colorTop = 11.0f / 255.0f;
		float colorBottom = 55.0f / 255.0f;

		backgroundGradientVertices.push_back(Vertex(-1.0f, -1.0f, maxZ,
			colorBottom, colorBottom, colorBottom,
			0.0f, 0.0f, 0.0f));
		backgroundGradientVertices.push_back(Vertex(1.0f, -1.0f, maxZ,
			colorBottom, colorBottom, colorBottom,
			0.0f, 0.0f, 0.0f));
		backgroundGradientVertices.push_back(Vertex(1.0f, 1.0f, maxZ,
			colorTop, colorTop, colorTop,
			0.0f, 0.0f, 0.0f));
		backgroundGradientVertices.push_back(Vertex(-1.0f, 1.0f, maxZ,
			colorTop, colorTop, colorTop,
			0.0f, 0.0f, 0.0f));
		backgroundGradientVertices.push_back(Vertex(-1.0f, -1.0f, maxZ,
			colorBottom, colorBottom, colorBottom,
			0.0f, 0.0f, 0.0f));
		backgroundGradientVertices.push_back(Vertex(1.0f, 1.0f, maxZ,
			colorTop, colorTop, colorTop,
			0.0f, 0.0f, 0.0f));
		backgroundGradient = unique_ptr<MeshContainer2D>(new MeshContainer2D(backgroundGradientVertices));
		backgroundGradient->SetShaderProgram(_glControl.GetShader(Orthographic));
		backgroundGradient->SetAlpha(1.0f);
		backgroundGradient->GenerateBuffers();

		
	}

	void OpenGLRenderer::Render(GraphicsControl& _glControl, ModelData& _modelData, IconData& _iconData)
	{

		if (_modelData.IsReadyForRendering())
			_modelData.Draw(_glControl.GetProjectionMatrix(), _glControl.GetViewMatrix());
		if (_iconData.IsReadyForRendering() && cameraMode == Sensor)
			_iconData.Draw(_glControl.GetViewportWidth(), _glControl.GetViewportHeight());
	}

	void OpenGLRenderer::PrepareRender(GraphicsControl& _glControl)
	{
		viewportWidth = _glControl.GetViewportWidth();
		viewportHeight = _glControl.GetViewportHeight();

		if (!viewportBackgroundInitialized)
		{
			viewportBackground = ColorIF{ (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().r / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().g / 255.0f, (float)StyleSheet::GetInstance()->GetInnerBackgroundColor().b / 255.0f };
			viewportBackgroundInitialized = true;
		}
		glClearColor(viewportBackground.r, viewportBackground.g, viewportBackground.b, 1.0f);
		glClearDepth(1.0);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//if (params.wireFrameMode)
		//	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		//else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glEnable(GL_DITHER);
		backgroundGradient->Draw(_glControl.GetViewportWidth(), _glControl.GetViewportHeight());

	}

	void OpenGLRenderer::FinishRender(GraphicsControl& _glControl)
	{
		if (_glControl.IsBusy())
			ShowStatusOverlay(_glControl);

		glEnable(GL_DEPTH_TEST);
		_glControl.SwapBuffers();
	}

	void OpenGLRenderer::ShowStatusOverlay(GraphicsControl& _glControl)
	{
		glDisable(GL_DEPTH_TEST);

		alphaOverlay->Draw(_glControl.GetViewportWidth(), _glControl.GetViewportHeight());
		statusMessageBackground->Draw(_glControl.GetViewportWidth(), _glControl.GetViewportHeight());

		float xPos = 0.0f - _glControl.GetStatusMessage().length() * 0.008f;
		wstring loadString = _glControl.GetStatusMessage() + L"" + dots[(int)floor(dotCount / 100)];
		if (dotCount == 299)
			dotCount = 0;
		else
			dotCount++;

		glText.PrepareForRender();


		/*if (appStatus.GetViewportPercentMessage().size() > 0)
		{
			glText.RenderText(loadString, 25, xPos, 0.05f, 2.0f / viewportWidth, 2.0f / viewportHeight);
			float xPercentPos = -0.05f;
			glText.RenderText(appStatus.GetViewportPercentMessage(), 20, xPercentPos, -0.08f, 2.0f / viewportWidth, 2.0f / viewportHeight);
		}
		else
		{*/
			//glText.RenderText(loadString, 25, xPos, 0.00f, 2.0f / viewportWidth, 2.0f / viewportHeight);

		if (viewportWidth > 100 && viewportWidth < 2000 && viewportHeight > 100 && viewportHeight < 2000)
			glText.RenderText(loadString, 25, xPos, 0.00f, 2.0f / viewportWidth, 2.0f / viewportHeight);
		else
			DebugUtility::DbgOut(L"yes:");
		//}
		glText.FinishRender();
		glEnable(GL_DEPTH_TEST);
	}

	OpenGLCameraMode OpenGLRenderer::GetCameraMode()
	{
		return cameraMode;
	}

	void OpenGLRenderer::SetCameraMode(OpenGLCameraMode _cameraMode)
	{
		cameraMode = _cameraMode;
	}
	void OpenGLRenderer::CleanUp()
	{
		if (!glText.IsInitialized())
			return;
		//shaderColor.DeleteProgram();
		//shader2d.DeleteProgram();
		glText.CleanUp();

		backgroundGradient->CleanUp();
		alphaOverlay->CleanUp();
		statusMessageBackground->CleanUp();
		//for (int i = 0; i < 4; i++)shaders[i].DeleteShader();

	}
}