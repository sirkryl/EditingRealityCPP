#pragma once
#define NOMINMAX

#include "OpenGLShaderProgram.h"
#include "OpenGLText.h"
#include "EnumDeclarations.h"
#include "CommonStructs.h"
#include <memory>
#include <vector>
#include <string>
namespace InteractiveFusion {

	class IconData;
	class ModelData;
	class MeshContainer2D;
	class OpenGLControl;
	class OpenGLRenderer
	{
	public:
		OpenGLRenderer();
		OpenGLRenderer(OpenGLCameraMode _cameraMode);
		virtual ~OpenGLRenderer();

		

		virtual void Initialize(OpenGLControl* _glControl);
		virtual void Render(OpenGLControl* _glControl, ModelData* _modelData, IconData* _iconData);

		OpenGLCameraMode GetCameraMode();
		void SetCameraMode(OpenGLCameraMode _cameraMode);

		virtual void CleanUp();

	protected:

		static OpenGLText glText;
		static std::unique_ptr<MeshContainer2D> backgroundGradient;
		static std::unique_ptr<MeshContainer2D> statusMessageBackground;
		static std::unique_ptr<MeshContainer2D> alphaOverlay;
		OpenGLCameraMode cameraMode;
		int viewportWidth;
		int viewportHeight;
		ColorIF viewportBackground;
		bool viewportBackgroundInitialized = false;
		static bool initialized;

		virtual void InitializeOverlays(OpenGLControl* _glControl);

		virtual void PrepareRender(OpenGLControl* _glControl);
		virtual void FinishRender(OpenGLControl* _glControl);
		virtual void ShowStatusOverlay(OpenGLControl* _glControl);

		int dotCount;
		std::vector<std::wstring> dots;

	};
}

