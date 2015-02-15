#pragma once
#include "Selector.h"
#include "SimplePlaneRenderable3D.h"
#include "EnumDeclarations.h"
#include <memory>

namespace InteractiveFusion {

	class PlaneSelector :
		public Selector
	{
	public:
		PlaneSelector(std::shared_ptr<SimplePlaneRenderable3D> _plane);
		void SetPlane(std::shared_ptr<SimplePlaneRenderable3D> _plane);
		~PlaneSelector();

		void ApplyModeChange();
		void ChangeMode(PlaneCutMode _mode);

	protected:
		virtual void HandleLeftMouseClick(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper, int _selectedIndex);
		virtual void DrawForColorPicking(OpenGLControl* _glControl, ModelData* _modelData, IconData* _overlayHelper);
		virtual void CleanUp();

		
		void HandlePlaneTransformation(OpenGLControl* _glControl);
		void ResetPlaneRotation();

		void UpdatePlaneTranslation();
		void ApplyPlaneRotation(OpenGLControl* _glControl, float _offSet);
		void ApplyPlaneTranslation(float _offSet);
	private:
		
		std::shared_ptr<SimplePlaneRenderable3D> plane;
		int oldPosX;
		int oldPosY;
		bool firstClick = false;
		PlaneCutMode currentMode = AxisY;
		
	};
}

