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
		void ChangeAxis(PlaneCutAxis _axis);
		void ChangeTransformation(PlaneCutTransformation _transformationMode);
		void ResetPlaneRotation();
	protected:
		virtual void HandleLeftMouseClick(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void DrawForColorPicking(GraphicsControl& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		virtual void CleanUp();

		bool CheckRotationLimitsX(float _offSetX);
		bool CheckRotationLimitsY(float _offSetY);
		void HandlePlaneTransformation(GraphicsControl& _glControl);
		

		void UpdatePlaneTranslation();
		void ApplyPlaneRotation(GraphicsControl& _glControl, float _offSetX, float _offsetY);
		void ApplyPlaneTranslation(float _offSet);

	private:
		
		std::shared_ptr<SimplePlaneRenderable3D> plane;
		int oldPosX;
		int oldPosY;
		bool firstClick = false;
		PlaneCutAxis currentAxis = PlaneCutAxis::AxisY;
		PlaneCutTransformation currentTransformation = PlaneCutTransformation::Translate;
		bool rotationEnabled = false;
		
	};
}

