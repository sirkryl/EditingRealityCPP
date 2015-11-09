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
		virtual void HandleLeftMouseClick(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseDown(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void HandleLeftMouseRelease(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper, int _selectedIndex);
		virtual void DrawForColorPicking(GraphicsController& _glControl, ModelData& _modelData, IconData& _overlayHelper);
		virtual void CleanUp();

		bool CheckRotationLimitsX(float _offSetX);
		bool CheckRotationLimitsY(float _offSetY);
		void HandlePlaneTransformation(GraphicsController& _glControl);
		

		void UpdatePlaneTranslation();
		void UpdatePreview(GraphicsController& _glControl, ModelData& _modelData, int _selectedIndex);
		void ApplyPlaneRotation(GraphicsController& _glControl, float _offSetX, float _offsetY);
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

