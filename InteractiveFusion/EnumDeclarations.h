#pragma once

namespace InteractiveFusion
{
	enum ObjectSegmentationType { Euclidean, RegionGrowth };

	enum InteractionMode { None, Duplication, Transformation };

	enum PlaneCutMode {
		AxisX,
		AxisY,
		AxisZ
	};

	enum WindowState { Prepare, Scan, PlaneSelection, Segmentation, PlaneCut, Processing, Interaction };

	enum HelpMessage { IntroHelp, PrepareHelp, ScanHelp, PlaneSelectionHelp, SegmentationHelp, PlaneCutHelp, ProcessingHelp, InteractionHelp };

	enum OpenGLControlEvent { ModelDataUpdated, CopyTemporaryInModelData, CopyTemporaryInNextStateModelData, ResetCurrentStateModelData, ModelHighlightsUpdated, RemoveModelHighlights, RemoveModelData, ResizeOpenGLViewport, ResetModelData, UpdateSegmentation, UpdateHoleFilling, UpdateCutPlane, SetupCutPlaneMode };

	enum OpenGLCameraMode { Free, Sensor };

	enum OpenGLShaderProgramType { Default, Orthographic};
}