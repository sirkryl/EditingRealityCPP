#pragma once

namespace InteractiveFusion
{
	enum ObjectSegmentationType { Euclidean, RegionGrowth };

	enum InteractionMode { None, Duplication, Transformation };

	enum PlaneCutAxis {
		AxisX,
		AxisY,
		AxisZ
	};

	enum class PlaneCutTransformation {
		Translate,
		Rotate,
		None
	};

	enum WindowState { Prepare, Scan, PlaneSelection, Segmentation, PlaneCut, Processing, Interaction };

	enum HelpMessage { IntroHelp, PrepareHelp, ScanHelp, PlaneSelectionHelp, SegmentationHelp, PlaneCutHelp, ProcessingHelp, InteractionHelp };

	enum OpenGLControlEvent {InitialLoading, StateUpdate, ModelDataUpdated, CopyTemporaryInModelData, CopyTemporaryInNextStateModelData, ResetCurrentStateModelData, ModelHighlightsUpdated, RemoveModelHighlights, FillHolesInScene, RemoveModelData, ResizeOpenGLViewport, ResetModelData, UpdateSegmentation, UpdateHoleFilling, UpdateCutPlane, SetupCutPlaneMode };

	enum OpenGLCameraMode { Free, Sensor };

	enum OpenGLShaderProgramType { Default, Orthographic};
}