#pragma once

namespace InteractiveFusion
{
	enum class ObjectSegmentationType { Euclidean, RegionGrowth };

	enum class InteractionMode { None, Duplication, Transformation };

	enum class PlaneCutAxis {
		AxisX,
		AxisY,
		AxisZ
	};

	enum class PlaneCutTransformation {
		Translate,
		Rotate,
		None
	};

	enum class WindowState { Prepare = 0, Scan = 1, PlaneSelection = 2, Segmentation = 3, PlaneCut = 4, Processing = 5, Interaction = 6 };

	enum class HelpMessage { IntroHelp, PrepareHelp, ScanHelp, PlaneSelectionHelp, SegmentationHelp, PlaneCutHelp, ProcessingHelp, InteractionHelp };

	enum class OpenGLControlEvent {InitialLoading, StateUpdate, ModelDataUpdated, CopyTemporaryInModelData, CopyTemporaryInNextStateModelData, ResetCurrentStateModelData, ModelHighlightsUpdated, RemoveModelHighlights, FillHolesInScene, RemoveModelData, ResizeOpenGLViewport, ResetModelData, UpdateSegmentation, UpdateHoleFilling, UpdateCutPlane, SetupCutPlaneMode };

	enum class OpenGLCameraMode { Free, Sensor };

	enum class OpenGLShaderProgramType { Default, Orthographic};
}