#include <Windows.h>
#include <glm/glm.hpp>
#include <string>
#include <memory>
#include "MeshContainer.h"
#pragma once

namespace InteractiveFusion {
	class IScanner
	{
	public:
		//IScanner();
		//virtual ~IScanner();

		virtual void Initialize(HWND handle, HWND depthView, HWND reconstructionView, HWND residualsView) = 0;
		virtual void PauseRendering() = 0;
		virtual glm::mat4 GetTransformedCameraMatrix() = 0;
		virtual void ResolvePotentialSensorConflict(LPARAM _lParam) = 0;
		virtual void HandleCompletedFrame() = 0;
		virtual void UpdateSensorStatus(WPARAM wParam) = 0;
		virtual void ChangeVolumeSize(int newSize) = 0;
		virtual float GetFramesPerSecond() = 0;
		virtual std::wstring GetAndResetStatus() = 0;
		virtual int GetGpuMemory() = 0;
		virtual void PrepareMeshSave() = 0;
		virtual void FinishMeshSave() = 0;
		virtual std::vector<Vertex>& GetScannedVertices() = 0;
		virtual std::vector<Triangle>& GetScannedTriangles() = 0;
		virtual void ResetScan() = 0;
		virtual void ResumeRendering() = 0;
		virtual void CleanUp() = 0;
		virtual void PauseProcessing(bool flag) = 0;
		virtual bool IsReconstructionReady() = 0;
		
	};
}

