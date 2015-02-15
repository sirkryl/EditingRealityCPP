#pragma once
#include "MeshContainer2D.h"
#include "OpenGLShaderProgram.h"
#include <memory>
#include <unordered_map>
namespace InteractiveFusion {
	class IconData
	{
	public:
		IconData();
		~IconData();

		bool LoadFromFile(const char* _fileName, int _colorCode);
		bool LoadFromFile(const char* _fileNameDefault, const char* _fileNameHover, int _colorCode);
		void GenerateBuffers();
		bool IsReadyForRendering();
		void DrawForColorPicking(int _colorCode, int _viewportWidth, int _viewportHeight);
		void DrawForColorPicking(int _viewportWidth, int _viewportHeight);
		void Draw(int _colorCode, int _viewportWidth, int _viewportHeight);
		void Draw(int _viewportWidth, int _viewportHeight);
		void SetHovered(int _colorCode, bool _isHovered);
		bool IsValidKey(int _colorCode);
		void SetDefaultShaderProgram(OpenGLShaderProgram _defaultProgram);
		void CleanUp();


	protected:
		std::unordered_map<int, std::unique_ptr<MeshContainer2D>> defaultIconData;
		std::unordered_map<int, std::unique_ptr<MeshContainer2D>> hoveredIconData;
		std::unordered_map<int, bool> isHoveredMap;

		OpenGLShaderProgram defaultShaderProgram;
	};
}