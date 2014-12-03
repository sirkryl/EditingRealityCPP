#pragma once
#include "VCGMeshContainer.h"


class OpenGL2DHelper
{
public:
	void InitialLoadFromFile(const char* fileName, int colorCode);
	void InitializeRectangle();
	bool IsRectangleInitialized();
	void DrawRectangle();
	void DrawRectangle(float y);
	void DrawAll();
	void DrawAllBB();

	bool isOpen = false;

	void GenerateBuffers();

	void CleanUp();
private:
	std::vector<shared_ptr<VCGMeshContainer>> meshData2d;

	std::vector<Vertex> rectangleVertices;
	GLuint rectangleVBO, rectangleVAO;
};

extern OpenGL2DHelper gl2DHelper;