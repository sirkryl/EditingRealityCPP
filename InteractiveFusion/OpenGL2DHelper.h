#pragma once
#include "VCGMeshContainer.h"


class OpenGL2DHelper
{
public:
	void InitialLoadFromFile(const char* fileName, const char* fileName2, int colorCode);
	void InitializeRectangle();
	void InitializeButton(int id, float x, float y, float w, float h, ColorIF defaultColor, ColorIF pressedColor, wstring text);
	bool IsRectangleInitialized();
	bool AreBuffersGenerated();
	void DrawRectangle();
	void DrawRectangle(float y, float alpha);
	void DrawRectangle(float y, float alpha, float h);
	void DrawTrash();
	void DrawTrashBB();
	void DrawButtons();
	void DrawButtonsBB();

	void UnselectButtons();
	bool SelectButton(int colorCode);

	bool isTrashOpen = false;

	void GenerateBuffers();
	void CleanUp();
private:
	std::vector<shared_ptr<VCGMeshContainer>> meshData2d;

	std::vector<Vertex> rectangleVertices;
	GLuint rectangleVBO, rectangleVAO;

	bool buffersInitialized = false;
};

extern OpenGL2DHelper gl2DHelper;