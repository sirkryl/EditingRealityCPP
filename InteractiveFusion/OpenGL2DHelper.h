#pragma once
#include "Mesh2D.h"

#define TRASH_BIN_COLOR 666

class OpenGL2DHelper
{
public:
	void InitialLoadFromFile(const char* fileName);

	void DrawAll();
	void DrawAllBB();

	bool isOpen = false;

	void GenerateBuffers();

	void CleanUp();
private:
	std::vector<shared_ptr<Mesh2D>> meshData2d;
};

extern OpenGL2DHelper gl2DHelper;