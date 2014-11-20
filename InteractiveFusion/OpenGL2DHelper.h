#pragma once
#include "VCGMeshContainer.h"



class OpenGL2DHelper
{
public:
	void InitialLoadFromFile(const char* fileName, int colorCode);

	void DrawAll();
	void DrawAllBB();

	bool isOpen = false;

	void GenerateBuffers();

	void CleanUp();
private:
	std::vector<shared_ptr<VCGMeshContainer>> meshData2d;
};

extern OpenGL2DHelper gl2DHelper;