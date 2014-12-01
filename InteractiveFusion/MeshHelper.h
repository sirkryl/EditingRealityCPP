#pragma once
#include "VcgMeshContainer.h"

class MeshHelper
{
public:

	void InitialLoadFromFile(const char* fileName);

	void DrawAll();

	void DrawAllForColorPicking();

	void FillHoles(int holeSize);

	void RemoveSmallComponents(int size);

	void CleanMesh();

	void CombineAndExport();

	void CleanAndParse(const char* fileName, std::vector<Vertex> &startingVertices, std::vector<Triangle> &startingIndices);

	void GenerateBuffers();

	void DeleteMesh(int index);

	int DuplicateMesh(int index);

	int GetNumberOfVertices();

	int GetNumberOfFaces();
	void HighlightObjects(int index, std::vector<int> triangles, ColorIF color);
	void RemoveHighlightObjects(int index);
	void RemoveAllHighlights();
	void ResetAll();

	glm::vec3 GetCombinedCenterPoint();
private:
	int numberOfVertices = 0;
	int numberOfFaces = 0;
};

//extern int numberOfVertices;
//extern int numberOfFaces;
extern std::vector<shared_ptr<VCGMeshContainer>> meshData;

extern MeshHelper meshHelper;