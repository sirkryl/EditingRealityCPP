#pragma once
#include "VcgMeshContainer.h"

class MeshHelper
{
public:

	void InitialLoadFromFile(const char* fileName);

	void DrawOriginalMesh();

	void DrawAll();

	void DrawAllForColorPicking();

	void FillHoles(int holeSize);
	void FillHoles(int index, int holeSize);
	void RemoveSmallComponents(int size);

	void CleanMesh();

	void CombineAndExport();

	void CleanAndParse(const char* fileName, std::vector<Vertex> &startingVertices, std::vector<Triangle> &startingIndices);

	void GenerateOriginalBuffers();
	void GenerateBuffers();

	void DeleteMesh(int index);

	int DuplicateMesh(int index);

	int GetNumberOfVertices();

	int GetNumberOfFaces();
	void HighlightObjectsInOriginal(std::vector<int> triangles, ColorIF color, bool additive);
	void HighlightObjects(int index, std::vector<int> triangles, ColorIF color, bool additive);
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
extern shared_ptr < VCGMeshContainer > originalMesh;
extern MeshHelper meshHelper;