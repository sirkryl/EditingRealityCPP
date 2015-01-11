#pragma once
#include "VcgMeshContainer.h"

class MeshHelper
{
public:

	void InitialLoadFromFile(const char* fileName);

	void DrawOriginalMesh();

	void DrawAll();

	void DrawAllForColorPicking();
	void Draw(int index);
	void FillHoles(int holeSize);
	void FillHoles(int index, int holeSize);
	void RemoveSmallComponents(int size);
	int GetVisibleMeshCount();
	void CleanMesh();
	void SetVerticesInPlane(int index, glm::vec3 planeCenter);
	void CombineAndExport();
	void Export(int index);
	void Export(int index, string fileName, bool saveDialog, bool align = false);
	void ExportForUnity();
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

	void SetGroundAlignmentRotation(glm::mat4 alignmentRotation);
	glm::vec3 GetCombinedCenterPoint();
private:
	glm::mat4 groundAlignmentRotation;
};

//extern int numberOfVertices;
//extern int numberOfFaces;
extern std::vector<shared_ptr<VCGMeshContainer>> meshData;
extern shared_ptr < VCGMeshContainer > originalMesh;
extern MeshHelper meshHelper;