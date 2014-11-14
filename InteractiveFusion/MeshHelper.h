#pragma once

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

	glm::vec3 GetCombinedCenterPoint();
private:
	
};

extern int numberOfVertices;
extern int numberOfFaces;

extern MeshHelper meshHelper;