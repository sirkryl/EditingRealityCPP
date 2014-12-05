#pragma once
#include<vcg/complex/complex.h>
#include "common.h"

class VCGFace;
class VCGVertex;
class VCGEdge;

struct VCGUsedTypes : public vcg::UsedTypes<	vcg::Use<VCGVertex>::AsVertexType, vcg::Use<VCGEdge>::AsEdgeType, vcg::Use<VCGFace>::AsFaceType>{};

class VCGVertex : public vcg::Vertex< VCGUsedTypes, vcg::vertex::VEAdj, vcg::vertex::Mark, vcg::vertex::VFAdj, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags >{};
class VCGEdge : public vcg::Edge<   VCGUsedTypes> {};
class VCGFace : public vcg::Face<   VCGUsedTypes, vcg::face::VFAdj, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags, vcg::face::Normal3f, vcg::face::Mark > {};
class VCGMesh : public vcg::tri::TriMesh< std::vector<VCGVertex>, std::vector<VCGFace>, std::vector<VCGEdge> > {};

class VCGMeshContainer
{
public:
	VCGMeshContainer();
	~VCGMeshContainer();
	void Load2DMesh(const char* filename);
	void LoadMesh(const char* filename);
	void LoadMesh(std::vector<Vertex> vertices, std::vector<Triangle> indices);

	void ParseData();
	void ParseData(std::vector<Vertex> inputVertices, std::vector<Triangle> inputIndices);

	void ConvertToVCG();
	void ConvertToVCG(std::vector<Vertex> inputVertices, std::vector<Triangle> inputIndices);

	void GenerateVAO();
	void GenerateBOs();

	void UpdateBuffers();

	bool AreBuffersInitialized();

	void HighlightObjects(std::vector<int> objTriangles, ColorIF color, bool additive);
	void ResetHighlights();
	void CleanMesh();
	int MergeCloseVertices(float threshold);
	void LaplacianSmooth(int step);
	void RemoveNonManifoldFace();
	int RemoveSmallComponents(int compSize);
	int FillHoles(int holeSize);
	void UnsharpColor(float alpha);

	void CleanAndParse(std::vector<Vertex> &startingVertices, std::vector<Triangle> &startingIndices);

	void Draw();
	void DrawBB();

	void AttachToCursor(glm::vec3 nearPoint, glm::vec3 farPoint, int distance);
	bool GetHitPoint(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output, glm::vec3 &outputNormal, bool snapToVertex);
	bool CheckCollision(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output);
	void TranslateVerticesToPoint(glm::vec3 point, std::vector<int> orien);
	
	bool IsLoaded();

	void SetSelected(bool val);
	void SetColorCode(int value);
	void SetTranslation(glm::vec3 trans);
	void SetAngleX(bool positive);
	void SetAngleY(bool positive);
	void SetAngleZ(bool positive);
	void SetScale(bool positive);
	void ResetSelectedTransformation();

	void SetWall(bool flag);
	void Set2D(bool flag);
	void SetDuplicate(bool flag);
	void SetPlaneParameters(float x, float y, float z, float d);

	
	std::vector<Vertex> GetVertices();
	std::vector<Triangle> GetIndices();
	int GetNumberOfVertices();
	int GetNumberOfTriangles();
	int GetColorCode();
	glm::vec3 GetCenterPoint();
	std::vector<int> GetOrientation();
	bool IsWall();
	bool IsDuplicate();
	void ToggleSelectedColor(bool flag);
	void TogglePreviewSelection(bool flag);
	void IsOverTrash(bool flag);
	void ClearMesh();
private:
	VCGMesh currentMesh;
	void SetSnapTransform(std::vector<int> orien);
	glm::vec3 FindClosestPoint(glm::vec3 inputPoint);
	glm::vec3 offSet;
	glm::vec3 snapPoint;

	bool is2D = false;
	bool isDuplicate = false;
	bool isOverTrash = false;
	std::vector<Vertex> vertices;
	std::vector<Vertex> verticesWithHighlights;
	std::vector<Triangle> indices;
	std::vector<float> storedColors;

	float planeParameters[4];
	std::vector<int> orientation;
	// -1 = no plane, 0 = x, 1 = y, 2 = z
	glm::mat4 storedTranslation;
	//int vertNum;

	GLuint vbo{ 0 }, vao{ 0 }, ibo{ 0 };
	GLuint bbVBO{ 0 }, bbVAO{ 0 }, bbIBO{ 0 };

	std::vector<int> snapOrientation;
	glm::mat4 snapTransform = glm::mat4(1.0f);
	glm::mat4 originTransform = glm::mat4(1.0f);
	glm::mat4 xRotation = glm::mat4(1.0f);
	glm::mat4 yRotation = glm::mat4(1.0f);
	glm::mat4 zRotation = glm::mat4(1.0f);
	glm::mat4 scaleMatrix = glm::mat4(1.0f);
	glm::mat4 selectScaleMatrix = glm::mat4(1.0f);
	glm::mat4 trashScaleMatrix = glm::mat4(1.0f);
	glm::vec3 selectTranslation;
	glm::mat4 cursorTranslation = glm::mat4(1.0f);

	float angleX = 0;
	float angleY = 0;
	float angleZ = 0;
	float scaleFactor = 1.0f;

	bool isLoaded = false;
	bool previewSelection = false;
	bool isSelected = false;
	bool colorSelection = false;
	bool isWall = false;
	//just temporary to create multiple visible meshs from the same file
	glm::vec3 translation;

	glm::vec3 centerPoint;
	glm::vec3 upperBounds;
	glm::vec3 lowerBounds;

	glm::vec3 originalUpperBounds;
	glm::vec3 originalLowerBounds;

	int colorCode;
};



