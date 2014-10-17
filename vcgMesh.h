#pragma once
#include<vcg/complex/complex.h>

class VCGFace;
class VCGVertex;
class VCGEdge;

struct VCGUsedTypes : public vcg::UsedTypes<	vcg::Use<VCGVertex>::AsVertexType, vcg::Use<VCGEdge>::AsEdgeType, vcg::Use<VCGFace>::AsFaceType>{};

class VCGVertex : public vcg::Vertex< VCGUsedTypes, vcg::vertex::VFAdj, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags  >{};
class VCGEdge : public vcg::Edge<   VCGUsedTypes> {};
class VCGFace : public vcg::Face<   VCGUsedTypes, vcg::face::VFAdj, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags > {};
class VCGMesh : public vcg::tri::TriMesh< std::vector<VCGVertex>, std::vector<VCGFace>, std::vector<VCGEdge> > {};

class VCGMeshContainer
{
public:
	VCGMeshContainer();
	~VCGMeshContainer();

	void LoadMesh(const char* filename);
	void GenerateVAO();
	void GenerateBOs();

	void CleanMesh();
	void ParseData();
	void ConvertToVCG();
	void ParseData(std::vector<float> inputVertices, std::vector<GLuint> inputIndices);

	void Draw();
	void DrawBB();

	void AttachToCursor(glm::vec3 nearPoint, glm::vec3 farPoint);
	bool CheckCollision(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output);
	void TranslateVerticesToPoint(glm::vec3 point);

	//just temporary to create multiple visible meshs from the same file
	void SetTranslation(glm::vec3 trans);

	void SetSelected(bool val);
	void SetColorCode(int value);
	void SetAngleX(bool positive);
	void SetAngleY(bool positive);
	void SetAngleZ(bool positive);
	void SetScale(bool positive);
	glm::vec3 GetCenterPoint();
	int RemoveSmallComponents(int compSize);
	int FillHoles(int holeSize);
	void ResetSelectedTransformation();
	void CleanAndParse(std::vector<float> &startingVertices, std::vector<GLuint> &startingIndices, std::vector<float> &startingNormals);
	std::vector<float> GetVertices();
	std::vector<GLuint> GetIndices();
	float GetLowestZ();
	int GetNumberOfVertices();
	int GetNumberOfIndices();
	void ConvertToVCG(std::vector<float> inputVertices, std::vector<GLuint> inputIndices);
	void ToggleSelectedColor(bool flag);
	int GetColorCode();
	glm::vec3 GetUpperBounds();
	glm::vec3 GetLowerBounds();
	
	void ClearMesh();
private:
	VCGMesh currentMesh;

	glm::vec3 FindClosestPoint(glm::vec3 inputPoint);
	std::vector<float> bBoxVertices;
	std::vector<GLuint> bBoxIndices;
	std::vector<float> vertices;
	std::vector<GLuint> indices;
	std::vector<float> storedColors;

	int vertNum;

	GLuint vbo;
	GLuint bbVBO;
	GLuint bbIBO;
	GLuint bbVAO;
	GLuint ibo;
	GLuint vao;

	glm::mat4 originTransform = glm::mat4(1.0f);
	glm::mat4 xRotation = glm::mat4(1.0f);
	glm::mat4 yRotation = glm::mat4(1.0f);
	glm::mat4 zRotation = glm::mat4(1.0f);
	glm::mat4 scaleMatrix = glm::mat4(1.0f);
	glm::vec3 selectTranslation;
	glm::mat4 cursorTranslation = glm::mat4(1.0f);

	float angleX = 0;
	float angleY = 0;
	float angleZ = 0;
	float scaleFactor = 1.0f;

	bool isSelected = false;
	bool colorSelection = false;
	//just temporary to create multiple visible meshs from the same file
	glm::vec3 translation;

	glm::vec3 centerPoint;
	glm::vec3 upperBounds;
	glm::vec3 lowerBounds;
	
	int colorCode;
};

extern std::vector<VCGMeshContainer*> meshData;
extern int numberOfVertices;
extern int numberOfFaces;

void CombineAndExport();
void CleanAndParse(const char* fileName, std::vector<float> &startingVertices, std::vector<GLuint> &startingIndices, std::vector<float> &startingNormals);