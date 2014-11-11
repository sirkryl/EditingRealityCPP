#pragma once
#include<vcg/complex/complex.h>

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

	void LoadMesh(const char* filename);
	void LoadMesh(std::vector<float> vertices, std::vector<GLuint> indices, std::vector<float> normals);
	void GenerateVAO();
	void GenerateBOs();

	void CleanMesh();
	void ParseData();
	void ConvertToVCG();
	void ParseData(std::vector<float> inputVertices, std::vector<GLuint> inputIndices);

	void Draw();
	void DrawBB();

	void AttachToCursor(glm::vec3 nearPoint, glm::vec3 farPoint, int distance);
	bool GetHitPoint(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output, glm::vec3 &outputNormal, bool snapToVertex);
	bool CheckCollision(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output);
	void TranslateVerticesToPoint(glm::vec3 point, std::vector<int> orien);
	void TemporaryTranslateVerticesToPoint(glm::vec3 point);
	//just temporary to create multiple visible meshs from the same file
	void SetTranslation(glm::vec3 trans);
	void UpdateBuffers();
	void SetSelected(bool val);
	void SetColorCode(int value);
	void SetAngleX(bool positive);
	void SetAngleY(bool positive);
	void SetAngleZ(bool positive);
	void SetScale(bool positive);
	int MergeCloseVertices(float threshold);
	void LaplacianSmooth(int step);
	void RemoveNonManifoldFace();
	glm::vec3 GetCenterPoint();
	int RemoveSmallComponents(int compSize);
	int FillHoles(int holeSize);
	void ResetSelectedTransformation();
	void CleanAndParse(std::vector<float> &startingVertices, std::vector<GLuint> &startingIndices, std::vector<float> &startingNormals);
	std::vector<float> GetVertices();
	std::vector<float> GetNormals();
	std::vector<GLuint> GetIndices();
	float GetLowestZ();
	float GetLowestY();
	void SetWall(bool flag);
	void SetPlaneParameters(float x, float y, float z, float d);
	void SetSnapTransform(std::vector<int> orien);
	std::vector<int> GetOrientation();
	bool IsWall();
	int GetNumberOfVertices();
	int GetNumberOfIndices();
	void ConvertToVCG(std::vector<float> inputVertices, std::vector<GLuint> inputIndices);
	void ToggleSelectedColor(bool flag);
	int GetColorCode();
	void TogglePreviewSelection(bool flag);
	glm::vec3 GetUpperBounds();
	glm::vec3 GetLowerBounds();
	
	
	void ClearMesh();
private:
	VCGMesh currentMesh;

	glm::vec3 FindClosestPoint(glm::vec3 inputPoint);
	glm::vec3 offSet;
	glm::vec3 snapPoint;
	std::vector<float> bBoxVertices;
	std::vector<GLuint> bBoxIndices;
	std::vector<float> vertices;
	std::vector<GLuint> indices;
	std::vector<float> normals;
	std::vector<float> storedColors;
	float planeParameters[4];
	std::vector<int> orientation;
	// -1 = no plane, 0 = x, 1 = y, 2 = z
	glm::mat4 storedTranslation;
	int vertNum;

	GLuint vbo;
	GLuint bbVBO;
	GLuint bbIBO;
	GLuint bbVAO;
	GLuint ibo;
	GLuint vao;

	std::vector<int> snapOrientation;
	glm::mat4 snapTransform = glm::mat4(1.0f);
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

extern std::vector<VCGMeshContainer*> meshData;
extern int numberOfVertices;
extern int numberOfFaces;

void CombineAndExport();
void CleanAndParse(const char* fileName, std::vector<float> &startingVertices, std::vector<GLuint> &startingIndices, std::vector<float> &startingNormals);