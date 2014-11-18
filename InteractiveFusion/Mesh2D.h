#pragma once
#include<vcg/complex/complex.h>
#include "common.h"
#include "VcgMeshContainer.h"

class Mesh2D
{
public:
	Mesh2D();
	~Mesh2D();
	void Load2dMesh(const char* filename);
	void LoadMesh(const char* filename);

	void ParseData();


	void GenerateVAO();
	void GenerateBO();


	void UpdateBuffers();

	void CleanMesh();

	void Draw();
	void DrawBB();
	void SetColorCode(int value);
	int GetColorCode();
	void SetSelected(bool val);
	void SetTranslation(glm::vec3 trans);
	void SetAngleX(bool positive);
	void SetAngleY(bool positive);
	void SetAngleZ(bool positive);
	void SetScale(bool positive);

	std::vector<Vertex> GetVertices();
	std::vector<Triangle> GetIndices();
	int GetNumberOfVertices();
	int GetNumberOfTriangles();

	void ClearMesh();
private:
	VCGMesh currentMesh;

	std::vector<Vertex> bBoxVertices;
	std::vector<Triangle> bBoxIndices;

	std::vector<Vertex> vertices;
	std::vector<Triangle> indices;
	std::vector<float> storedColors;
	glm::vec3 centerPoint;

	GLuint vbo{ 0 }, vao{ 0 }, ibo{ 0 };
	GLuint bbVBO{ 0 }, bbVAO{ 0 }, bbIBO{ 0 };

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
	int colorCode;
};



