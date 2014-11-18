#include "colorCoding.h"
#include "Mesh2D.h"
#include "OpenGLShaders.h"
#include "InteractiveFusion.h"
#include "OpenGLCamera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/clustering.h>
#include<wrap/io_trimesh/import.h>
#include<wrap/io_trimesh/export.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/complex/algorithms/update/normal.h>

Mesh2D::Mesh2D() {
}

Mesh2D::~Mesh2D() { }

void Mesh2D::SetColorCode(int value)
{
	colorCode = value;
}

int Mesh2D::GetColorCode()
{
	return colorCode;
}

void Mesh2D::Load2dMesh(const char* filename)
{
	vcg::tri::io::ImporterPLY<VCGMesh>::Open(currentMesh, filename);
}

void Mesh2D::CleanMesh()
{
	//vcg::tri::RequirePerVertexNormal(currentMesh);
	//vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(currentMesh);

}

void Mesh2D::ParseData()
{
	vertices.clear();
	indices.clear();

	
	std::clock_t start;
	double duration;
	glm::vec4 color = colorCoding::IntToColor(colorCode);
	start = std::clock();
	VCGMesh::VertexIterator vi;
	std::vector<int> VertexId((currentMesh).vert.size());
	//std::vector<float> colors;
	int numvert = 0;
	int curNormalIndex = 1;
	for (vi = (currentMesh).vert.begin(); vi != (currentMesh).vert.end(); ++vi) if (!(*vi).IsD())
	{
		VertexId[vi - (currentMesh).vert.begin()] = numvert;
		int dim = 0;
		Vertex vertex;
		vertex.x = (*vi).P()[0];
		vertex.y = (*vi).P()[1];
		vertex.z = (*vi).P()[2];
		vertex.normal_x = (*vi).N()[0];
		vertex.normal_y = (*vi).N()[1];
		vertex.normal_z = (*vi).N()[2];
		vertex.r = (*vi).C()[0] / 255.0f;
		vertex.g = (*vi).C()[1] / 255.0f;
		vertex.b = (*vi).C()[2] / 255.0f;
		//vertex.r = 0.5f;
		//vertex.g = 0.5f;
		//vertex.b = 0.5f;
		vertices.push_back(vertex);

		Vertex bBoxVertex;
		bBoxVertex.x = vertex.x;
		bBoxVertex.y = vertex.y;
		bBoxVertex.z = vertex.z;
		bBoxVertex.normal_x = vertex.normal_x;
		bBoxVertex.normal_y = vertex.normal_y;
		bBoxVertex.normal_z = vertex.normal_z;
		bBoxVertex.r = color.r;
		bBoxVertex.g = color.g;
		bBoxVertex.b = color.b;
		bBoxVertices.push_back(bBoxVertex);

		numvert++;
	}

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		Triangle triangle;
		triangle.v1 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(0))];
		triangle.v2 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(1))];
		triangle.v3 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(2))];

		indices.push_back(triangle);
		bBoxIndices.push_back(triangle);
	}



	//bbox



	//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	//cDebug::DbgOut(L"parse duration: ", duration);
}


void Mesh2D::GenerateVAO()
{
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 3));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 6));

	glGenVertexArrays(1, &bbVAO);
	glBindVertexArray(bbVAO);
	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 3));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 6));
}

void Mesh2D::GenerateBO()
{
	glGenBuffers(1, &vbo);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &ibo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(Triangle), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenBuffers(1, &bbVBO);

	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glBufferData(GL_ARRAY_BUFFER, bBoxVertices.size() * sizeof(Vertex), &bBoxVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &bbIBO);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bbIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, bBoxIndices.size() * sizeof(Triangle), &bBoxIndices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	
}

void Mesh2D::Draw()
{

	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));

	float hRatio = (float)openGLWin.glControl.GetViewportHeight() / (float)openGLWin.glControl.GetViewportWidth();
	scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(hRatio, 1.0f, 1.0f));

	glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.95f - hRatio*0.15f, -0.65f, 0.0f)) * scaleMatrix;
	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, (GLvoid*)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void Mesh2D::DrawBB()
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));

	float hRatio = (float)openGLWin.glControl.GetViewportHeight() / (float)openGLWin.glControl.GetViewportWidth();
	scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(hRatio, 1.0f, 1.0f));

	glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.95f - hRatio*0.15f, -0.65f, 0.0f)) * scaleMatrix;
	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(bbVAO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bbIBO);
	glDrawElements(GL_TRIANGLES, bBoxIndices.size() * 3, GL_UNSIGNED_INT, (GLvoid*)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}


void Mesh2D::ClearMesh()
{
	currentMesh.Clear();
	glDeleteBuffers(1, &vbo);
	glDeleteBuffers(1, &ibo);
	glDeleteVertexArrays(1, &vao);
	vertices.clear();
	indices.clear();
}

void Mesh2D::SetScale(bool positive)
{
	if (positive)
		scaleFactor += 0.1f;
	else
		scaleFactor -= 0.1f;

	scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(scaleFactor, scaleFactor, scaleFactor));
}

void Mesh2D::SetAngleX(bool positive)
{
	if (positive)
		angleX += 20;
	else
		angleX -= 20;

	xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
}

void Mesh2D::SetAngleY(bool positive)
{
	if (positive)
		angleY += 20;
	else
		angleY -= 20;

	yRotation = glm::rotate(glm::mat4(1.0), angleY, glm::vec3(0.0f, 1.0f, 0.0f));
}

void Mesh2D::SetAngleZ(bool positive)
{
	if (positive)
		angleZ += 20;
	else
		angleZ -= 20;

	zRotation = glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f));
}

void Mesh2D::SetTranslation(glm::vec3 trans)
{
	translation = trans;
}

void Mesh2D::SetSelected(bool val)
{
	isSelected = val;
}

void Mesh2D::UpdateBuffers()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(Triangle), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


std::vector<Vertex> Mesh2D::GetVertices()
{
	return vertices;
}

std::vector<Triangle> Mesh2D::GetIndices()
{
	return indices;
}

int Mesh2D::GetNumberOfVertices()
{
	return vertices.size();
}

int Mesh2D::GetNumberOfTriangles()
{
	return indices.size();
}
