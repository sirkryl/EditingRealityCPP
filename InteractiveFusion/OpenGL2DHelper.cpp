#include "OpenGL2DHelper.h"
#include "OpenGLShaders.h"
#include "InteractiveFusion.h"
#include <glm/gtc/matrix_transform.hpp>
void OpenGL2DHelper::InitialLoadFromFile(const char* fileName, int colorCode)
{
	shared_ptr<VCGMeshContainer> meshTo(new VCGMeshContainer);
	//meshTo->SetColorCode(100);
	meshTo->Load2DMesh(fileName);
	meshTo->SetColorCode(colorCode);
	meshTo->CleanMesh();
	meshTo->ParseData();
	//mesh->GenerateBOs();
	//mesh->GenerateVAO();
	//cDebug::DbgOut(L"vertices: " + mesh->GetNumberOfVertices());
	meshData2d.push_back(meshTo);
	
}

void OpenGL2DHelper::GenerateBuffers()
{
	cDebug::DbgOut(L"2D Generate Buffers");
	//numberOfVertices = 0;
	//numberOfFaces = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->GenerateBOs();

	}
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->GenerateVAO();
	}
}

void OpenGL2DHelper::InitializeRectangle()
{
	if (rectangleVBO == 0)
	{
		Vertex v1 = { -1.0f, -0.15f, 0.0f,
			openGLWin.bgRed, openGLWin.bgGreen, openGLWin.bgBlue,
			0.0f, 0.0f, 0.0f };
		Vertex v2 = { -1.0f, 0.15f, 0.0f,
			openGLWin.bgRed, openGLWin.bgGreen, openGLWin.bgBlue,
			0.0f, 0.0f, 0.0f };
		Vertex v3 = { 1.0f, -0.15f, 0.0f,
			openGLWin.bgRed, openGLWin.bgGreen, openGLWin.bgBlue,
			0.0f, 0.0f, 0.0f };
		Vertex v4 = { 1.0f, 0.15f, 0.0f,
			openGLWin.bgRed, openGLWin.bgGreen, openGLWin.bgBlue,
			0.0f, 0.0f, 0.0f };
		rectangleVertices.push_back(v1);
		rectangleVertices.push_back(v3);
		rectangleVertices.push_back(v4);
		rectangleVertices.push_back(v2);
		rectangleVertices.push_back(v1);
		rectangleVertices.push_back(v4);

		glGenBuffers(1, &rectangleVBO);

		glBindBuffer(GL_ARRAY_BUFFER, rectangleVBO);
		glBufferData(GL_ARRAY_BUFFER, rectangleVertices.size() * sizeof(Vertex), &rectangleVertices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glGenVertexArrays(1, &rectangleVAO);
		glBindVertexArray(rectangleVAO);

		glBindBuffer(GL_ARRAY_BUFFER, rectangleVBO);
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 3));
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 6));
	}
}

bool OpenGL2DHelper::IsRectangleInitialized()
{
	return rectangleVBO != 0;
}

void OpenGL2DHelper::DrawRectangle()
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("colorPicking", false);
	shaderColor.SetUniform("transparent", true);
	glm::mat4 modelMatrix;
	shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));

	modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f));

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	//if (isOverTrash)
	//glDisable(GL_DEPTH_TEST);
	glBindVertexArray(rectangleVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	glUseProgram(0);
	//if (isOverTrash)
	//glEnable(GL_DEPTH_TEST);
}

void OpenGL2DHelper::DrawRectangle(float y)
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("colorPicking", false);
	shaderColor.SetUniform("transparent", true);
	glm::mat4 modelMatrix;
	shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));

	modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f,y,0.0f));

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	//if (isOverTrash)
	//glDisable(GL_DEPTH_TEST);
	glBindVertexArray(rectangleVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	glUseProgram(0);
	//if (isOverTrash)
	//glEnable(GL_DEPTH_TEST);
}

void OpenGL2DHelper::DrawAll()
{
	if (!isOpen)
		meshData2d[0]->Draw();
	else
		meshData2d[1]->Draw();
	//cDebug::DbgOut(L"2D DrawAll");
	/*for (vector <shared_ptr<Mesh2D>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->Draw();
	}*/
}

void OpenGL2DHelper::DrawAllBB()
{
	//cDebug::DbgOut(L"2D DrawAll");
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->DrawBB();
	}
}

void OpenGL2DHelper::CleanUp()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData2d.clear();
}