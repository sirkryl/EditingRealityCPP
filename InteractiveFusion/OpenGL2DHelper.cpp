#include "OpenGL2DHelper.h"
#include "OpenGLShaders.h"
#include "InteractiveFusion.h"
#include <glm/gtc/matrix_transform.hpp>
#include "OpenGLButton.h"

shared_ptr < VCGMeshContainer > trashOpen(new VCGMeshContainer);
shared_ptr < VCGMeshContainer > trashClosed(new VCGMeshContainer);

map<int, shared_ptr<OpenGLButton>> buttons;

void OpenGL2DHelper::InitialLoadFromFile(const char* fileName, const char* fileName2, int colorCode)
{
	trashClosed->Load2DMesh(fileName);
	trashClosed->SetColorCode(colorCode);
	trashClosed->CleanMesh();
	trashClosed->ParseData();
	trashOpen->Load2DMesh(fileName2);
	trashOpen->SetColorCode(colorCode);
	trashOpen->CleanMesh();
	trashOpen->ParseData();
	
}

bool OpenGL2DHelper::SelectButton(int colorCode)
{
	if (buttons.count(colorCode) != 0)
	{
		buttons[colorCode]->SetClicked(true);
		return true;
	}
	return false;
}

void OpenGL2DHelper::UnselectButtons()
{
	map<int, shared_ptr<OpenGLButton>>::iterator iter;

	for (iter = buttons.begin(); iter != buttons.end(); ++iter)
	{
		iter->second->SetClicked(false);
	}
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
	map<int, shared_ptr<OpenGLButton>>::iterator iter;

	for (iter = buttons.begin(); iter != buttons.end(); ++iter)
	{
		iter->second->GenerateBuffers();
	}
	
	trashOpen->GenerateBOs();
	trashOpen->GenerateVAO();
	trashClosed->GenerateBOs();
	trashClosed->GenerateVAO();

	buffersInitialized = true;
}

void OpenGL2DHelper::InitializeButton(int colorCode, float x, float y, float w, float h, ColorIF defaultColor, ColorIF pressedColor, wstring text)
{
	shared_ptr<OpenGLButton> button(new OpenGLButton(colorCode, x, y, w, h, defaultColor, pressedColor, text));

	buttons[colorCode] = button;
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

bool OpenGL2DHelper::AreBuffersGenerated()
{
	return buffersInitialized;
}

void OpenGL2DHelper::DrawButtons()
{
	map<int, shared_ptr<OpenGLButton>>::iterator iter;

	for (iter = buttons.begin(); iter != buttons.end(); ++iter)
	{
		iter->second->Draw();
	}
}

void OpenGL2DHelper::DrawButtonsBB()
{
	map<int, shared_ptr<OpenGLButton>>::iterator iter;

	for (iter = buttons.begin(); iter != buttons.end(); ++iter)
	{
		iter->second->DrawBB();
	}
}

void OpenGL2DHelper::DrawRectangle()
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("colorPicking", false);
	shaderColor.SetUniform("alpha", 0.8f);
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

void OpenGL2DHelper::DrawRectangle(float y, float alpha)
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("colorPicking", false);
	shaderColor.SetUniform("alpha", alpha);
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

void OpenGL2DHelper::DrawRectangle(float y, float alpha, float h)
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("colorPicking", false);
	shaderColor.SetUniform("alpha", alpha);
	glm::mat4 modelMatrix;
	shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));
	float scaleY = h / 0.4f;
	modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, y, 0.0f)) * glm::scale(glm::mat4(1.0f), glm::vec3(1.0f, scaleY, 1.0f));

	
	//modelMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f, scaleY, 1.0f)) * modelMatrix;
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

void OpenGL2DHelper::DrawTrash()
{
	//cDebug::DbgOut(L"DRAWTRASH");
	//glDisable(GL_DEPTH_TEST);
	if (!isTrashOpen)
		trashClosed->Draw();
	else
		trashOpen->Draw();
	//glEnable(GL_DEPTH_TEST);
	//cDebug::DbgOut(L"2D DrawAll");
	/*for (vector <shared_ptr<Mesh2D>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->Draw();
	}*/
}

void OpenGL2DHelper::DrawTrashBB()
{
	trashClosed->DrawBB();
	trashOpen->DrawBB();
}

void OpenGL2DHelper::CleanUp()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData2d.clear();
}