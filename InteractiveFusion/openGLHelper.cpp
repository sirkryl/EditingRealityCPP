#include "openGLHelper.h"
#include "openGLCamera.h"
#include "openGLSelector.h"
#include "openGLShaders.h"
#include "openGLWin.h"
void OpenGLHelper::FillPointsToVisualize()
{
	pointVertices.clear();
	pointVertices.push_back(glCamera.GetPosition().x);
	pointVertices.push_back(glCamera.GetPosition().y);
	pointVertices.push_back(glCamera.GetPosition().z);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(1.0f);
	pointVertices.push_back(glSelector.nearPoint.x);
	pointVertices.push_back(glSelector.nearPoint.y);
	pointVertices.push_back(glSelector.nearPoint.z);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(1.0f);
	if (glSelector.hitPoint.x != -1)
	{
		pointVertices.push_back(glSelector.hitPoint.x);
		pointVertices.push_back(glSelector.hitPoint.y);
		pointVertices.push_back(glSelector.hitPoint.z);
		pointVertices.push_back(0.0f);
		pointVertices.push_back(0.0f);
		pointVertices.push_back(1.0f);
	}

	if (pointVBO == 0)
	{
		glGenBuffers(1, &pointVBO);
		glGenVertexArrays(1, &pointVAO);

		glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));
	}
	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	glBufferData(GL_ARRAY_BUFFER, pointVertices.size() * sizeof(float), &pointVertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(pointVAO);


}

void OpenGLHelper::RenderHelpingVisuals()
{
	//RENDER HELPING POINTS
	FillPointsToVisualize();
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix = glm::mat4(1.0);

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(pointVAO);
	glPointSize(10.0f);
	glDrawArrays(GL_POINTS, 0, pointVertices.size());
	glBindVertexArray(0);

	//RENDER RAY
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glBindVertexArray(rayVAO);
	glLineWidth(10.0f);
	glDrawArrays(GL_LINES, 0, rayVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);
	glDisable(GL_LINE_SMOOTH);
}

void OpenGLHelper::InitializeRayVisual()
{
	rayVertices.clear();
	rayVertices.push_back(glSelector.nearPoint.x);
	rayVertices.push_back(glSelector.nearPoint.y);
	rayVertices.push_back(glSelector.nearPoint.z);
	rayVertices.push_back(1.0f);
	rayVertices.push_back(0.0f);
	rayVertices.push_back(0.0f);
	if (rayVBO == 0)
	{
		glGenBuffers(1, &rayVBO);
		glGenVertexArrays(1, &rayVAO);

		glBindVertexArray(rayVAO);

		glBindBuffer(GL_ARRAY_BUFFER, rayVBO);
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));
	}
	glBindBuffer(GL_ARRAY_BUFFER, rayVBO);
	glBufferData(GL_ARRAY_BUFFER, rayVertices.size() * sizeof(float), &rayVertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


}

void OpenGLHelper::CleanUp()
{
	glDeleteBuffers(1, &rayVBO);
	glDeleteBuffers(1, &pointVBO);
	glDeleteVertexArrays(1, &rayVAO);
	glDeleteVertexArrays(1, &pointVAO);
}