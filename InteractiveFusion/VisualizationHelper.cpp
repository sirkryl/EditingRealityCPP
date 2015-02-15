#include "VisualizationHelper.h"
#include "OpenGLCamera.h"
#include "SelectionHelper.h"
#include "OpenGLShaderProgram.h"
#include "OpenGLControl.h"

namespace InteractiveFusion {
	void VisualizationHelper::FillPointsToVisualize()
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
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, reinterpret_cast<void*>(0));
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, reinterpret_cast<void*>(sizeof(float) * 3));
		}
		glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
		glBufferData(GL_ARRAY_BUFFER, pointVertices.size() * sizeof(float), &pointVertices[0], GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindVertexArray(pointVAO);


	}

	void VisualizationHelper::RenderHelpingVisuals()
	{
		//RENDER HELPING POINTS
		FillPointsToVisualize();
		shaderColor.UseProgram();
		shaderColor.SetUniform("matrices.projectionMatrix", glControl.GetProjectionMatrix());
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

	bool VisualizationHelper::IsRayInitialized()
	{
		if (rayVBO == 0)
			return false;
		return true;
	}

	void VisualizationHelper::InitializeRayVisual()
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
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, reinterpret_cast<void*>(0));
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 6, reinterpret_cast<void*>(sizeof(float) * 3));
		}
		glBindBuffer(GL_ARRAY_BUFFER, rayVBO);
		glBufferData(GL_ARRAY_BUFFER, rayVertices.size() * sizeof(float), &rayVertices[0], GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);


	}

	void VisualizationHelper::InitializePlane(glm::vec3 lowerBounds, glm::vec3 upperBounds)
	{
		if (planeVBO == 0)
		{
			Vertex v1 = { 0.0f, upperBounds.y, lowerBounds.z,
				0.8f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f };
			Vertex v2 = { 0.0f, upperBounds.y, upperBounds.z,
				0.8f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f };
			Vertex v3 = { 0.0f, lowerBounds.y, lowerBounds.z,
				0.8f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f };
			Vertex v4 = { 0.0f, lowerBounds.y, upperBounds.z,
				0.8f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f };

			planeTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f));

			planeVertices.push_back(v1);
			planeVertices.push_back(v3);
			planeVertices.push_back(v4);
			planeVertices.push_back(v2);
			planeVertices.push_back(v1);
			planeVertices.push_back(v4);

			glGenBuffers(1, &planeVBO);

			glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
			glBufferData(GL_ARRAY_BUFFER, planeVertices.size() * sizeof(Vertex), &planeVertices[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);

			glGenVertexArrays(1, &planeVAO);
			glBindVertexArray(planeVAO);

			glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
			glEnableVertexAttribArray(0);
			glEnableVertexAttribArray(1);
			glEnableVertexAttribArray(2);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 3));
			glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 6));
		}
	}


	VisualizationMode VisualizationHelper::GetVisualizationMode()
	{
		return visMode;
	}
	void VisualizationHelper::SetVisualizationMode(VisualizationMode _visMode)
	{
		visMode = _visMode;
	}

	bool VisualizationHelper::IsPlaneInitialized()
	{
		return planeVBO != 0;
	}

	void VisualizationHelper::DrawPlane()
	{
		shaderColor.UseProgram();
		shaderColor.SetUniform("colorPicking", false);
		shaderColor.SetUniform("alpha", 0.3f);
		glm::mat4 modelMatrix;
		shaderColor.SetUniform("matrices.projectionMatrix", glControl.GetProjectionMatrix());
		shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
		modelMatrix = glm::translate(glm::mat4(1.0), glm::vec3(0.0f));
		modelMatrix = planeTranslation * modelMatrix;
		shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

		//if (isOverTrash)
		//glDisable(GL_DEPTH_TEST);
		glBindVertexArray(planeVAO);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		glBindVertexArray(0);
		glUseProgram(0);
	}

	void VisualizationHelper::SetPlaneTranslation(glm::vec3 point)
	{
		planeTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(point.x, 0.0f, 0.0f));
	}

	void VisualizationHelper::CleanUp()
	{
		glDeleteBuffers(1, &rayVBO);
		glDeleteBuffers(1, &pointVBO);
		glDeleteVertexArrays(1, &rayVAO);
		glDeleteVertexArrays(1, &pointVAO);
		glDeleteBuffers(1, &planeVBO);
		glDeleteVertexArrays(1, &rayVAO);
	}
}