#include "Renderable.h"
#include <glm/gtc/matrix_transform.hpp>
#include "DebugUtility.h"
namespace InteractiveFusion {
	Renderable::Renderable()
	{
	}


	Renderable::~Renderable()
	{
	}

	Renderable::Renderable(std::vector<Vertex> _vertices)
	{
		vertices = _vertices;
	}

	Renderable::Renderable(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles)
	{
		DebugUtility::DbgOut(L"Renderable::Renderable()::_vertices size ", (int)_vertices.size());
		vertices = _vertices;
		DebugUtility::DbgOut(L"Renderable::Renderable()::vertices size ", (int)vertices.size());
		triangles = _triangles;
	}

	void Renderable::SetColorCode(int _colorCode)
	{
		colorCode = _colorCode;
	}

	void Renderable::SetShaderProgram(OpenGLShaderProgram _shaderProgram)
	{
		meshShaderProgram = _shaderProgram;
	}

	int Renderable::GetColorCode()
	{
		return colorCode;
	}

	void Renderable::UpdateEssentials()
	{
		UpdateBounds();
	}

	void Renderable::UpdateBounds()
	{
		lowerBounds.x = 99999.0f;
		lowerBounds.y = 99999.0f;
		lowerBounds.z = 99999.0f;
		upperBounds.x = -99999.0f;
		upperBounds.y = -99999.0f;
		upperBounds.z = -99999.0f;

		for (auto& vertex : vertices)
		{
			lowerBounds.x = glm::min(lowerBounds.x, vertex.x);
			lowerBounds.y = glm::min(lowerBounds.y, vertex.y);
			lowerBounds.z = glm::min(lowerBounds.z, vertex.z);
			upperBounds.x = glm::max(upperBounds.x, vertex.x);
			upperBounds.y = glm::max(upperBounds.y, vertex.y);
			upperBounds.z = glm::max(upperBounds.z, vertex.z);
		}

		centerPoint.x = (lowerBounds.x + upperBounds.x) / 2.0f;
		centerPoint.y = (lowerBounds.y + upperBounds.y) / 2.0f;
		centerPoint.z = (lowerBounds.z + upperBounds.z) / 2.0f;
	}

	void Renderable::GenerateBuffers()
	{
		GenerateBufferObjects();
		GenerateVertexArrayObject();
	}

	void Renderable::GenerateBufferObjects()
	{
		DebugUtility::DbgOut(L"Renderable::GenerateBufferObjects()::Begin");
		GenerateVertexBuffer();
		DebugUtility::DbgOut(L"Renderable::GenerateBufferObjects()::22");
		if (triangles.size() > 2)
			GenerateIndexBuffer();
	}

	void Renderable::GenerateIndexBuffer()
	{
		DebugUtility::DbgOut(L"Renderable::GenerateIndexBuffer()::Begin");
		if (!glIsBuffer(ibo))
			glGenBuffers(1, &ibo);
		UpdateIndexBuffer();
	}

	void Renderable::GenerateVertexBuffer()
	{
		DebugUtility::DbgOut(L"Renderable::GenerateVertexBuffer()::Begin");
		if (!glIsBuffer(vbo))
			glGenBuffers(1, &vbo);
		UpdateVertexBuffer();
	}

	void Renderable::GenerateVertexArrayObject()
	{
		DebugUtility::DbgOut(L"Renderable::GenerateVertexArrayObject()::Begin");
		if (!glIsBuffer(vao))
			glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		if (triangles.size() > 2)
			glEnableVertexAttribArray(2);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 3));

		if (triangles.size() > 2)
			glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float) * 6));
	}

	void Renderable::UpdateVertexBuffer()
	{
		DebugUtility::DbgOut(L"Renderable::UpdateVertexBuffer()::Begin, vertices size: ", (int)vertices.size());
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	void Renderable::UpdateIndexBuffer()
	{
		DebugUtility::DbgOut(L"Renderable::UpdateIndexBuffer()::Begin");
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.size() * sizeof(Triangle), &triangles[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	bool Renderable::AreBuffersInitialized()
	{
		return glIsBuffer(vbo);
	}

	void Renderable::ClearBuffers()
	{
		if (glIsBuffer(vbo))
			glDeleteBuffers(1, &vbo);
		if (glIsBuffer(ibo))
			glDeleteBuffers(1, &ibo);
		if (glIsBuffer(vao))
			glDeleteVertexArrays(1, &vao);
	}

	glm::vec3 Renderable::GetLowerBounds()
	{
		return lowerBounds;
	}

	int Renderable::GetNumberOfTriangles()
	{
		return triangles.size();
	}

	int Renderable::GetNumberOfVertices()
	{
		return vertices.size();
	}

	glm::vec3 Renderable::GetUpperBounds()
	{
		return upperBounds;
	}

	glm::vec3 Renderable::GetCenterPoint()
	{
		return centerPoint;
	}

	void Renderable::FinishDrawing()
	{
		if (triangles.size() > 2)
		{
			glBindVertexArray(vao);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
			glDrawElements(GL_TRIANGLES, triangles.size() * 3, GL_UNSIGNED_INT, (GLvoid*)0);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		}
		else
		{
			glBindVertexArray(vao);
			glDrawArrays(GL_TRIANGLES, 0, vertices.size());
		}
		glBindVertexArray(0);
		glUseProgram(0);
	}

	void Renderable::CleanUp()
	{
		ClearBuffers();
		vertices.clear();
		triangles.clear();
	}
}