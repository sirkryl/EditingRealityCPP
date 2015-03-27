#include "Renderable.h"
#include <glm/gtc/matrix_transform.hpp>
#include "DebugUtility.h"
#include <exception>
#include <sstream>
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
		for(auto v : _vertices)
		{
			vertices.push_back(v);
		}
		for (auto t : _triangles)
		{
			triangles.push_back(t);
		}
		/*DebugUtility::DbgOut(L"Renderable::Renderable()::_vertices size ", (int)_vertices.size());
		vertices = _vertices;
		DebugUtility::DbgOut(L"Renderable::Renderable()::vertices size ", (int)vertices.size());
		triangles = _triangles;*/
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
			lowerBounds.x = (glm::min)(lowerBounds.x, vertex.x);
			lowerBounds.y = (glm::min)(lowerBounds.y, vertex.y);
			lowerBounds.z = (glm::min)(lowerBounds.z, vertex.z);
			upperBounds.x = (glm::max)(upperBounds.x, vertex.x);
			upperBounds.y = (glm::max)(upperBounds.y, vertex.y);
			upperBounds.z = (glm::max)(upperBounds.z, vertex.z);
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
		GenerateVertexBuffer();
		if (triangles.size() > 2)
			GenerateIndexBuffer();
	}

	void Renderable::GenerateIndexBuffer()
	{
		try
		{
			if (!glIsBuffer(ibo))
				glGenBuffers(1, &ibo);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while generating index buffer in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
		UpdateIndexBuffer();
	}

	void Renderable::GenerateVertexBuffer()
	{
		try
		{
			if (!glIsBuffer(vbo))
				glGenBuffers(1, &vbo);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while generating vertex buffer in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
		UpdateVertexBuffer();
	}

	void Renderable::GenerateVertexArrayObject()
	{
		try
		{
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
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while generating vertex array buffer in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
	}

	void Renderable::UpdateVertexBuffer()
	{
		try
		{
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while updating vertex buffer in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
	}

	void Renderable::UpdateIndexBuffer()
	{
		try
		{
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.size() * sizeof(Triangle), &triangles[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while updating index buffer in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
	}

	bool Renderable::AreBuffersInitialized()
	{
		try
		{
			return glIsBuffer(vbo);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while checking if buffer is initialized in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
	}

	void Renderable::ClearBuffers()
	{
		try
		{
			if (glIsBuffer(vbo))
				glDeleteBuffers(1, &vbo);
			if (glIsBuffer(ibo))
				glDeleteBuffers(1, &ibo);
			if (glIsBuffer(vao))
				glDeleteVertexArrays(1, &vao);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while clearing buffers in renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new BufferException(ss.str().c_str());
		}
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
		try
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
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while finishing renderable drawing with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new RenderingException(ss.str().c_str());
		}
	}

	void Renderable::CleanUp()
	{
		ClearBuffers();
		vertices.clear();
		triangles.clear();
	}
}