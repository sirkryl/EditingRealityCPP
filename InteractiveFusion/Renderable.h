#pragma once

#include "CommonStructs.h"
#include "OpenGLShaderProgram.h"
#include "RenderingException.h"
#include "BufferException.h"
#include <vector>
namespace InteractiveFusion {
	class Renderable
	{
	public:
		Renderable();
		Renderable(std::vector<Vertex> _vertices);
		Renderable(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles);
		virtual ~Renderable();

		void SetColorCode(int _colorCode);
		int GetColorCode();
		void GenerateBuffers();
		void ClearBuffers();

		bool AreBuffersInitialized();

		virtual void UpdateEssentials();
		
		void SetShaderProgram(OpenGLShaderProgram _shaderProgram);

		int GetNumberOfVertices();
		int GetNumberOfTriangles();

		

		glm::vec3 GetCenterPoint();
		glm::vec3 GetUpperBounds();
		glm::vec3 GetLowerBounds();

		virtual void FinishDrawing();


		void CleanUp();

	protected:
		std::vector<Vertex> vertices;
		std::vector<Triangle> triangles;

		int colorCode = -1;

		OpenGLShaderProgram meshShaderProgram;

		GLuint vbo{ 0 }, vao{ 0 }, ibo{ 0 };
		GLuint bbVBO{ 0 }, bbVAO{ 0 }, bbIBO{ 0 };

		glm::vec3 centerPoint;
		glm::vec3 upperBounds;
		glm::vec3 lowerBounds;

		

		virtual void UpdateBounds();

		virtual void GenerateVertexArrayObject();
		void GenerateIndexBuffer();
		void GenerateVertexBuffer();
		void UpdateVertexBuffer();
		void UpdateIndexBuffer();
		virtual void GenerateBufferObjects();

		//virtual void SetUniforms(bool _highlight, bool _colorPicking, glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);
	};
}
