#pragma once
#include "OpenGLShader.h"
#include <string>
#include <vector>
#include <gl/glew.h>
#include <glm/glm.hpp>

namespace InteractiveFusion {
	class OpenGLShaderProgram
	{
	public:
		OpenGLShaderProgram();
		OpenGLShaderProgram(const OpenGLShaderProgram& _program);
		void CreateProgram();
		void DeleteProgram();

		bool AddShaderToProgram(OpenGLShader shader);
		bool LinkProgram();
		bool IsLinked();
		void UseProgram();
		void UnUseProgram();
		GLuint GetProgramID();

		void SetUniform(std::string name, const glm::vec4 vector);
		void SetUniform(std::string name, glm::mat4* mMatrices, int count = 1);
		void SetUniform(std::string name, const glm::mat4 mMatrix);
		void SetUniform(std::string name, const int value);
		void SetUniform(std::string name, const glm::vec3 vector);
		void SetUniform(std::string name, const float value);
	private:
		GLuint programId;
		std::vector<OpenGLShader> shaders;

		int GetUniformLocation(std::string _name);
	};
}

