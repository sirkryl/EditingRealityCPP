#include "OpenGLShaderProgram.h"

using namespace std;

namespace InteractiveFusion {
	OpenGLShaderProgram::OpenGLShaderProgram()
	{
	}

	OpenGLShaderProgram::OpenGLShaderProgram(const OpenGLShaderProgram& _program)
	{
		shaders = _program.shaders;
		programId = _program.programId;
	}

	bool OpenGLShaderProgram::IsLinked()
	{
		int status;
		glGetProgramiv(programId, GL_LINK_STATUS, &status);
		return status == GL_TRUE;
	}

	void OpenGLShaderProgram::CreateProgram()
	{
		programId = glCreateProgram();
	}

	bool OpenGLShaderProgram::AddShaderToProgram(OpenGLShader shader)
	{
		if (!shader.IsCompiled())return false;

		glAttachShader(programId, shader.GetShaderID());

		shaders.push_back(shader);

		return true;
	}

	bool OpenGLShaderProgram::LinkProgram()
	{
		glLinkProgram(programId);
		return IsLinked();
	}

	void OpenGLShaderProgram::DeleteProgram()
	{
		if (!IsLinked())
			return;

		glDeleteProgram(programId);

		for (auto shader : shaders)
			shader.DeleteShader();
		shaders.clear();
	}

	void OpenGLShaderProgram::UseProgram()
	{
		if (IsLinked())
			glUseProgram(programId);
	}

	void OpenGLShaderProgram::UnUseProgram()
	{
		if (IsLinked())
			glUseProgram(0);
	}

	GLuint OpenGLShaderProgram::GetProgramID()
	{
		return programId;
	}


	void OpenGLShaderProgram::SetUniform(string name, const glm::vec4 vector)
	{
		glUniform4fv(GetUniformLocation(name), 1, (GLfloat*)&vector);
	}

	void OpenGLShaderProgram::SetUniform(string name, const glm::vec3 vector)
	{
		glUniform3fv(GetUniformLocation(name), 1, (GLfloat*)&vector);
	}

	void OpenGLShaderProgram::SetUniform(string name, glm::mat4* mMatrices, int count)
	{
		glUniformMatrix4fv(GetUniformLocation(name), count, false, (GLfloat*)mMatrices);
	}

	void OpenGLShaderProgram::SetUniform(string name, const glm::mat4 mMatrix)
	{
		glUniformMatrix4fv(GetUniformLocation(name), 1, false, (GLfloat*)&mMatrix);
	}

	void OpenGLShaderProgram::SetUniform(string name, const int value)
	{
		glUniform1i(GetUniformLocation(name), value);
	}

	void OpenGLShaderProgram::SetUniform(string name, const float value)
	{
		glUniform1f(GetUniformLocation(name), value);
	}

	int OpenGLShaderProgram::GetUniformLocation(string _name)
	{
		return glGetUniformLocation(programId, _name.c_str());
	}
}