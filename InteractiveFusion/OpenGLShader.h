#pragma once
#include <string>
#include <vector>
#include <gl/glew.h>
namespace InteractiveFusion {
	class OpenGLShader
	{
	public:
		OpenGLShader();
		OpenGLShader(const OpenGLShader& _shader);
		bool LoadShader(std::string sFile, int sType);
		void DeleteShader();

		bool IsCompiled();
		GLuint GetShaderID();

	private:
		GLuint shaderId;
		int shaderType;

		std::vector<std::string> GetLinesFromFile(std::string sFile);
	};

	
}