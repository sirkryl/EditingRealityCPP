#include "OpenGLShader.h"

#include <tchar.h>
#include <windows.h>

using namespace std;

namespace InteractiveFusion {
	OpenGLShader::OpenGLShader()
	{
	}

	OpenGLShader::OpenGLShader(const OpenGLShader& _shader)
	{
		shaderId = _shader.shaderId;
		shaderType = _shader.shaderType;
	}

	bool OpenGLShader::LoadShader(string _fileName, int _shaderType)
	{
		vector<string> lines = GetLinesFromFile(_fileName);

		if (lines.size() == 0)
			return false;

		const char** programString = new const char*[lines.size()];
		for (int i = 0; i < lines.size(); i++)
		{
			programString[i] = lines[i].c_str();
		}

		shaderId = glCreateShader(_shaderType);

		glShaderSource(shaderId, (int)lines.size(), programString, NULL);

		glCompileShader(shaderId);

		delete[] programString;

		int status;
		glGetShaderiv(shaderId, GL_COMPILE_STATUS, &status);

		if (status == GL_FALSE)
		{
			char infoLog[1024];
			_TCHAR finalMessage[1536];
			int logLength;
			glGetShaderInfoLog(shaderId, 1024, &logLength, infoLog);
			swprintf(finalMessage, L"Error! Shader file %s wasn't compiled! The compiler returned:\n\n%s", _fileName.c_str(), infoLog);
			MessageBox(NULL, finalMessage, _T("Error"), MB_ICONERROR);
			return false;
		}

		shaderType = _shaderType;

		return true;
	}

	vector<string> OpenGLShader::GetLinesFromFile(string _fileName)
	{
		vector<string> outputLines;

		FILE* fileParser = fopen(_fileName.c_str(), "rt");
		if (!fileParser)
			return outputLines;

		string fileDirectory;
		int slashIndex = -1;
		for (int i = (int)_fileName.size() - 1; i >= 0; i--)
		{
			if (_fileName[i] == '\\' || _fileName[i] == '/')
			{
				slashIndex = i;
				break;
			}
		}

		fileDirectory = _fileName.substr(0, slashIndex + 1);

		char lineString[255];

		
		while (fgets(lineString, 255, fileParser))
		{
			outputLines.push_back(lineString);
		}

		fclose(fileParser);

		return outputLines;
	}

	bool OpenGLShader::IsCompiled()
	{
		int status;
		glGetShaderiv(shaderId, GL_COMPILE_STATUS, &status);
		return status == GL_TRUE;
	}

	GLuint OpenGLShader::GetShaderID()
	{
		return shaderId;
	}

	void OpenGLShader::DeleteShader()
	{
		if (!IsCompiled())
			return;

		glDeleteShader(shaderId);
	}

	
}