#include "common.h"
#include "openGLShaders.h"

#include <glm/gtc/type_ptr.hpp>

OpenGLShader::OpenGLShader()
{
	ready = false;
}

OpenGLShader shaders[4];
OpenGLShaderProgram shaderColor, shaderFont;

/*-----------------------------------------------

Name:	PrepareShaderPrograms

Params:	none

Result:	Loads all shaders and creates shader programs.

/*---------------------------------------------*/

bool PrepareShaderPrograms()
{
	// Load shaders and create shader program

	string fileNames[] = { "color.vert", "color.frag", "font.vert", "font.frag"
	};

	for(int i = 0; i < 4; i++)
	{
		string sExt = fileNames[i].substr((int)fileNames[i].size() - 4, 4);
		int shaderType;
		if (sExt == "vert")
			shaderType = GL_VERTEX_SHADER;
		else if (sExt == "frag")
			shaderType = GL_FRAGMENT_SHADER;

		shaders[i].LoadShader("data\\shaders\\" + fileNames[i], shaderType);
	}

	shaderColor.CreateProgram();
	shaderColor.AddShaderToProgram(&shaders[0]);
	shaderColor.AddShaderToProgram(&shaders[1]);
	if (!shaderColor.LinkProgram())
		return false;

	shaderFont.CreateProgram();
	shaderFont.AddShaderToProgram(&shaders[2]);
	shaderFont.AddShaderToProgram(&shaders[3]);
	shaderFont.LinkProgram();
	return true;
}

/*-----------------------------------------------

Name:    LoadShader

Params:  sFile - path to a file
         sType - type of shader (fragment, vertex, geometry)

Result:	Loads and compiles shader.

/*---------------------------------------------*/

bool OpenGLShader::LoadShader(string sFile, int sType)
{
	vector<string> sLines;

	if(!GetLinesFromFile(sFile, false, &sLines))return false;

	const char** sProgram = new const char*[(int)sLines.size()];
	for (int i = 0; i < (int)sLines.size(); i++)sProgram[i] = sLines[i].c_str();
	
	shaderId = glCreateShader(sType);

	glShaderSource(shaderId, (int)sLines.size(), sProgram, NULL);
	glCompileShader(shaderId);

	delete[] sProgram;

	int iCompilationStatus;
	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &iCompilationStatus);

	if(iCompilationStatus == GL_FALSE)
	{
		char sInfoLog[1024];
		_TCHAR sFinalMessage[1536];
		int iLogLength;
		glGetShaderInfoLog(shaderId, 1024, &iLogLength, sInfoLog);
		swprintf(sFinalMessage, L"Error! Shader file %s wasn't compiled! The compiler returned:\n\n%s", sFile.c_str(), sInfoLog);
		MessageBox(NULL, sFinalMessage, _T("Error"), MB_ICONERROR);
		return false;
	}
	shaderType = sType;
	ready = true;

	return true;
}

/*-----------------------------------------------

Name:    GetLinesFromFile

Params:  sFile - path to a file
         bIncludePart - whether to add include part only
         vResult - vector of strings to store result to

Result:  Loads and adds include part.

/*---------------------------------------------*/

bool OpenGLShader::GetLinesFromFile(string sFile, bool bIncludePart, vector<string>* vResult)
{
	FILE* fp = fopen(sFile.c_str(), "rt");
	if(!fp)return false;

	string sDirectory;
	int slashIndex = -1;
	for (int i = (int)sFile.size()-1; i >= 0; i--)
	{
		if(sFile[i] == '\\' || sFile[i] == '/')
		{
			slashIndex = i;
			break;
		}
	}

	sDirectory = sFile.substr(0, slashIndex+1);

	// Get all lines from a file

	char sLine[255];

	while(fgets(sLine, 255, fp))
	{
		vResult->push_back(sLine);
	}
	fclose(fp);

	return true;
}

/*-----------------------------------------------

Name:	IsLoaded

Params:	none

Result:	True if shader was loaded and compiled.

/*---------------------------------------------*/

bool OpenGLShader::IsLoaded()
{
	return ready;
}

/*-----------------------------------------------

Name:	GetShaderID

Params:	none

Result:	Returns ID of a generated shader.

/*---------------------------------------------*/

UINT OpenGLShader::GetShaderID()
{
	return shaderId;
}

/*-----------------------------------------------

Name:	DeleteShader

Params:	none

Result:	Deletes shader and frees memory in GPU.

/*---------------------------------------------*/

void OpenGLShader::DeleteShader()
{
	if(!IsLoaded())return;
	ready = false;
	glDeleteShader(shaderId);
}

OpenGLShaderProgram::OpenGLShaderProgram()
{
	linked = false;
}

/*-----------------------------------------------

Name:	CreateProgram

Params:	none

Result:	Creates a new program.

/*---------------------------------------------*/

void OpenGLShaderProgram::CreateProgram()
{
	glProgram = glCreateProgram();
}

/*-----------------------------------------------

Name:	AddShaderToProgram

Params:	sShader - shader to add

Result:	Adds a shader (like source file) to
		a program, but only compiled one.

/*---------------------------------------------*/

bool OpenGLShaderProgram::AddShaderToProgram(OpenGLShader* shader)
{
	if(!shader->IsLoaded())return false;

	glAttachShader(glProgram, shader->GetShaderID());

	return true;
}

/*-----------------------------------------------

Name:	LinkProgram

Params:	none

Result:	Performs final linkage of OpenGL program.

/*---------------------------------------------*/

bool OpenGLShaderProgram::LinkProgram()
{
	glLinkProgram(glProgram);
	int tmpRes;
	glGetProgramiv(glProgram, GL_LINK_STATUS, &tmpRes);
	linked = (tmpRes == GL_TRUE);
	return linked;
}

/*-----------------------------------------------

Name:	DeleteProgram

Params:	none

Result:	Deletes program and frees memory on GPU.

/*---------------------------------------------*/

void OpenGLShaderProgram::DeleteProgram()
{
	if(!linked) 
		return;

	linked = false;
	glDeleteProgram(glProgram);
}

/*-----------------------------------------------

Name:	UseProgram

Params:	none

Result:	Tells OpenGL to use this program.

/*---------------------------------------------*/

void OpenGLShaderProgram::UseProgram()
{
	if(linked)
		glUseProgram(glProgram);
}

/*-----------------------------------------------

Name:	GetProgramID

Params:	none

Result:	Returns OpenGL generated shader program ID.

/*---------------------------------------------*/

UINT OpenGLShaderProgram::GetProgramID()
{
	return glProgram;
}


//xx
void OpenGLShaderProgram::SetUniform(string name, const glm::vec4 vector)
{
	int iLoc = glGetUniformLocation(glProgram, name.c_str());
	glUniform4fv(iLoc, 1, (GLfloat*)&vector);
}

//xx
void OpenGLShaderProgram::SetUniform(string name, glm::mat4* mMatrices, int count)
{
	int iLoc = glGetUniformLocation(glProgram, name.c_str());
	glUniformMatrix4fv(iLoc, count, FALSE, (GLfloat*)mMatrices);
}

//xx
void OpenGLShaderProgram::SetUniform(string name, const glm::mat4 mMatrix)
{
	int iLoc = glGetUniformLocation(glProgram, name.c_str());
	glUniformMatrix4fv(iLoc, 1, FALSE, (GLfloat*)&mMatrix);
}

void OpenGLShaderProgram::SetUniform(string name, const int value)
{
	int iLoc = glGetUniformLocation(glProgram, name.c_str());
	glUniform1i(iLoc, value);
}