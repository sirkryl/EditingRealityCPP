#pragma once

class OpenGLShader
{
public:
	OpenGLShader();

	bool LoadShader(string sFile, int sType);
	void DeleteShader();

	bool GetLinesFromFile(string sFile, bool bIncludePart, vector<string>* vResult);

	bool IsLoaded();
	UINT GetShaderID();

private:
	UINT shaderId;
	int shaderType; 
	bool ready; 
};

class OpenGLShaderProgram
{
public:
	OpenGLShaderProgram();

	void CreateProgram();
	void DeleteProgram();

	bool AddShaderToProgram(OpenGLShader* shader);
	bool LinkProgram();

	void UseProgram();

	UINT GetProgramID();

	void SetUniform(string name, const glm::vec4 vector);
	void SetUniform(string name, glm::mat4* mMatrices, int count = 1);
	void SetUniform(string name, const glm::mat4 mMatrix);
	void SetUniform(string name, const int value);
private:
	UINT glProgram; 
	bool linked; 
};

bool PrepareShaderPrograms();

extern OpenGLShader shaders[4];
extern OpenGLShaderProgram shaderColor, shaderFont;