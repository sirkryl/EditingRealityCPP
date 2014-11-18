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
	void UnUseProgram();
	UINT GetProgramID();

	void SetUniform(string name, const glm::vec4 vector);
	void SetUniform(string name, glm::mat4* mMatrices, int count = 1);
	void SetUniform(string name, const glm::mat4 mMatrix);
	void SetUniform(string name, const int value);
	void SetUniform(string name, const glm::vec3 vector);
private:
	UINT glProgram; 
	bool linked; 
};

bool PrepareShaderPrograms();

extern OpenGLShader shaders[6];
extern OpenGLShaderProgram shaderColor, shaderFont, shader2d;