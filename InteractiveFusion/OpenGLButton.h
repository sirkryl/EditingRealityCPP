#pragma once
#include "common.h"


class OpenGLButton
{
public:
	OpenGLButton(int _colorCode, float _x, float _y, float _w, float _h, ColorIF _defaultColor, ColorIF _pressedColor, wstring _text);
	~OpenGLButton();
	void GenerateBuffers();
	bool IsMouseInHandle();
	void SetColorCode(int code);

	void SetClicked(bool flag);
	int GetColorCode();
	void Draw();
	void DrawBB();
private:
	float x, y;
	float w, h;
	bool isClicked = false;

	float lineWidth = 0.005f;
	wstring text;
	int colorCode;
	ColorIF defaultColor;
	ColorIF pressedColor;
	std::vector<Vertex> buttonVertices;
	std::vector<Vertex> borderVertices;
	GLuint buttonVBO;
	GLuint buttonVAO;
	GLuint borderVBO;
	GLuint borderVAO;
};