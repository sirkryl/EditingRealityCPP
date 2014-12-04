#include "OpenGLButton.h"
#include "OpenGLShaders.h"
#include <glm/gtc/matrix_transform.hpp>
#include "colorCoding.h"
#include "OpenGLText.h"
#include "InteractiveFusion.h"
OpenGLButton::OpenGLButton(int _colorCode, float _x, float _y, float _w, float _h, ColorIF _defaultColor, ColorIF _pressedColor, wstring _text)
{
	colorCode = _colorCode;
	x = _x;
	y = _y;
	w = _w;
	h = _h;
	defaultColor = _defaultColor;
	pressedColor = _pressedColor;

	Vertex v1 = { x, y, 0.0f};
	
	Vertex v2 = { x + w, y, 0.0f};
	Vertex v3 = { x + w, y + h, 0.0f };
	Vertex v4 = { x, y + h, 0.0f };
	

	buttonVertices.push_back(v1);
	buttonVertices.push_back(v3);
	buttonVertices.push_back(v4);
	buttonVertices.push_back(v1);
	buttonVertices.push_back(v2);
	buttonVertices.push_back(v3);


	Vertex b1 = { x - lineWidth/2, y - lineWidth, 0.0f };
	Vertex b2 = { x + w + lineWidth/2, y - lineWidth, 0.0f };
	Vertex b3 = { x + w + lineWidth/2, y + h + lineWidth, 0.0f };
	Vertex b4 = { x - lineWidth/2, y + h + lineWidth, 0.0f };
	borderVertices.push_back(b1);
	borderVertices.push_back(b3);
	borderVertices.push_back(b4);
	borderVertices.push_back(b1);
	borderVertices.push_back(b2);
	borderVertices.push_back(b3);
	text = _text;
}

OpenGLButton::~OpenGLButton()
{
	buttonVertices.clear();
}

void OpenGLButton::GenerateBuffers()
{
	glGenBuffers(1, &buttonVBO);

	glBindBuffer(GL_ARRAY_BUFFER, buttonVBO);
	glBufferData(GL_ARRAY_BUFFER, buttonVertices.size() * sizeof(Vertex), &buttonVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &buttonVAO);
	glBindVertexArray(buttonVAO);

	glBindBuffer(GL_ARRAY_BUFFER, buttonVBO);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));

	glGenBuffers(1, &borderVBO);

	glBindBuffer(GL_ARRAY_BUFFER, borderVBO);
	glBufferData(GL_ARRAY_BUFFER, borderVertices.size() * sizeof(Vertex), &borderVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &borderVAO);
	glBindVertexArray(borderVAO);

	glBindBuffer(GL_ARRAY_BUFFER, borderVBO);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
}

bool OpenGLButton::IsMouseInHandle()
{

}

void OpenGLButton::Draw()
{
	float hRatio = ((float)openGLWin.glControl.GetViewportHeight() + (float)openGLWin.glControl.GetOffSetBottom()) / ((float)openGLWin.glControl.GetViewportWidth() + (float)openGLWin.glControl.GetOffSetRight());
	hRatio =  hRatio * 1.6f;
	glm::mat4 twoDScaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(hRatio, 1.0f, 1.0f));
	glm::mat4 modelMatrix;
	//glEnable(GL_POLYGON_OFFSET_FILL);
	//glPolygonOffset(1.0, 1.0);
	shaderButton.UseProgram();
	modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(-0.00f, -0.00f, 0.0f));
	shaderButton.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderButton.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));
	shaderButton.SetUniform("matrices.modelMatrix", modelMatrix);
	shaderButton.SetUniform("vertexColor", glm::vec4(0.5f, 0.5f, 0.5f, 0.8f));

	glBindVertexArray(borderVAO);
	glDrawArrays(GL_TRIANGLES, 0, borderVertices.size());
	glBindVertexArray(0);
	glUseProgram(0);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glDisable(GL_POLYGON_OFFSET_FILL);
	
	shaderButton.UseProgram();
	if (isClicked)
		shaderButton.SetUniform("vertexColor", glm::vec4(pressedColor.r, pressedColor.g, pressedColor.b, pressedColor.a));
	else
		shaderButton.SetUniform("vertexColor", glm::vec4(defaultColor.r, defaultColor.g, defaultColor.b, defaultColor.a));
	
	shaderButton.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderButton.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));

	modelMatrix =glm::translate(glm::mat4(1.0f), glm::vec3(-0.00f, -0.00f, 0.0f));

	shaderButton.SetUniform("matrices.modelMatrix", modelMatrix);

	//if (isOverTrash)
	//glDisable(GL_DEPTH_TEST);
	glDisable(GL_DEPTH_TEST);
	glBindVertexArray(buttonVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	glUseProgram(0);
	

	if (text.size() > 0)
	{
		int storedWidth = openGLWin.glControl.GetViewportWidth();
		int storedHeight = openGLWin.glControl.GetViewportHeight();
		glText.PrepareForRender();
		float textWidth = 87.0f * (2.0f / storedWidth);
		glText.RenderText(text, 25 / hRatio, ((x + (x + w)) / 2.0f - textWidth/2), ((y + (y + h)) / 2.0f - 0.02f), 2.0f / storedWidth, 2.0f / storedHeight);
		//(x + text.length() * 0.008f)
	}
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_DEPTH_TEST);
	
	/*glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBindVertexArray(buttonVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	glUseProgram(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);*/

	/*glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glBindVertexArray(buttonVAO);
	glLineWidth(0.5f);
	glDrawArrays(GL_LINES, 0, borderVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);
	glDisable(GL_LINE_SMOOTH);*/

}
void OpenGLButton::DrawBB()
{
	shaderButton.UseProgram();
	shaderButton.SetUniform("vertexColor", colorCoding::IntToColor(colorCode));
	glm::mat4 modelMatrix;
	shaderButton.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
	shaderButton.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));

	modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f));

	shaderButton.SetUniform("matrices.modelMatrix", modelMatrix);

	//if (isOverTrash)
	//glDisable(GL_DEPTH_TEST);
	glBindVertexArray(buttonVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	glUseProgram(0);
}

void OpenGLButton::SetClicked(bool flag)
{
	isClicked = flag;
}

void OpenGLButton::SetColorCode(int code)
{
	colorCode = code;
}

int OpenGLButton::GetColorCode()
{
	return colorCode;
}
