#version 330

layout (location = 0) in vec4 inPosition;
layout (location = 1) in vec4 color;

smooth out vec4 theColor;
uniform mat4 transformationMatrix = mat4(1.0);
void main()
{
	gl_Position = transformationMatrix * inPosition;
	theColor = color;
}