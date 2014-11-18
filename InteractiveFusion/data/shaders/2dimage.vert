#version 330

layout (location = 0) in vec4 inPosition;
layout (location = 1) in vec4 color;
layout (location = 2) in vec3 inNormal;

smooth out vec4 theColor;
smooth out vec3 theNormal;

uniform struct Matrices
{
	mat4 projectionMatrix;
	mat4 modelMatrix;
	mat4 viewMatrix;
	mat4 normalMatrix;
} matrices;

void main()
{
	vec4 vEyeSpacePosVertex = matrices.viewMatrix*matrices.modelMatrix*inPosition;
	gl_Position = matrices.projectionMatrix*vEyeSpacePosVertex;
	theColor = color;
	theNormal = inNormal;
}