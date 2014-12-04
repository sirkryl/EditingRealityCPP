#version 330

layout (location = 0) in vec4 inPosition;

uniform struct Matrices
{
	mat4 projectionMatrix;
	mat4 modelMatrix;
	mat4 viewMatrix;
} matrices;

void main()
{
	vec4 vEyeSpacePosVertex = matrices.viewMatrix*matrices.modelMatrix*inPosition;
	gl_Position = matrices.projectionMatrix*vEyeSpacePosVertex;
}