#version 330

smooth in vec4 theColor;

uniform vec4 pickColor;
uniform bool colorPicking = false;
uniform float alpha;
out vec4 outputColor;

void main()
{
	if (colorPicking)
		outputColor = pickColor;
	else
	{
		outputColor = theColor;
		outputColor.w = alpha;
	}
}
