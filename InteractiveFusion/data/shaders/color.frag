#version 330

smooth in vec4 theColor;

uniform vec4 pickColor;
uniform bool colorPicking = false;
uniform bool transparent = false;
out vec4 outputColor;

void main()
{
	if (colorPicking)
		outputColor = pickColor;
	else
	{
		outputColor = theColor;
		if(transparent)
			outputColor.w = 0.8;
	}
}
