#version 330

smooth in vec4 theColor;

uniform vec4 pickColor;
uniform bool colorPicking = false;
uniform bool highlight = false;
uniform float alpha;
out vec4 outputColor;

void main()
{
	if (colorPicking)
		outputColor = pickColor;
	else if (highlight)
	{
		outputColor = theColor + vec4(0.1,0.0,0.0,0.0);
	}
	else
	{
		outputColor = theColor;
		outputColor.w = alpha;
	}
}
