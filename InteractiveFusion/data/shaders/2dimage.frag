#version 330

smooth in vec4 theColor;
smooth in vec3 theNormal;

out vec4 outputColor;

uniform struct SimpleDirectionalLight
{
	vec3 vColor;
	vec3 vDirection;
	float fAmbientIntensity;
} sunLight;

void main()
{
	vec4 objColor = theColor;
	float fDiffuseIntensity = max(0.0, dot(normalize(theNormal), -sunLight.vDirection));
	outputColor = objColor*vec4(sunLight.vColor*(sunLight.fAmbientIntensity+fDiffuseIntensity), 1.0);
}
