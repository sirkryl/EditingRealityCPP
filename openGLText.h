#pragma once

#include <ft2build.h>
#include FT_FREETYPE_H

class OpenGLText
{
public:
	void PrepareForRender();
	void RenderText(const std::wstring &str, int pixelSize, float x, float y, float sx, float sy);
	void RenderText(const std::wstring &str, int data, int pixelSize, float x, float y, float sx, float sy);
	void Initialize(string fontname);
	void CleanUp();
private:
	GLuint fontTexture{ 0 }, fontSampler{ 0 };
	GLuint fontVBO{ 0 }, fontVAO{ 0 };

	FT_Library ft_lib{ nullptr };
	FT_Face face{ nullptr };
};