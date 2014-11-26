#include "common.h"
#include "OpenGLText.h"
#include "InteractiveFusion.h"
#include "OpenGLShaders.h"


void OpenGLText::Initialize(string fontname)
{
	if (FT_Init_FreeType(&ft_lib) != 0) {
		MessageBox(NULL, _T("Couldn't initialize FreeType library"), _T("Error"), MB_ICONERROR);
	}

	if (FT_New_Face(ft_lib, fontname.c_str(), 0, &face) != 0) {
		MessageBox(NULL, _T("Unable to load myfont.ttf"), _T("Error"), MB_ICONERROR);
	}

	glGenBuffers(1, &fontVBO);
	glGenVertexArrays(1, &fontVAO);
	glGenTextures(1, &fontTexture);
	glGenSamplers(1, &fontSampler);
	glSamplerParameteri(fontSampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glSamplerParameteri(fontSampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glSamplerParameteri(fontSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glSamplerParameteri(fontSampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void OpenGLText::PrepareForRender()
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glEnable(GL_BLEND);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	shaderFont.UseProgram();
	

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, fontTexture);
	glBindSampler(0, fontSampler);
	glBindVertexArray(fontVAO);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, fontVBO);

	shaderFont.SetUniform("color", glm::vec4(1, 1, 1, 1));

	shaderFont.SetUniform("tex", 0);

}

void OpenGLText::RenderText(const std::wstring &str, int data, int pixelSize, float x, float y, float sx, float sy)
{
	wstringstream meshStr;
	meshStr << data;
	wstring label = str + meshStr.str();
	RenderText(label, pixelSize, x, y, sx, sy);
}

void OpenGLText::RenderText(const std::wstring &str, int pixelSize, float x, float y, float sx, float sy)
{
	FT_Set_Pixel_Sizes(face, 0, pixelSize);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	const FT_GlyphSlot glyph = face->glyph;

	for (auto c : str) {
		if (FT_Load_Char(face, c, FT_LOAD_RENDER) != 0)
			continue;

		glTexImage2D(GL_TEXTURE_2D, 0, GL_R8,
			glyph->bitmap.width, glyph->bitmap.rows,
			0, GL_RED, GL_UNSIGNED_BYTE, glyph->bitmap.buffer);

		const float vx = x + glyph->bitmap_left * sx;
		const float vy = y + glyph->bitmap_top * sy;
		const float w = glyph->bitmap.width * sx;
		const float h = glyph->bitmap.rows * sy;

		struct {
			float x, y, s, t;
		} data[6] = {
			{ vx, vy, 0, 0 },
			{ vx, vy - h, 0, 1 },
			{ vx + w, vy, 1, 0 },
			{ vx + w, vy, 1, 0 },
			{ vx, vy - h, 0, 1 },
			{ vx + w, vy - h, 1, 1 }
		};

		glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), data, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
		glDrawArrays(GL_TRIANGLES, 0, 6);

		x += (glyph->advance.x >> 6) * sx;
		y += (glyph->advance.y >> 6) * sy;
	}

	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
}

void OpenGLText::CleanUp()
{
	FT_Done_Face(face);
	FT_Done_FreeType(ft_lib);
	glDeleteTextures(1, &fontTexture);
	glDeleteSamplers(1, &fontSampler);
	glDeleteBuffers(1, &fontVBO);
	glDeleteVertexArrays(1, &fontVAO);
}