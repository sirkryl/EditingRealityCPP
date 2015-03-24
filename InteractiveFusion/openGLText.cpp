#include "OpenGLText.h"

#include "OpenGLShaderProgram.h"
#include "StyleSheet.h"
#include "DebugUtility.h"
#include <Windows.h>
#include <sstream>
using namespace std;

namespace InteractiveFusion {

	OpenGLShaderProgram textShader;

	void OpenGLText::Initialize(string fontname)
	{
		if (IsInitialized())
			return;

		DebugUtility::DbgOut(L"OpenGLText::INitialize()::");
		if (FT_Init_FreeType(&ft_lib) != 0) {
			MessageBox(NULL, L"Couldn't initialize FreeType library", L"Error", MB_ICONERROR);
		}

		if (FT_New_Face(ft_lib, fontname.c_str(), 0, &face) != 0) {
			MessageBox(NULL, L"Unable to load myfont.ttf", L"Error", MB_ICONERROR);
		}

		glGenBuffers(1, &fontVBO);
		glGenVertexArrays(1, &fontVAO);
		glGenTextures(1, &fontTexture);
		glGenSamplers(1, &fontSampler);
		glSamplerParameteri(fontSampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glSamplerParameteri(fontSampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glSamplerParameteri(fontSampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glSamplerParameteri(fontSampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		OpenGLShader vertexShader;
		vertexShader.LoadShader("data\\shaders\\font.vert", GL_VERTEX_SHADER);
		OpenGLShader fragmentShader;
		fragmentShader.LoadShader("data\\shaders\\font.frag", GL_FRAGMENT_SHADER);

		textShader.CreateProgram();
		textShader.AddShaderToProgram(vertexShader);
		textShader.AddShaderToProgram(fragmentShader);
		textShader.LinkProgram();
	}

	bool OpenGLText::IsInitialized()
	{
		return glIsBuffer(fontVBO);
	}

	void OpenGLText::PrepareForRender()
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glEnable(GL_BLEND);
		glDisable(GL_CULL_FACE);
		glDisable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		textShader.UseProgram();


		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, fontTexture);
		glBindSampler(0, fontSampler);
		glBindVertexArray(fontVAO);
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, fontVBO);

		textShader.SetUniform("color", glm::vec4((float)StyleSheet::GetInstance()->GetDefaultTextColor().r / 255.0f, (float)StyleSheet::GetInstance()->GetDefaultTextColor().g / 255.0f, (float)StyleSheet::GetInstance()->GetDefaultTextColor().b / 255.0f, 1));

		textShader.SetUniform("tex", 0);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	}

	void OpenGLText::RenderText(std::wstring text, int data, int pixelSize, float x, float y, float sx, float sy)
	{
		RenderText(text + std::to_wstring(data), pixelSize, x, y, sx, sy);
	}

	void OpenGLText::RenderText(std::wstring text, int pixelSize, float x, float y, float sx, float sy)
	{
		/*text_mutex.lock();
		FT_Set_Pixel_Sizes(face, 0, pixelSize);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		FT_GlyphSlot g = face->glyph;
		for (auto& p : text) {
			if (FT_Load_Char(face, p, FT_LOAD_RENDER))
				continue;

			glTexImage2D(
				GL_TEXTURE_2D,
				0,
				GL_RED,
				g->bitmap.width,
				g->bitmap.rows,
				0,
				GL_RED,
				GL_UNSIGNED_BYTE,
				g->bitmap.buffer
				);

			float x2 = x + g->bitmap_left * sx;
			float y2 = -y - g->bitmap_top * sy;
			float w = g->bitmap.width * sx;
			float h = g->bitmap.rows * sy;

			GLfloat box[4][4] = {
				{ x2, -y2, 0, 0 },
				{ x2 + w, -y2, 1, 0 },
				{ x2, -y2 - h, 0, 1 },
				{ x2 + w, -y2 - h, 1, 1 },
			};

			glBufferData(GL_ARRAY_BUFFER, sizeof box, box, GL_DYNAMIC_DRAW);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

			x += (g->advance.x >> 6) * sx;
			y += (g->advance.y >> 6) * sy;
		}
		text_mutex.unlock();*/
		text_mutex.lock();
		if (text.length() == 0)
		{
			DebugUtility::DbgOut(L"length is zero");
			return;
		}
		if (pixelSize > 40)
		{
			DebugUtility::DbgOut(L"PixelSize way too big... ", pixelSize);
		}
		FT_Set_Pixel_Sizes(face, 0, pixelSize);

		
		FT_GlyphSlot glyph = face->glyph;

		for (auto& c : text) {
			if (FT_Load_Char(face, c, FT_LOAD_RENDER))
				continue;

			glTexImage2D(GL_TEXTURE_2D, 0, GL_R8,
				glyph->bitmap.width, glyph->bitmap.rows,
				0, GL_RED, GL_UNSIGNED_BYTE, glyph->bitmap.buffer);

			float vx = x + glyph->bitmap_left * sx;
			float vy = y + glyph->bitmap_top * sy;
			float w = glyph->bitmap.width * sx;
			float h = glyph->bitmap.rows * sy;

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
		text_mutex.unlock();
	}

	void OpenGLText::FinishRender()
	{
		glBindTexture(GL_TEXTURE_2D, 0);
		glBindVertexArray(0);
		textShader.UnUseProgram();
		glDisableVertexAttribArray(0);

	}

	void OpenGLText::CleanUp()
	{
		if (glIsBuffer(fontVBO))
		{
			FT_Done_Face(face);
			FT_Done_FreeType(ft_lib);
			glDeleteTextures(1, &fontTexture);
			glDeleteSamplers(1, &fontSampler);
			glDeleteBuffers(1, &fontVBO);
			glDeleteVertexArrays(1, &fontVAO);
		}
		textShader.DeleteProgram();
	}
}