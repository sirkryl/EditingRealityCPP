#pragma once
#include <ft2build.h>
#include <string>
#include <gl/glew.h>
#include <boost/thread/mutex.hpp>
#include FT_FREETYPE_H

namespace InteractiveFusion {
	class OpenGLText
	{
	public:
		void PrepareForRender();
		void RenderText(const std::wstring text, int pixelSize, float x, float y, float sx, float sy);
		void RenderText(const std::wstring text, int data, int pixelSize, float x, float y, float sx, float sy);
		void Initialize(std::string fontname);
		void FinishRender();
		bool IsInitialized();

		void CleanUp();
	private:
		GLuint fontTexture{ 0 }, fontSampler{ 0 };
		GLuint fontVBO{ 0 }, fontVAO{ 0 };

		FT_Library ft_lib{ nullptr };
		FT_Face face{ nullptr };

		boost::mutex text_mutex;
	};
}