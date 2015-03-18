#include "Renderable2D.h"

#include "ColorCoder.h"
#include <glm/gtc/matrix_transform.hpp>
#include "DebugUtility.h"
namespace InteractiveFusion {
	Renderable2D::Renderable2D()
	{
	}


	Renderable2D::~Renderable2D()
	{
	}

	Renderable2D::Renderable2D(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles) :
		Renderable(_vertices, _triangles)
	{
	}

	Renderable2D::Renderable2D(std::vector<Vertex> _vertices) :
		Renderable(_vertices)
	{
		//Renderable::Renderable(_vertices);
		DebugUtility::DbgOut(L"Renderable2D::Renderable2D()::_vertices size ", (int)_vertices.size());
	}

	void Renderable2D::SetAlpha(float _alpha)
	{
		alphaValue = _alpha;
	}

	void Renderable2D::SetUniforms(float _alpha, bool _colorPicking, glm::mat4 _transformationMatrix)
	{
		meshShaderProgram.SetUniform("alpha", _alpha);
		meshShaderProgram.SetUniform("colorPicking", _colorPicking);
		meshShaderProgram.SetUniform("pickColor", ColorCoder::IntToColor(colorCode));
		meshShaderProgram.SetUniform("transformationMatrix", _transformationMatrix);
	}

	glm::mat4 Renderable2D::CalculateTransformationMatrix(int _viewportWidth, int _viewportHeight)
	{
		if (scaleWithViewport)
		{
			float hRatio = (float)_viewportHeight / (float)_viewportWidth;
			glm::mat4 twoDScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(hRatio, 1.0f, 1.0f));
			return glm::translate(glm::mat4(1.0f), glm::vec3(0.95f - hRatio*0.15f, -0.75f, 0.0f)) * twoDScaleMatrix;
		}
		else
			return glm::mat4(1.0f);
	}

	void Renderable2D::Draw(int _viewportWidth, int _viewportHeight)
	{
		meshShaderProgram.UseProgram();

		SetUniforms(alphaValue, false, CalculateTransformationMatrix(_viewportWidth, _viewportHeight));

		FinishDrawing();
	}

	void Renderable2D::DrawForColorPicking(int _viewportWidth, int _viewportHeight)
	{
		if (colorCode == -1)
			return;

		meshShaderProgram.UseProgram();

		SetUniforms(1.0f, true, CalculateTransformationMatrix(_viewportWidth, _viewportHeight));

		FinishDrawing();
	}
}