#pragma once
#include "Renderable.h"

namespace InteractiveFusion {
	class Renderable2D :
		public Renderable
	{
	public:
		Renderable2D();
		~Renderable2D();
		Renderable2D(std::vector<Vertex> _vertices);
		Renderable2D(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles);

		void SetAlpha(float _alpha);

		virtual void Draw(int _viewportWidth, int _viewportHeight);
		virtual void DrawForColorPicking(int _viewportWidth, int _viewportHeight);

		glm::mat4 CalculateTransformationMatrix(int _viewportWidth, int _viewportHeight);
		void SetUniforms(float _alpha, bool _colorPicking, glm::mat4 _transformationMatrix);

	protected:
		bool scaleWithViewport = false;
		float alphaValue = 1.0f;
	};
}
