#pragma once
#include "Renderable3D.h"

namespace InteractiveFusion {
	class SimplePlaneRenderable3D :
		public Renderable3D
	{
	public:
		SimplePlaneRenderable3D();
		~SimplePlaneRenderable3D();
		SimplePlaneRenderable3D(std::vector<Vertex> _vertices);

		void SetVertices(std::vector<Vertex> _vertices);

		PlaneParameters GetPlaneParameters();

		virtual glm::mat4 CalculateModelMatrix();

		void ApplyTransformation(glm::mat4 _vertexTransformation, glm::mat4 _normalTransformation);

		virtual void Draw(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);
	protected:
		PlaneParameters planeParameters;

		void CalculatePlaneParameters();

	};
}
