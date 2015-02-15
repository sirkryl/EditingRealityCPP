#pragma once
#include "Renderable.h"

namespace InteractiveFusion {
	class Renderable3D :
		public Renderable
	{
	public:
		Renderable3D();
		Renderable3D(std::vector<Vertex> _vertices);
		Renderable3D(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles);

		~Renderable3D();

		virtual void UpdateEssentials();
		virtual void UpdateTransformationMatrices();
		void HighlightTrianglesWithColor(std::vector<int> _indicesOfTrianglesToBeHighlighted, ColorIF _highlightColor, bool _addToExistingColor);
		void Highlight(bool _highlight);
		void ClearColorHighlights();
		bool HasColorHighlights();
		void SwapToHighlightBuffer();

		void SetScale(bool _isPositiveFactor);
		void RotateX(int _degree);
		void RotateX(float _degree, glm::vec3 _axis);
		void RotateY(int _degree);
		void RotateY(float _degree, glm::vec3 _axis);

		virtual void Draw(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);
		virtual void DrawForColorPicking(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);

		virtual void SetUniforms(bool _highlight, bool _colorPicking, float _alpha, glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);
		virtual glm::mat4 CalculateModelMatrix();

		virtual void ApplyTransformation(glm::mat4 _vertexTransformation, glm::mat4 _normalTransformation);

		void SetTranslation(glm::vec3 _translation);

	protected:
		std::vector<Vertex> verticesWithHighlights;
		bool isHighlighted = false;

		glm::mat4 translation;

		glm::mat4 originTransform = glm::mat4(1.0f);

		glm::mat4 xRotation;
		glm::mat4 yRotation;
		glm::mat4 zRotation;
		glm::mat4 scaleMatrix;

		const int rotateBy = 5;
		const float scaleBy = 0.025f;

		float angleX = 0;
		float angleY = 0;
		float angleZ = 0;
		float scaleFactor = 1.0f;

	};
	

}

