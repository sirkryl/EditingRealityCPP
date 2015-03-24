#include "Renderable3D.h"
#include <glm/gtc/matrix_transform.hpp>
#include "ColorCoder.h"
#include <sstream>
#include "DebugUtility.h"
namespace InteractiveFusion {
	Renderable3D::Renderable3D()
	{
		Renderable::Renderable();
	}


	Renderable3D::~Renderable3D()
	{
	}

	Renderable3D::Renderable3D(std::vector<Vertex> _vertices) :
		Renderable(_vertices)
	{
		
	}

	Renderable3D::Renderable3D(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles) :
		Renderable(_vertices, _triangles)
	{
		
	}

	void Renderable3D::UpdateEssentials()
	{
		UpdateBounds();
		UpdateTransformationMatrices();
	}

	void Renderable3D::UpdateTransformationMatrices()
	{
		originTransform = glm::translate(glm::mat4(1.0), -centerPoint);
		
		
	}

	void Renderable3D::SetTranslation(glm::vec3 _translation)
	{
		translation = glm::translate(glm::mat4(1.0f), _translation);;
	}


	void Renderable3D::HighlightTrianglesWithColor(std::vector<int> _indicesOfTrianglesToBeHighlighted, ColorIF _highlightColor, bool _addToExistingColor)
	{
		if (verticesWithHighlights.size() == 0)
			verticesWithHighlights.insert(verticesWithHighlights.begin(), vertices.begin(), vertices.end());
		for (auto& highlightedIndex : _indicesOfTrianglesToBeHighlighted)
		{
			if (_addToExistingColor)
			{
				verticesWithHighlights[highlightedIndex].r = glm::min(verticesWithHighlights[highlightedIndex].r + _highlightColor.r, 1.0f);
				verticesWithHighlights[highlightedIndex].g = glm::min(verticesWithHighlights[highlightedIndex].g + _highlightColor.g, 1.0f);
				verticesWithHighlights[highlightedIndex].b = glm::min(verticesWithHighlights[highlightedIndex].b + _highlightColor.b, 1.0f);
			}
			else
			{
				verticesWithHighlights[highlightedIndex].r = _highlightColor.r;
				verticesWithHighlights[highlightedIndex].g = _highlightColor.g;
				verticesWithHighlights[highlightedIndex].b = _highlightColor.b;
			}
		}
	}

	void Renderable3D::Highlight(bool _highlight)
	{
		isHighlighted = _highlight;
	}

	void Renderable3D::ClearColorHighlights()
	{
		verticesWithHighlights.clear();
	}

	bool Renderable3D::HasColorHighlights()
	{
		return verticesWithHighlights.size() != 0;
	}

	void Renderable3D::SwapToHighlightBuffer()
	{
		try
		{
			if (!glIsBuffer(vbo))
				return;
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			glBufferData(GL_ARRAY_BUFFER, verticesWithHighlights.size() * sizeof(Vertex), &verticesWithHighlights[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while swapping to highlight buffers in 3d renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new RenderingException(ss.str().c_str());
		}
	}

	void Renderable3D::SetScale(bool _isPositiveFactor)
	{
		if (_isPositiveFactor)
			scaleFactor += scaleBy;
		else
			scaleFactor -= scaleBy;

		scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(scaleFactor, scaleFactor, scaleFactor));
	}
	void Renderable3D::AddRotation(float _angle, glm::vec3 _axis)
	{
		quatRotation = glm::angleAxis(_angle, _axis) * quatRotation;

	}

	void Renderable3D::ResetRotation()
	{
		quatRotation = glm::quat();

	}
	/*void Renderable3D::RotateX(int _degree)
	{
		xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
	}

	void Renderable3D::RotateX(float _degree, glm::vec3 _axis)
	{
		xRotation = glm::rotate(glm::mat4(1.0), (float)_degree, _axis);
	}

	void Renderable3D::RotateY(int _degree)
	{
		yRotation = glm::rotate(glm::mat4(1.0), (float)_degree, glm::vec3(0.0f, 1.0f, 0.0f));
	}

	void Renderable3D::RotateY(float _degree, glm::vec3 _axis)
	{
		yRotation = glm::rotate(glm::mat4(1.0), (float)_degree, _axis);
	}*/

	void Renderable3D::ApplyTransformation(glm::mat4 _vertexTransformation, glm::mat4 _normalTransformation)
	{
		for (auto& vertex : vertices)
		{
			glm::vec4 tmp = glm::vec4(vertex.x, vertex.y, vertex.z, 1.0f);
			glm::vec4 tmpNormal = glm::vec4(vertex.normal_x, vertex.normal_y, vertex.normal_z, 0.0f);

			tmpNormal = _normalTransformation * tmpNormal;
			tmpNormal = glm::normalize(tmpNormal);

			tmp = _vertexTransformation * tmp;

			vertex.x = tmp.x;
			vertex.y = tmp.y;
			vertex.z = tmp.z;
			vertex.normal_x = tmpNormal.x;
			vertex.normal_y = tmpNormal.y;
			vertex.normal_z = tmpNormal.z;
		}
		UpdateBounds();
	}

	void Renderable3D::SetUniforms(bool _highlight, bool _colorPicking, float _alpha, glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix)
	{

		meshShaderProgram.SetUniform("highlight", _highlight);
		meshShaderProgram.SetUniform("colorPicking", _colorPicking);
		meshShaderProgram.SetUniform("alpha", _alpha);
		meshShaderProgram.SetUniform("pickColor", ColorCoder::IntToColor(colorCode));
		meshShaderProgram.SetUniform("matrices.projectionMatrix", _projectionMatrix);
		meshShaderProgram.SetUniform("matrices.viewMatrix", _viewMatrix);
	}

	glm::mat4 Renderable3D::CalculateModelMatrix()
	{
		return translation * scaleMatrix * glm::mat4_cast(quatRotation) * originTransform;
		/*return translation * scaleMatrix * zRotation * yRotation * xRotation * originTransform;*/
	}

	void Renderable3D::Draw(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix)
	{
		try
		{
			meshShaderProgram.UseProgram();

			SetUniforms(isHighlighted, false, 1.0f, _projectionMatrix, _viewMatrix);

			glm::mat4 modelMatrix = CalculateModelMatrix();

			meshShaderProgram.SetUniform("matrices.modelMatrix", modelMatrix);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while drawing 3d renderable with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new RenderingException(ss.str().c_str());
		}
		FinishDrawing();
	}

	void Renderable3D::DrawForColorPicking(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix)
	{
		try
		{
			meshShaderProgram.UseProgram();

			SetUniforms(false, true, 1.0f, _projectionMatrix, _viewMatrix);

			glm::mat4 modelMatrix = CalculateModelMatrix();

			meshShaderProgram.SetUniform("matrices.modelMatrix", modelMatrix);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while drawing 3d renderable for color picking with vertex count: ";
			ss << GetNumberOfVertices();
			ss << ", Exception type: ";
			ss << e.what();
			throw new RenderingException(ss.str().c_str());
		}
		FinishDrawing();
	}
}
