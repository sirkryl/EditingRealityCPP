#include "SimplePlaneRenderable3D.h"
#include "DebugUtility.h"

namespace InteractiveFusion {
	SimplePlaneRenderable3D::SimplePlaneRenderable3D()
	{
	}


	SimplePlaneRenderable3D::~SimplePlaneRenderable3D()
	{
	}

	SimplePlaneRenderable3D::SimplePlaneRenderable3D(std::vector<Vertex> _vertices) :
		Renderable3D(_vertices)
	{
	}

	void SimplePlaneRenderable3D::SetVertices(std::vector<Vertex> _vertices)
	{
		vertices = _vertices;
	}

	PlaneParameters SimplePlaneRenderable3D::GetPlaneParameters()
	{
		PlaneParameters outputParameters = planeParameters;
		if (glm::abs(translation[3].x) > glm::abs(translation[3].y) &&
			glm::abs(translation[3].x) > glm::abs(translation[3].z))
			outputParameters.d += translation[3].x;
		else if (glm::abs(translation[3].y) > glm::abs(translation[3].x) &&
			glm::abs(translation[3].y) > glm::abs(translation[3].z))
			outputParameters.d -= translation[3].y;
		else if (glm::abs(translation[3].z) > glm::abs(translation[3].x) &&
			glm::abs(translation[3].z) > glm::abs(translation[3].y))
			outputParameters.d -= translation[3].z;
		return outputParameters;
	}

	glm::mat4 SimplePlaneRenderable3D::CalculateModelMatrix()
	{
		//return translation * scaleMatrix * zRotation * yRotation * xRotation;
		//return Renderable3D::CalculateModelMatrix();
		//return -originTransform * translation * xRotation * originTransform;
		return translation * xRotation;
	}

	void SimplePlaneRenderable3D::CalculatePlaneParameters()
	{
		if (vertices.size() > 2)
		{
			glm::vec3 point1(vertices[0].x, vertices[0].y, vertices[0].z);
			glm::vec3 point2(vertices[1].x, vertices[1].y, vertices[1].z);
			glm::vec3 point3(vertices[2].x, vertices[2].y, vertices[2].z);
			glm::vec3 direction = glm::cross((point3 - point1), (point2 - point1));
			direction = glm::normalize(direction);
			float offset = glm::dot(point1, direction);
			planeParameters.x = direction.x;
			planeParameters.y = direction.y;
			planeParameters.z = direction.z;
			planeParameters.d = offset;

			//DebugUtility::DbgOut(L"PlaneRenderer::GetPlaneParameters()::outputParameters x: ", outputParameters.x);
			//DebugUtility::DbgOut(L"PlaneRenderer::GetPlaneParameters()::outputParameters y: ", outputParameters.y);
			//DebugUtility::DbgOut(L"PlaneRenderer::GetPlaneParameters()::outputParameters z: ", outputParameters.z);
		}
	}

	void SimplePlaneRenderable3D::ApplyTransformation(glm::mat4 _vertexTransformation, glm::mat4 _normalTransformation)
	{
		for (auto& vertex : vertices)
		{
			glm::vec4 tmp = glm::vec4(vertex.x, vertex.y, vertex.z, 1.0f);
			glm::vec4 tmpNormal = glm::vec4(vertex.normal_x, vertex.normal_y, vertex.normal_z, 0.0f);

			tmpNormal = -originTransform * _normalTransformation * originTransform * tmpNormal;
			tmpNormal = glm::normalize(tmpNormal);

			tmp = -originTransform * _vertexTransformation * originTransform * tmp;

			vertex.x = tmp.x;
			vertex.y = tmp.y;
			vertex.z = tmp.z;
			vertex.normal_x = tmpNormal.x;
			vertex.normal_y = tmpNormal.y;
			vertex.normal_z = tmpNormal.z;
		}
		UpdateBounds();
		UpdateTransformationMatrices();
		CalculatePlaneParameters();
		UpdateVertexBuffer();
		
	}

	void SimplePlaneRenderable3D::Draw(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix)
	{
		meshShaderProgram.UseProgram();

		SetUniforms(isHighlighted, false, 0.3f, _projectionMatrix, _viewMatrix);

		glm::mat4 modelMatrix = CalculateModelMatrix();

		meshShaderProgram.SetUniform("matrices.modelMatrix", modelMatrix);

		FinishDrawing();
	}
}
