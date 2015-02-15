#pragma once
#define NOMINMAX

#include "CommonStructs.h"
#include "VcgTypes.h"
#include "OpenGLShaderProgram.h"
#include "Renderable3D.h"
#include <vector>
namespace InteractiveFusion {

	class MeshContainer : public Renderable3D
	{
	public:
		MeshContainer();
		~MeshContainer();
		MeshContainer(const MeshContainer& _meshContainer);
		MeshContainer(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles);
		MeshContainer(VCGMesh& _vcgMesh);
		virtual void LoadFromFile(const char* filename);
		void PrepareMesh();

		void CopyInternalToVisibleData();
		void CopyVisibleToInternalData();

		virtual void UpdateEssentials();

		void Clean();

		OpenGLShaderProgram GetShaderProgram();

		virtual void Draw(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);
		virtual void DrawForColorPicking(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix);

		Vertex GetHitpoint(Ray _ray);
		
		void TranslateToPoint(glm::vec3 _point, std::vector<int> _orientation);
		
		void PositionOnRayInDistance(Ray _ray, int _distance);
		
		void ResetSelectedTransformation();

		PlaneParameters GetPlaneParameters();
		void SetPlaneParameters(PlaneParameters _planeParameters);

		
		void SetDeleted(bool flag);
		void SetDuplicate(bool flag);
		void SetPlane(bool _isPlane);
		void SetSelected(bool _isSelected);

		glm::vec3 GetBasePoint();

		std::vector<int> GetOrientation();
		
		bool IsPlane();
		bool IsDuplicate();
		bool IsDeleted();

		int FillHoles(int _maxHoleSize);
		void LaplacianSmooth(int _vertexStep);
		int MergeCloseVertices(float _distanceThreshold);
		void RemoveNonManifoldFaces();
		int RemoveSmallComponents(int _maxComponentSize);
		void UnsharpColor(float _alpha);
		
		const std::vector<Vertex>& GetVertices();
		const std::vector<Triangle>& GetTriangles();
		void GetVcgData(VCGMesh& _outputMesh);
		void GetAlignedVcgData(VCGMesh& _outputMesh, glm::mat4 _originTranslation, glm::mat4 _groundAlignmentRotation);
		
		void CleanUp();

	protected:
		VCGMesh currentMesh;

		virtual glm::mat4 CalculateModelMatrix();

		void CopyVisibleToVcg(VCGMesh& _outputMesh);
		

		glm::vec3 offSet;

		//glm::vec3 snapPoint;


		PlaneParameters planeParameters;

		std::vector<int> orientation;

		
		glm::mat4 selectScaleMatrix = glm::mat4(1.0f);
		glm::mat4 trashScaleMatrix = glm::mat4(1.0f);
		glm::mat4 cursorTranslation = glm::mat4(1.0f);


		bool attachToCursor = false;
		bool isSelected = false;
		bool colorSelection = false;
		bool isPlane = false;
		bool isDeleted = false;
		bool isDuplicate = false;

		bool IsRayIntersecting(Ray _ray);

		virtual void UpdateBounds();
		virtual void UpdateTransformationMatrices();
		void EnsureAttributesAreCorrect();
		glm::mat4 CalculateSnapTransform(std::vector<int> orien);
	};
}