#pragma once

#include "CommonStructs.h"
#include "OpenGLShaderProgram.h"
#include <memory>
#include <vector>
#include "VcgTypes.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/lockable_adapter.hpp>

namespace InteractiveFusion {
	class MeshContainer;
	class ModelData
	{
		friend class Selector;
	public:
		ModelData();
		virtual ~ModelData();
		void LoadFromFile(const char* fileName);
		void LoadFromData(std::vector<Vertex>& _vertices, std::vector<Triangle>& _triangles);
		void GenerateBuffers();
		void SwapToHighlightBuffers();

		glm::vec3 GetLowerBounds();
		glm::vec3 GetUpperBounds();

		bool IsReadyForRendering();
		void SetReadyForRendering(bool _flag);
		glm::vec3 GetCenterPoint();

		int FillHoles(int _holeSize);
		int FillHoles(int _holeSize, int _index);

		int RemoveConnectedComponents(int _maxComponentSize);

		void TemporarilyColorTriangles(int _index, std::vector<int> _triangles, ColorIF _color, bool _additive);
		void RemoveTemporaryTriangleColor(int _index);
		void TemporarilyColorMesh(int _index);
		void RemoveTemporaryMeshColor(int _index);
		void RemoveTemporaryMeshColor();


		glm::vec3* GetBasePoint(int _index);
		void MarkMeshAsSelected(int _index);
		int GetCurrentlySelectedMeshIndex();
		void UnselectMesh();

		void TranslateMeshToCursorRay(int _index, Ray _ray, int _distance);
		void TranslateMeshToPoint(int _index, glm::vec3 _point, std::vector<int> _orientation);
		void RotateMeshAroundAxis(int _index, float _degree, glm::vec3 _axis);
		/*void RotateMeshAroundHorizontalAxis(int _index, float _degree, glm::vec3 _axis);
		void RotateMeshAroundVerticalAxis(int _index, float _degree, glm::vec3 _axis);*/
		void ScaleMeshUp(int _index);
		void ScaleMeshDown(int _index);
		void ResetTemporaryTranslations(int _index);

		Vertex GetHitpoint(int _index, Ray _ray);

		int DuplicateMeshAndGetItsIndex(int _index);

		

		void SetMeshAsDeleted(int _index);
		
		void ResetToInitialState();

		int GetVisibleMeshCount();
		int GetMeshCount();
		int GetNumberOfVertices();
		int GetNumberOfTriangles();

		std::vector<int> GetPlaneIndices();
		std::vector<int> GetObjectIndices();
		bool IsEmpty();
		void AddObjectMeshToData(MeshContainer _mesh);
		void AddPlaneMeshToData(MeshContainer _plane, PlaneParameters _planeParameters);
		void MarkDataAsDeleted();

		void PermanentlyRemoveAllMeshWithDeletedFlag();

		bool GetAlignedVcgMesh(int _index, VCGMesh& _outputMesh);
		std::shared_ptr<MeshContainer> GetFirstMeshThatIsNotPlane();
		std::shared_ptr<MeshContainer> GetCurrentlySelectedMesh();
		int GetFirstMeshIndexThatIsNotPlane();
		void RemoveTemporaryTriangleColor();
		void CopyFrom(const ModelData& _modelData);
		void CopyPlanesFrom(ModelData& _modelData);
		void CleanUp();

		

		void Draw(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix);
		void DrawWithAssignedColorCodes(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix);
		void DrawNonStaticMeshWithAssignedColorCodes(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix);
		void DrawAllButIndexWithAssignedColorCodes(int _index, glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix);

		void SetGroundPlane(int _index);
		glm::mat4 GetNegativeGroundAlignmentRotation();
		void CombineAndAlignModelData(VCGMesh& _combinedMesh);

		void SetDefaultShaderProgram(OpenGLShaderProgram _defaultShaderProgram);

		void PlaneCutPreview(int _index, PlaneParameters _parameters);
		void CutWithPlane(int _index, PlaneParameters _parameters);

		void Lock();
		void Unlock();
		void TryLock();

	protected:

		boost::mutex scene_mutex;

		OpenGLShaderProgram defaultShaderProgram;

		std::vector<std::shared_ptr<MeshContainer>> currentMeshData;

		int currentlySelectedMesh = -1;

		glm::mat4 groundAlignmentRotation;
		glm::mat4 negativeGroundAlignmentRotation;

		int ReturnIndexOfMeshWithColorCode(int _colorCode);
		
		bool IsValidMeshDataIndex(int _index);

		private:
			static int colorCodeCount;
			int GetNextColorCode();
			bool isBusy = false;
	};
}