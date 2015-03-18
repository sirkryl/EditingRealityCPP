#include "ModelData.h"

#include "MeshContainer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <wrap/io_trimesh/export.h>
#include <vcg/complex/algorithms/clip.h>
#include "DebugUtility.h"
#include "KeyState.h"

#include <unordered_map>

using namespace std;

namespace InteractiveFusion {
	

	int ModelData::colorCodeCount = 0;

	ModelData::ModelData()
	{
	}


	ModelData::~ModelData()
	{
	}

	
	void ModelData::LoadFromFile(const char* fileName)
	{
		CleanUp();

		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer));
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(GetNextColorCode());
		
		currentMeshData[currentMeshData.size() - 1]->LoadFromFile(fileName);
		currentMeshData[currentMeshData.size() - 1]->RemoveNonManifoldFaces();
		currentMeshData[currentMeshData.size() - 1]->CopyInternalToVisibleData();
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(defaultShaderProgram);
	}

	void ModelData::LoadFromData(std::shared_ptr<MeshContainer> _meshContainer)
	{
		CleanUp();

		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(_meshContainer->GetVertices(), _meshContainer->GetTriangles())));
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(GetNextColorCode());
		currentMeshData[currentMeshData.size() - 1]->CopyVisibleToInternalData();
		currentMeshData[currentMeshData.size() - 1]->PrepareMesh();
		currentMeshData[currentMeshData.size() - 1]->RemoveNonManifoldFaces();
		currentMeshData[currentMeshData.size() - 1]->CopyInternalToVisibleData();
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(defaultShaderProgram);
	}

	void ModelData::GenerateBuffers()
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			//(*mI)->ClearBuffers();
			(*mI)->GenerateBuffers();
		}
	}

	void ModelData::SwapToHighlightBuffers()
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if ((*mI)->HasColorHighlights())
				(*mI)->SwapToHighlightBuffer();
		}
	}

	glm::vec3 ModelData::GetLowerBounds()
	{
		glm::vec3 lowestBounds = glm::vec3(999.0f, 999.0f, 999.0f);

		for (auto &mesh : currentMeshData)
		{
			glm::vec3 meshLowerBounds = mesh->GetLowerBounds();
			lowestBounds.x = min(meshLowerBounds.x, lowestBounds.x);
			lowestBounds.y = min(meshLowerBounds.y, lowestBounds.y);
			lowestBounds.z = min(meshLowerBounds.z, lowestBounds.z);
		}
		return lowestBounds;
	}

	glm::vec3 ModelData::GetUpperBounds()
	{
		glm::vec3 uppestBounds = glm::vec3(-999.0f, -999.0f, -999.0f);

		for (auto &mesh : currentMeshData)
		{
			glm::vec3 meshUpperBounds = mesh->GetUpperBounds();
			uppestBounds.x = max(meshUpperBounds.x, uppestBounds.x);
			uppestBounds.y = max(meshUpperBounds.y, uppestBounds.y);
			uppestBounds.z = max(meshUpperBounds.z, uppestBounds.z);
		}
		return uppestBounds;
	}


	void ModelData::Draw(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix)
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if ((*mI)->AreBuffersInitialized())
				(*mI)->Draw(_projectionMatrix, _viewMatrix);
		}
	}

	void ModelData::DrawWithAssignedColorCodes(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix)
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->IsDeleted() && (*mI)->AreBuffersInitialized())
				(*mI)->DrawForColorPicking(_projectionMatrix, _viewMatrix);
		}
	}
	
	void ModelData::DrawNonStaticMeshWithAssignedColorCodes(glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix)
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->IsPlane() && !(*mI)->IsDeleted() && (*mI)->AreBuffersInitialized())
				(*mI)->DrawForColorPicking(_projectionMatrix, _viewMatrix);
		}
	}

	void ModelData::DrawAllButIndexWithAssignedColorCodes(int _index, glm::mat4* _projectionMatrix, glm::mat4* _viewMatrix)
	{
		int meshIndex = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->IsDeleted() && meshIndex != _index && (*mI)->AreBuffersInitialized())
				(*mI)->DrawForColorPicking(_projectionMatrix, _viewMatrix);
			meshIndex++;
		}
	}

	bool ModelData::IsReadyForRendering()
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->AreBuffersInitialized())
			{
				return false;
			}
		}
		return true;
	}

	glm::vec3 ModelData::GetCenterPoint()
	{
		glm::vec3 centerPoint(0.0f, 0.0f, 0.0f);
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			centerPoint += (*mI)->GetCenterPoint();
		}
		centerPoint = centerPoint / (float)currentMeshData.size();
		return centerPoint;
	}

	void ModelData::TemporarilyColorTriangles(int _index, std::vector<int> _triangles, ColorIF _color, bool _additive)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		DebugUtility::DbgOut(L"Hello, temporarily coloring ", _index);
		currentMeshData[_index]->HighlightTrianglesWithColor(_triangles, _color, _additive);
	}

	void ModelData::RemoveTemporaryTriangleColor(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->ClearColorHighlights();
	}

	void ModelData::RemoveTemporaryTriangleColor()
	{
		for (auto &mesh: currentMeshData)
			mesh->ClearColorHighlights();
	}

	void ModelData::TemporarilyColorMesh(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->Highlight(true);
	}

	void ModelData::RemoveTemporaryMeshColor(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->Highlight(false);
	}

	void ModelData::RemoveTemporaryMeshColor()
	{
		for (auto &mesh : currentMeshData)
			mesh->Highlight(false);
	}

	glm::vec3* ModelData::GetBasePoint(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return nullptr;
		return &currentMeshData[_index]->GetBasePoint();
		
	}

	void ModelData::MarkMeshAsSelected(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
		{
			return;
		}
		currentMeshData[_index]->SetSelected(true);
		currentlySelectedMesh = _index;
	}

	int ModelData::GetCurrentlySelectedMeshIndex()
	{
		return currentlySelectedMesh;
	}

	void ModelData::UnselectMesh()
	{
		if (currentlySelectedMesh != -1)
		{
			if (!IsValidMeshDataIndex(currentlySelectedMesh))
				return;
			currentMeshData[currentlySelectedMesh]->SetSelected(false);
			currentlySelectedMesh = -1;
		}
	}

	void ModelData::TranslateMeshToCursorRay(int _index, Ray _ray, int _distance)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->PositionOnRayInDistance(_ray, _distance);
	}

	void ModelData::TranslateMeshToPoint(int _index, glm::vec3 _point, vector<int> _orientation)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->TranslateToPoint(_point, _orientation);
		//currentMeshData[_index]->UpdateEssentials();
		//currentMeshData[_index]->UpdateBuffers();
	}

	void ModelData::RotateMeshAroundHorizontalAxis(int _index, float _degree, glm::vec3 _axis)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->RotateX(_degree, _axis);
	}

	void ModelData::RotateMeshAroundVerticalAxis(int _index, float _degree, glm::vec3 _axis)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->RotateY(_degree, _axis);
	}

	void ModelData::ResetTemporaryTranslations(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->ResetSelectedTransformation();
	}

	void ModelData::ScaleMeshUp(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->SetScale(true);
	}

	void ModelData::ScaleMeshDown(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->SetScale(false);
	}

	Vertex ModelData::GetHitpoint(int _index, Ray _ray)
	{
		if (!IsValidMeshDataIndex(_index))
		{
			DebugUtility::DbgOut(L"ModelData::GetHitPoint::No valid MeshData");
			return Vertex();
		}
		return currentMeshData[_index]->GetHitpoint(_ray);
	}

	int ModelData::FillHoles(int _holeSize)
	{
		int holeCount = 0;
		if (currentlySelectedMesh != -1)
		{
			holeCount += FillHoles(_holeSize, currentlySelectedMesh);
		}
		else
		{
			for (int index = 0; index < currentMeshData.size(); index++)
			{
				holeCount += FillHoles(_holeSize, index);
			}
		}
		return holeCount;
	}

	int ModelData::FillHoles(int _holeSize, int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return 0;

		int holeCount = 0;
		currentMeshData[_index]->CopyVisibleToInternalData();
		currentMeshData[_index]->Clean();
		currentMeshData[_index]->RemoveNonManifoldFaces();
		currentMeshData[_index]->FillHoles(_holeSize);
		currentMeshData[_index]->CopyInternalToVisibleData();
		currentMeshData[_index]->UpdateEssentials();
		return holeCount;
	}

	int ModelData::RemoveConnectedComponents(int _maxComponentSize)
	{
		int removedComponentCount = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			(*mI)->CopyVisibleToInternalData();
			removedComponentCount += (*mI)->RemoveSmallComponents(_maxComponentSize);
			(*mI)->CopyInternalToVisibleData();
			(*mI)->UpdateEssentials();
		}
		return removedComponentCount;
	}

	int ModelData::DuplicateMeshAndGetItsIndex(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return -1;

		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(currentMeshData[_index]->GetVertices(), currentMeshData[_index]->GetTriangles())));
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(GetNextColorCode());
		currentMeshData[currentMeshData.size() - 1]->CopyVisibleToInternalData();
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetDuplicate(true);
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(currentMeshData[_index]->GetShaderProgram());
		return currentMeshData.size() - 1;
	}

	void ModelData::SetMeshAsDeleted(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		currentMeshData[_index]->SetSelected(false);
		currentlySelectedMesh = -1;
		currentMeshData[_index]->SetDeleted(true);
	}

	void ModelData::MarkDataAsDeleted()
	{
		for (int i = 0; i < currentMeshData.size(); i++)
		{
			SetMeshAsDeleted(i);
		}
	}

	void ModelData::PermanentlyRemoveAllMeshWithDeletedFlag()
	{
		std::vector<int> indicesToBeRemoved;
		int meshCount = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if ((*mI)->IsDeleted())
			{
				indicesToBeRemoved.push_back(meshCount);
				(*mI)->CleanUp();
			}
			meshCount++;
		}

		int deletionOffset = 0;
		for (int i = 0; i < indicesToBeRemoved.size(); i++)
		{
			currentMeshData.erase(currentMeshData.begin() + i - deletionOffset);
			deletionOffset++;
		}
		
	}

	void ModelData::ResetToInitialState()
	{
		std::vector<int> indicesOfDuplicatesToBeRemoved;
		int meshCount = 0;
		for (auto& mesh : currentMeshData)
		{
			if (mesh->IsDuplicate())
			{
				indicesOfDuplicatesToBeRemoved.push_back(meshCount);
				meshCount++;
				continue;
			}
			if (mesh->IsDeleted())
			{
				mesh->SetDeleted(false);
				//(*mI)->ResetSelectedTransformation();
			}
			mesh->CopyInternalToVisibleData();
			mesh->UpdateEssentials();

			meshCount++;
		}

		meshCount = 0;
		for (auto& index : indicesOfDuplicatesToBeRemoved)
		{
			currentMeshData[index - meshCount]->CleanUp();
			currentMeshData.erase(currentMeshData.begin() + index - meshCount);
			meshCount++;
		}
		/*for (int i = 0; i < indicesOfDuplicatesToBeRemoved.size(); i++)
		{
			currentMeshData[indicesOfDuplicatesToBeRemoved[i] - meshCount]->CleanUp();
			currentMeshData.erase(currentMeshData.begin() + indicesOfDuplicatesToBeRemoved[i] - meshCount);
			meshCount++;
		}*/
	}

	void ModelData::SetDefaultShaderProgram(OpenGLShaderProgram _defaultShaderProgram)
	{
		defaultShaderProgram = _defaultShaderProgram;
	}

	void ModelData::CombineAndAlignModelData(VCGMesh& _combinedMesh)
	{
		glm::mat4 originTranslation = glm::translate(glm::mat4(1.0), -GetCenterPoint());
		for (auto& mesh : currentMeshData)
		{
			if (!mesh->IsDeleted())
			{
				VCGMesh singleMesh;
				mesh->GetAlignedVcgData(singleMesh, originTranslation, groundAlignmentRotation);
				vcg::tri::Append<VCGMesh, VCGMesh>::Mesh(_combinedMesh, singleMesh);
			}
		}
	}

	void ModelData::PlaneCutPreview(int _index, PlaneParameters _parameters)
	{
		if (!IsValidMeshDataIndex(_index))
			return;

		//DebugUtility::DbgOut(L"GraphicsControl::CutWithPlane ", _index);
		std::vector<int> verticesAbove;
		std::vector<int> verticesBelow;

		int index = 0;
		for (auto& vertex : currentMeshData[_index]->GetVertices())
		{
			float iO = vertex.x*_parameters.x + vertex.y*_parameters.y + vertex.z * _parameters.z - _parameters.d;
			if (iO > 0)
				verticesAbove.push_back(index);
			else
				verticesBelow.push_back(index);
			index++;
		}

		currentMeshData[_index]->HighlightTrianglesWithColor(verticesAbove, ColorIF{ 0.0f, 1.0f, 0.0f, 1.0f }, false);
		currentMeshData[_index]->HighlightTrianglesWithColor(verticesBelow, ColorIF{ 0.0f, 0.0f, 1.0f, 1.0f }, false);
	}

	void ModelData::CutWithPlane(int _index, PlaneParameters _parameters)
	{
		if (!IsValidMeshDataIndex(_index))
			return;

		DebugUtility::DbgOut(L"GraphicsControl::CutWithPlane ", _index);

/*		VCGMesh mesh;
		VCGMesh outputMesh;
		currentMeshData[_index]->GetVcgData(mesh);

		vcg::Box3f ClipBB;
		ClipBB.min = vcg::Point3f(-10000.0f, -10000.0f, _parameters.d);
		ClipBB.max = vcg::Point3f(10000.0f, 10000.0f, _parameters.d + 10000.0f);
		vcg::tri::GenericVertexInterpolator<VCGMesh> interp(mesh);
		vcg::tri::TriMeshClipper<VCGMesh>::Box(ClipBB, interp, mesh);
		
		DebugUtility::DbgOut(L"GraphicsControl::CutWithPlane::Clipped Mesh VN ", (int)mesh.vn);
		DebugUtility::DbgOut(L"GraphicsControl::CutWithPlane::Clipped Mesh FN ", (int)mesh.fn);

		currentMeshData[_index]->SetDeleted(true);
		currentMeshData.erase(currentMeshData.begin() + _index);
		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(mesh)));
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(currentMeshData.size() + 2);
		currentMeshData[currentMeshData.size() - 1]->CopyInternalToVisibleData();
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(currentMeshData[_index]->GetShaderProgram());
		UnselectMesh();*/
		std::vector<Vertex> verticesAbove;
		std::vector<Vertex> verticesBelow;

		std::unordered_map<int, int> vertexAboveIndexMap;
		std::unordered_map<int, int> vertexBelowIndexMap;

		std::unordered_map<int, std::vector<int>> indexMap;

		for (auto& triangle : currentMeshData[_index]->GetTriangles())
		{
			indexMap[triangle.v1] = { (int)triangle.v2, (int)triangle.v3 };
			indexMap[triangle.v2] = { (int)triangle.v1, (int)triangle.v3 };
			indexMap[triangle.v3] = { (int)triangle.v1, (int)triangle.v2 };
		}

		int index = 0;

		std::vector<Vertex> originalVertices = currentMeshData[_index]->GetVertices();

		for (auto& vertex : originalVertices)
		{
			float iO = vertex.x*_parameters.x + vertex.y*_parameters.y + vertex.z * _parameters.z + _parameters.d;
			if (iO > 0)
			{
				verticesAbove.push_back(vertex);
				vertexAboveIndexMap[index] = verticesAbove.size() - 1;
			}
			else
			{
				verticesBelow.push_back(vertex);
				vertexBelowIndexMap[index] = verticesBelow.size() - 1;
			}
			index++;
		}

		std::vector<Triangle> meshTriangles = currentMeshData[_index]->GetTriangles();

		std::vector<Triangle> trianglesAbove;
		std::vector<Triangle> trianglesBelow;
		
		std::unordered_map<int, int>::const_iterator mapIterator;

		for (auto& vertexIndexAbove : vertexAboveIndexMap)
		{
			vector<int> savedIndexValues = indexMap[vertexIndexAbove.first];
			for (size_t j = 0; j < savedIndexValues.size(); j += 3)
			{
				Triangle triangle;
				/*if the index is the first value in the current face, look for 2nd and 3rd in the current cluster
				and either add them to the new face or add new vertices to fill the face */
				if (savedIndexValues[j] == vertexIndexAbove.first)
				{
					//first index of face
					triangle.v1 = vertexIndexAbove.second;

					//second index of face

					//way faster than vector::find or unordered_set::find for that matter
					mapIterator = vertexAboveIndexMap.find(savedIndexValues[j + 1]);

					size_t indexOfSecondVertex;
					if (mapIterator != vertexAboveIndexMap.end())
					{
						indexOfSecondVertex = vertexAboveIndexMap[savedIndexValues[j + 1]];
					}
					else
					{
						indexOfSecondVertex = verticesAbove.size();
						verticesAbove.push_back(originalVertices[savedIndexValues[j + 1]]);
					}
					triangle.v2 = indexOfSecondVertex;

					//third index of face
					mapIterator = vertexAboveIndexMap.find(savedIndexValues[j + 2]);

					size_t indexOfThirdVertex;
					if (mapIterator != vertexAboveIndexMap.end())
					{
						indexOfThirdVertex = vertexAboveIndexMap[savedIndexValues[j + 2]];
					}
					else
					{
						indexOfThirdVertex = verticesAbove.size();
						verticesAbove.push_back(originalVertices[savedIndexValues[j + 2]]);
					}

					triangle.v3 = indexOfThirdVertex;

					trianglesAbove.push_back(triangle);
				}
				else if (savedIndexValues[j + 1] == vertexIndexAbove.first)
				{
					//first index of face
					mapIterator = vertexAboveIndexMap.find(savedIndexValues[j]);

					size_t indexOfFirstVertex;
					if (mapIterator != vertexAboveIndexMap.end())
					{
						indexOfFirstVertex = vertexAboveIndexMap[savedIndexValues[j]];
					}
					else
					{
						indexOfFirstVertex = verticesAbove.size();
						verticesAbove.push_back(originalVertices[savedIndexValues[j]]);
					}

					triangle.v1 = indexOfFirstVertex;

					//second index of face
					triangle.v2 = vertexIndexAbove.second;

					//third index of face
					mapIterator = vertexAboveIndexMap.find(savedIndexValues[j + 2]);

					size_t indexOfThirdVertex;
					if (mapIterator != vertexAboveIndexMap.end())
					{
						indexOfThirdVertex = vertexAboveIndexMap[savedIndexValues[j + 2]];
					}
					else
					{
						indexOfThirdVertex = verticesAbove.size();
						verticesAbove.push_back(originalVertices[savedIndexValues[j + 2]]);
					}
					triangle.v3 = indexOfThirdVertex;

					trianglesAbove.push_back(triangle);
				}
				else if (savedIndexValues[j + 2] == vertexIndexAbove.first)
				{
					//first index of face
					mapIterator = vertexAboveIndexMap.find(savedIndexValues[j]);

					size_t indexOfFirstVertex;
					if (mapIterator != vertexAboveIndexMap.end())
					{
						indexOfFirstVertex = vertexAboveIndexMap[savedIndexValues[j]];
					}
					else
					{
						indexOfFirstVertex = verticesAbove.size();
						verticesAbove.push_back(originalVertices[savedIndexValues[j]]);
					}
					triangle.v1 = indexOfFirstVertex;

					//second index of face
					mapIterator = vertexAboveIndexMap.find(savedIndexValues[j + 1]);

					size_t indexOfSecondVertex;
					if (mapIterator != vertexAboveIndexMap.end())
					{
						indexOfSecondVertex = vertexAboveIndexMap[savedIndexValues[j + 1]];
					}
					else
					{
						indexOfSecondVertex = verticesAbove.size();
						verticesAbove.push_back(originalVertices[savedIndexValues[j + 1]]);
					}
					triangle.v2 = indexOfSecondVertex;

					//third index of face
					triangle.v3 = vertexIndexAbove.second;
					trianglesAbove.push_back(triangle);
				}
			}
			/*Triangle newTriangle;
			auto firstTriangle = indexMap.find(vertexIndexAbove.first);
			if (firstTriangle != indexMap.end())
			{
				Triangle oldTriangle;
				oldTriangle.v1 = vertexIndexAbove.first;
				oldTriangle.v2 = indexMap[vertexIndexAbove.first][0];
				oldTriangle.v3 = indexMap[vertexIndexAbove.first][1];

				newTriangle.v1 = vertexIndexAbove.second;

				auto secondTriangle = vertexAboveIndexMap.find(oldTriangle.v2);
				if (secondTriangle != vertexAboveIndexMap.end())
				{
					newTriangle.v2 = vertexAboveIndexMap[secondTriangle];
				}
			}*/
			
		}

		currentMeshData[_index]->SetDeleted(true);
		//currentMeshData.erase(currentMeshData.begin() + _index);
		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(verticesAbove, trianglesAbove)));
		currentMeshData[currentMeshData.size() - 1]->CopyVisibleToInternalData();
		currentMeshData[currentMeshData.size() - 1]->PrepareMesh();
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(GetNextColorCode());
		
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(defaultShaderProgram);

		
	}

	int ModelData::ReturnIndexOfMeshWithColorCode(int _colorCode)
	{
		int selectedMeshIndex = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (_colorCode == (*mI)->GetColorCode())
				return selectedMeshIndex;
			selectedMeshIndex++;
		}
		return -1;
	}

	int ModelData::GetVisibleMeshCount()
	{
		int visibleMeshCount = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->IsDeleted())
				visibleMeshCount++;
		}
		return visibleMeshCount;
	}

	int ModelData::GetMeshCount()
	{
		return currentMeshData.size();
	}

	int ModelData::GetNumberOfVertices()
	{
		int numberOfVertices = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->IsDeleted())
				numberOfVertices += (*mI)->GetNumberOfVertices();
		}
		return numberOfVertices;
	}

	int ModelData::GetNumberOfTriangles()
	{
		int numberOfFaces = 0;
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			if (!(*mI)->IsDeleted())
				numberOfFaces += (*mI)->GetNumberOfTriangles();
		}
		return numberOfFaces;
	}

	std::vector<int> ModelData::GetPlaneIndices()
	{
		std::vector<int> planeIndices;

		int index = 0;
		for (auto& mesh : currentMeshData)
		{
			if (mesh->IsPlane() && !mesh->IsDeleted())
				planeIndices.push_back(index);
			index++;
		}
		
		return planeIndices;
	}

	std::vector<int> ModelData::GetObjectIndices()
	{
		std::vector<int> objectIndices;

		int index = 0;
		for (auto& mesh : currentMeshData)
		{
			if (!mesh->IsPlane() && !mesh->IsDeleted())
				objectIndices.push_back(index);
			index++;
		}

		return objectIndices;
	}

	bool ModelData::IsEmpty()
	{
		return currentMeshData.size() == 0;
	}
	bool ModelData::IsValidMeshDataIndex(int _index)
	{
		return (_index >= 0 && _index <= currentMeshData.size());
	}

	void ModelData::AddObjectMeshToData(MeshContainer* _mesh)
	{
		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(_mesh->GetVertices(), _mesh->GetTriangles())));
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(GetNextColorCode());
		currentMeshData[currentMeshData.size() - 1]->CopyVisibleToInternalData();
		currentMeshData[currentMeshData.size() - 1]->Clean();
		currentMeshData[currentMeshData.size() - 1]->RemoveNonManifoldFaces();
		currentMeshData[currentMeshData.size() - 1]->CopyInternalToVisibleData();
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(defaultShaderProgram);
	}

	void ModelData::AddPlaneMeshToData(MeshContainer* _plane, PlaneParameters _planeParameters)
	{
		currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(_plane->GetVertices(), _plane->GetTriangles())));
		currentMeshData[currentMeshData.size() - 1]->SetColorCode(GetNextColorCode());
		currentMeshData[currentMeshData.size() - 1]->CopyVisibleToInternalData();
		currentMeshData[currentMeshData.size() - 1]->MergeCloseVertices(0.005f);
		currentMeshData[currentMeshData.size() - 1]->Clean();
		currentMeshData[currentMeshData.size() - 1]->RemoveSmallComponents(_plane->GetNumberOfVertices() / 5);
		currentMeshData[currentMeshData.size() - 1]->RemoveNonManifoldFaces();
		currentMeshData[currentMeshData.size() - 1]->MergeCloseVertices(0.005f);
		currentMeshData[currentMeshData.size() - 1]->Clean();
		currentMeshData[currentMeshData.size() - 1]->CopyInternalToVisibleData();
		currentMeshData[currentMeshData.size() - 1]->UpdateEssentials();
		currentMeshData[currentMeshData.size() - 1]->SetPlane(true);
		currentMeshData[currentMeshData.size() - 1]->SetPlaneParameters(_planeParameters);
		currentMeshData[currentMeshData.size() - 1]->SetShaderProgram(defaultShaderProgram);

	}

	void ModelData::SetGroundPlane(int _index)
	{
		if (!IsValidMeshDataIndex(_index))
			return;
		PlaneParameters planeParameters = currentMeshData[_index]->GetPlaneParameters();
		glm::vec3 normalVector = glm::normalize(glm::vec3(planeParameters.x, planeParameters.y, planeParameters.z));

		glm::vec3 yAxis = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));

		if (normalVector.y < 0)
			yAxis.y = -1.0f;
		DebugUtility::DbgOut(L"ModelData::SetGroundPlane::", _index);
		DebugUtility::DbgOut(L"PlaneParameters::X", planeParameters.x);
		DebugUtility::DbgOut(L"PlaneParameters::Y", planeParameters.y);
		DebugUtility::DbgOut(L"PlaneParameters::Z", planeParameters.z);
		DebugUtility::DbgOut(L"PlaneParameters::D", planeParameters.d);

		glm::vec3 rotationAxis = glm::cross(normalVector, yAxis);

		float rotY = glm::acos(glm::dot(normalVector, yAxis)) * 180.0f / M_PI;

		groundAlignmentRotation = glm::rotate(glm::mat4(1.0), rotY, rotationAxis);
		negativeGroundAlignmentRotation = glm::rotate(glm::mat4(1.0), -rotY, rotationAxis);
	}

	glm::mat4 ModelData::GetNegativeGroundAlignmentRotation()
	{
		return negativeGroundAlignmentRotation;
	}

	void ModelData::CopyFrom(const ModelData& _modelData)
	{
		CleanUp();
		int meshCount = 0;

		for (auto & mesh : _modelData.currentMeshData)
		{
			if (!mesh->IsDeleted())
			{
				currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(mesh->GetVertices(), mesh->GetTriangles())));
				currentMeshData[meshCount]->CopyVisibleToInternalData();
				currentMeshData[meshCount]->SetColorCode(mesh->GetColorCode());
				currentMeshData[meshCount]->UpdateEssentials();
				currentMeshData[meshCount]->SetPlane(mesh->IsPlane());
				currentMeshData[meshCount]->SetPlaneParameters(currentMeshData[meshCount]->GetPlaneParameters());
				currentMeshData[meshCount]->SetShaderProgram(defaultShaderProgram);
				DebugUtility::DbgOut(L"ModelData::CopyFrom::Added " + to_wstring(meshCount) + L"nd mesh with " + to_wstring((int)mesh->GetVertices().size()) + L" vertices and " + to_wstring((int)mesh->GetTriangles().size()) + L"indices");
				DebugUtility::DbgOut(L"ModelData::CopyFrom::New " + to_wstring(meshCount) + L"nd mesh with " + to_wstring((int)currentMeshData[meshCount]->GetVertices().size()) + L" vertices and " + to_wstring((int)currentMeshData[meshCount]->GetTriangles().size()) + L"indices");

				meshCount++;
			}
			
		}
		groundAlignmentRotation = _modelData.groundAlignmentRotation;
		negativeGroundAlignmentRotation = _modelData.negativeGroundAlignmentRotation;
		DebugUtility::DbgOut(L"ModelData::CopyFrom::Added " + to_wstring(meshCount) + L" meshs");
	}

	void ModelData::CopyPlanesFrom(ModelData& _modelData)
	{
		CleanUp();
		int meshCount = 0;

		for (auto & mesh : _modelData.currentMeshData)
		{
			if (mesh->IsPlane())
			{
				currentMeshData.push_back(unique_ptr<MeshContainer>(new MeshContainer(mesh->GetVertices(), mesh->GetTriangles())));
				currentMeshData[meshCount]->CopyVisibleToInternalData();
				currentMeshData[meshCount]->SetColorCode(mesh->GetColorCode());
				currentMeshData[meshCount]->UpdateEssentials();
				currentMeshData[meshCount]->SetPlane(mesh->IsPlane());
				currentMeshData[meshCount]->SetPlaneParameters(currentMeshData[meshCount]->GetPlaneParameters());
				currentMeshData[meshCount]->SetShaderProgram(defaultShaderProgram);
				DebugUtility::DbgOut(L"ModelData::CopyFrom::Added " + to_wstring(meshCount) + L"nd mesh with " + to_wstring((int)mesh->GetVertices().size()) + L" vertices and " + to_wstring((int)mesh->GetTriangles().size()) + L"indices");
				DebugUtility::DbgOut(L"ModelData::CopyFrom::New " + to_wstring(meshCount) + L"nd mesh with " + to_wstring((int)currentMeshData[meshCount]->GetVertices().size()) + L" vertices and " + to_wstring((int)currentMeshData[meshCount]->GetTriangles().size()) + L"indices");

				meshCount++;
			}

		}
		groundAlignmentRotation = _modelData.groundAlignmentRotation;
		negativeGroundAlignmentRotation = _modelData.negativeGroundAlignmentRotation;
		DebugUtility::DbgOut(L"ModelData::CopyFrom::Added " + to_wstring(meshCount) + L" meshs");
	}

	bool ModelData::GetAlignedVcgMesh(int _index, VCGMesh& _outputMesh)
	{
		if (!IsValidMeshDataIndex(_index))
			return false;
		if (currentMeshData[_index]->IsDeleted())
			return false;

		glm::mat4 originTranslation = glm::translate(glm::mat4(1.0), -GetCenterPoint());

		currentMeshData[_index]->GetAlignedVcgData(_outputMesh, originTranslation, groundAlignmentRotation);

		return true;
	}

	MeshContainer* ModelData::GetFirstMeshThatIsNotPlane()
	{
		for (auto &mesh : currentMeshData)
		{
			if (!mesh->IsPlane())
			{
				return mesh.get();
			}
		}
		return nullptr;
	}

	MeshContainer* ModelData::GetCurrentlySelectedMesh()
	{
		if (currentlySelectedMesh != -1)
			return currentMeshData[currentlySelectedMesh].get();
		else
			return nullptr;
	}

	int ModelData::GetFirstMeshIndexThatIsNotPlane()
	{
		int meshCount = 0;
		for (auto &mesh : currentMeshData)
		{
			if (!mesh->IsPlane())
			{
				return meshCount;
			}
			meshCount++;
		}
		return 0;
	}

	int ModelData::GetNextColorCode()
	{
		int output = ModelData::colorCodeCount;
		ModelData::colorCodeCount++;
		return output;
	}

	void ModelData::CleanUp()
	{
		for (vector <unique_ptr<MeshContainer>>::iterator mI = currentMeshData.begin(); mI != currentMeshData.end(); ++mI)
		{
			(*mI)->CleanUp();
		}
		currentMeshData.clear();
	}
}
