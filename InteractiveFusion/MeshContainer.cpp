#include "MeshContainer.h"
#include "DebugUtility.h"
#include "StopWatch.h"
#include "ColorCoder.h"

#include <glm/gtc/matrix_transform.hpp>

#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <wrap/io_trimesh/import.h>
#include "VcgException.h"


namespace InteractiveFusion {

	#pragma region


	MeshContainer::MeshContainer()  :
		Renderable3D()
	{ 
		orientation = { 0, 0, 0 }; 
		
	}

	MeshContainer::~MeshContainer()
	{
	}

	MeshContainer::MeshContainer(const MeshContainer& _meshContainer)
	{
		DebugUtility::DbgOut(L"MeshContainer::CopyConstructor");
		vertices = _meshContainer.vertices;
		triangles = _meshContainer.triangles;
		orientation = _meshContainer.orientation;
		colorCode = _meshContainer.colorCode;
		isPlane = _meshContainer.isPlane;
		planeParameters = _meshContainer.planeParameters;
		CopyVisibleToInternalData();
		UpdateEssentials();
	}

	MeshContainer::MeshContainer(vector<Vertex> _vertices, vector<Triangle> _triangles) :
		Renderable3D(_vertices, _triangles)
	{
		orientation = { 0, 0, 0 };
		
	}

	MeshContainer::MeshContainer(VCGMesh& _mesh)
	{
		try
		{
			vcg::tri::Append<VCGMesh, VCGMesh>::MeshCopy(currentMesh, _mesh);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while calling Append function in MeshContainer Constructor";
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}
	#pragma endregion Constructors

	#pragma region

	void MeshContainer::LoadFromFile(const char* filename)
	{
		try
		{
			vcg::tri::io::ImporterPLY<VCGMesh>::Open(currentMesh, filename);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception while calling Open function in LoadFromFile() with parameter ";
			ss << filename;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
		PrepareMesh();
	}

	void MeshContainer::PrepareMesh()
	{
		StopWatch stopWatch;
		stopWatch.Start();

		EnsureAttributesAreCorrect();

		float threshold = 0.0005f;
		int total = MergeCloseVertices(threshold);

		DebugUtility::DbgOut(L"Merged " + to_wstring(total) + L" close vertices in ", stopWatch.Stop());


		LaplacianSmooth(3);
		//UnsharpColor(0.3f);
		Clean();
		if (currentMesh.vn > 100)
		{
			RemoveSmallComponents(currentMesh.vn / 100);
		}
		else
		{
			DebugUtility::DbgOut(L"MeshContainer::PrepareMesh()::Not enough vertices for component removal: ", currentMesh.vn);
		}
		Clean();

	}

	#pragma endregion Initializing/Loading/Preparing

	#pragma region

	void MeshContainer::UpdateEssentials()
	{
		UpdateBounds();
		UpdateTransformationMatrices();
	}

	void MeshContainer::UpdateBounds()
	{
		Renderable3D::UpdateBounds();

		offSet.x = (centerPoint.x - lowerBounds.x);
		offSet.y = (centerPoint.y - lowerBounds.y);
		offSet.z = (centerPoint.z - lowerBounds.z);
	}

	void MeshContainer::UpdateTransformationMatrices()
	{
		Renderable3D::UpdateTransformationMatrices();

		if ((upperBounds.y - lowerBounds.y) > 0.3f)
		{
			float selectScaleFactor = 0.3f / (upperBounds.y - lowerBounds.y);
			selectScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3((selectScaleFactor, selectScaleFactor, selectScaleFactor)));
		}
		if ((upperBounds.y - lowerBounds.y) > 0.05f)
		{
			float trashScaleFactor = 0.05f / (upperBounds.y - lowerBounds.y);
			trashScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3((trashScaleFactor, trashScaleFactor, trashScaleFactor)));
		}
	}

	void MeshContainer::CopyInternalToVisibleData()
	{
		vertices.clear();
		triangles.clear();

		VCGMesh::VertexIterator vi;

		std::vector<int> VertexId((currentMesh).vert.size());
		int numvert = 0;

		for (vi = (currentMesh).vert.begin(); vi != (currentMesh).vert.end(); ++vi) if (!(*vi).IsD())
		{
			VertexId[vi - (currentMesh).vert.begin()] = numvert;

			Vertex vertex;
			vertex.x = (*vi).P()[0];
			vertex.y = (*vi).P()[1];
			vertex.z = (*vi).P()[2];
			vertex.normal_x = (*vi).N()[0];
			vertex.normal_y = (*vi).N()[1];
			vertex.normal_z = (*vi).N()[2];
			vertex.r = (*vi).C()[0] / 255.0f;
			vertex.g = (*vi).C()[1] / 255.0f;
			vertex.b = (*vi).C()[2] / 255.0f;
			vertices.push_back(vertex);

			numvert++;
		}

		for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
		{
			Triangle triangle;
			triangle.v1 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(0))];
			triangle.v2 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(1))];
			triangle.v3 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(2))];

			triangles.push_back(triangle);
		}

	}
	void MeshContainer::CopyVisibleToVcg(VCGMesh& _outputMesh)
	{
		try
		{
			_outputMesh.Clear();
			VCGMesh::VertexIterator vi = vcg::tri::Allocator<VCGMesh>::AddVertices(_outputMesh, vertices.size());

			std::vector<VCGMesh::VertexPointer> ivp;
			ivp.resize(vertices.size());

			for (int i = 0; i < vertices.size(); i++)
			{
				ivp[i] = &*vi; 
				vi->P() = vcg::Point3f(vertices[i].x, vertices[i].y, vertices[i].z);
				vi->C() = vcg::Color4b((int)(vertices[i].r * 255.0f), (int)(vertices[i].g * 255.0f), (int)(vertices[i].b * 255.0f), 255);
				vi->N() = vcg::Point3f(vertices[i].normal_x, vertices[i].normal_y, vertices[i].normal_z);
				++vi;
				/*_outputMesh.vert[i].P() = vcg::Point3f(vertices[i].x, vertices[i].y, vertices[i].z);
				_outputMesh.vert[i].C() = vcg::Color4b((int)(vertices[i].r * 255.0f), (int)(vertices[i].g * 255.0f), (int)(vertices[i].b * 255.0f), 255);
				_outputMesh.vert[i].N() = vcg::Point3f(vertices[i].normal_x, vertices[i].normal_y, vertices[i].normal_z);*/
			}

			VCGMesh::FaceIterator fi = vcg::tri::Allocator<VCGMesh>::AddFaces(_outputMesh, triangles.size());

			for (int i = 0; i < triangles.size(); i++)
			{
				fi->V(0) = ivp[triangles[i].v1];
				fi->V(1) = ivp[triangles[i].v2];
				fi->V(2) = ivp[triangles[i].v3];
				++fi;
				/*_outputMesh.face[i].V(0) = &_outputMesh.vert[triangles[i].v1];
				_outputMesh.face[i].V(1) = &_outputMesh.vert[triangles[i].v2];
				_outputMesh.face[i].V(2) = &_outputMesh.vert[triangles[i].v3];*/
			}
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in CopyVisibleToVcg() with parameter ";
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	void MeshContainer::CopyVisibleToInternalData()
	{
		CopyVisibleToVcg(currentMesh);
	}

	void MeshContainer::ResetSelectedTransformation()
	{
		scaleFactor = 1.0f;
		angleX = 0;
		angleY = 0;
		angleZ = 0;
		xRotation = glm::mat4(1.0f);
		yRotation = glm::mat4(1.0f);
		zRotation = glm::mat4(1.0f);
		quatRotation = glm::quat();
		scaleMatrix = glm::mat4(1.0f);
	}

	#pragma endregion Data Update/Parsing

	#pragma region

	void MeshContainer::EnsureAttributesAreCorrect()
	{
		try
		{
			vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
			vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromFF(currentMesh);
			vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalizedPerFace(currentMesh);
			vcg::tri::UpdateBounding<VCGMesh>::Box(currentMesh);
			vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
			vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromNone(currentMesh);
			vcg::tri::UpdateSelection<VCGMesh>::FaceFromBorderFlag(currentMesh);
			vcg::tri::UpdateFlags<VCGMesh>::VertexBorderFromNone(currentMesh);
			vcg::tri::UpdateSelection<VCGMesh>::VertexFromBorderFlag(currentMesh);
			vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromNone(currentMesh);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::EnsureAttributesAreCorrect() ";
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	void MeshContainer::Clean()
	{

		EnsureAttributesAreCorrect();

		try
		{
			int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(currentMesh);
			DebugUtility::DbgOut(L"Removed duplicates: ", dup);
			int dupFa = vcg::tri::Clean<VCGMesh>::RemoveDuplicateFace(currentMesh);
			DebugUtility::DbgOut(L"Removed duplicate faces: ", dupFa);
			int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(currentMesh);
			DebugUtility::DbgOut(L"Removed unreferenced: ", unref);
			int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(currentMesh);
			DebugUtility::DbgOut(L"Removed degenerate faces: ", deg);
			int zero = vcg::tri::Clean<VCGMesh>::RemoveZeroAreaFace(currentMesh);
			DebugUtility::DbgOut(L"Removed zero area faces: ", zero);

			vcg::tri::RequirePerVertexNormal(currentMesh);
			vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(currentMesh);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::Clean() ";
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}

	}

	#pragma endregion Mesh Cleaning

	#pragma region

	glm::mat4 MeshContainer::CalculateModelMatrix()
	{
		if (attachToCursor)
		{
			return translation * cursorTranslation * selectScaleMatrix * scaleMatrix * glm::mat4_cast(quatRotation) * originTransform;
		}
		else
			return glm::mat4(1.0f);
	}

	void MeshContainer::Draw(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix)
	{
		if (!isDeleted)
		{
			Renderable3D::Draw(_projectionMatrix, _viewMatrix);
		}
	}

	void MeshContainer::DrawForColorPicking(glm::mat4& _projectionMatrix, glm::mat4& _viewMatrix)
	{
		if (!isDeleted)
		{
			Renderable3D::DrawForColorPicking(_projectionMatrix, _viewMatrix);
		}
	}

	#pragma endregion Drawing

	#pragma region

	void MeshContainer::PositionOnRayInDistance(Ray _ray, int _distance)
	{
		glm::vec3 selectTranslation;
		float carryDistance = (float)_distance / 1000.0f;
		selectTranslation.x = _ray.startPoint.x + (carryDistance / 10.0f) * (_ray.endPoint.x - _ray.startPoint.x);
		selectTranslation.y = _ray.startPoint.y + (carryDistance / 10.0f) * (_ray.endPoint.y - _ray.startPoint.y);
		selectTranslation.z = _ray.startPoint.z + (carryDistance / 10.0f) * (_ray.endPoint.z - _ray.startPoint.z);

		cursorTranslation = glm::translate(glm::mat4(1.0), selectTranslation);
		attachToCursor = true;
	}

	glm::mat4 MeshContainer::CalculateSnapTransform(std::vector<int> orien)
	{
		glm::vec3 snapPoint;
		snapPoint.x = 0.0f;
		snapPoint.y = 0.0f;
		snapPoint.z = 0.0f;
		if (orien[0] != 0)
			snapPoint.x = orien[0] * offSet.x;
		if (orien[1] != 0)
			snapPoint.y = orien[1] * offSet.y;
		if (orien[2] != 0)
			snapPoint.z = orien[2] * offSet.z;

		return glm::translate(glm::mat4(1.0), snapPoint);
	}

	void MeshContainer::TranslateToPoint(glm::vec3 _point, std::vector<int> _orientation)
	{
		glm::mat4 pointTranslation = glm::translate(glm::mat4(1.0), _point);


		glm::mat4 snapTransform = CalculateSnapTransform(_orientation);


		glm::mat4 combinedTranslation = (snapTransform * pointTranslation * scaleMatrix * glm::mat4_cast(quatRotation) * originTransform);
		glm::mat4 normalTranslation = glm::transpose(glm::inverse(glm::mat4(combinedTranslation)));

		ApplyTransformation(combinedTranslation, normalTranslation);

		ResetSelectedTransformation();
		UpdateEssentials();
		UpdateVertexBuffer();

		attachToCursor = false;
	}


	#pragma endregion Transformations

	#pragma region

	bool MeshContainer::IsRayIntersecting(Ray _ray)
	{
		glm::vec3 nearPoint(_ray.startPoint.x, _ray.startPoint.y, _ray.startPoint.z);
		glm::vec3 farPoint(_ray.endPoint.x, _ray.endPoint.y, _ray.endPoint.z);
		glm::vec3 rayDirection = glm::normalize(farPoint - nearPoint);
		glm::vec3 dirfrac;

		dirfrac.x = 1.0f / rayDirection.x;
		dirfrac.y = 1.0f / rayDirection.y;
		dirfrac.z = 1.0f / rayDirection.z;

		float t1 = (lowerBounds.x - nearPoint.x)*dirfrac.x;
		float t2 = (upperBounds.x - nearPoint.x)*dirfrac.x;
		float t3 = (lowerBounds.y - nearPoint.y)*dirfrac.y;
		float t4 = (upperBounds.y - nearPoint.y)*dirfrac.y;
		float t5 = (lowerBounds.z - nearPoint.z)*dirfrac.z;
		float t6 = (upperBounds.z - nearPoint.z)*dirfrac.z;

		float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
		float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

		// if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
		if (tmax < 0)
		{
			DebugUtility::DbgOut(L"MeshContainer::IsRayIntersecting::FALSE::tmax < 0");
			return false;
		}

		// if tmin > tmax, ray doesn't intersect AABB
		if (tmin > tmax)
		{
			DebugUtility::DbgOut(L"MeshContainer::IsRayIntersecting::FALSE::tmin > tmax");
			return false;
		}


		return true;
	}

	Vertex MeshContainer::GetHitpoint(Ray _ray)
	{
		Vertex hitPoint;
		hitPoint.x = _ray.startPoint.x;
		hitPoint.y = _ray.startPoint.y;
		hitPoint.z = _ray.startPoint.z;

		if (!IsRayIntersecting(_ray))
			return hitPoint;

		glm::vec3 nearPoint(_ray.startPoint.x, _ray.startPoint.y, _ray.startPoint.z);
		glm::vec3 farPoint(_ray.endPoint.x, _ray.endPoint.y, _ray.endPoint.z);
		glm::vec3 rayDirection = glm::normalize(farPoint - nearPoint);

		float tinyBitBelowZero = 0.f - std::numeric_limits<float>::epsilon();
		float tinyBitAboveZero = 0.f + std::numeric_limits<float>::epsilon();
		float highestZ = -99999.0f;
		float u, v, tX;
		glm::vec3 normal;
		for (auto& triangle : triangles)
		{
			glm::vec3 vertex0(vertices[triangle.v1].x, vertices[triangle.v1].y, vertices[triangle.v1].z);
			glm::vec3 vertex1(vertices[triangle.v2].x, vertices[triangle.v2].y, vertices[triangle.v2].z);
			glm::vec3 vertex2(vertices[triangle.v3].x, vertices[triangle.v3].y, vertices[triangle.v3].z);

			glm::vec3 edge1 = vertex1 - vertex0;
			glm::vec3 edge2 = vertex2 - vertex0;

			glm::vec3 pVec = glm::cross(rayDirection, edge2);

			float det = glm::dot(edge1, pVec);
			if (det > tinyBitBelowZero && det < tinyBitAboveZero)
				continue;
			float invDet = 1 / det;
			glm::vec3 tVec = nearPoint - vertex0;
			u = glm::dot(tVec, pVec) * invDet;
			if (u < 0.0f || u > 1.0f)
				continue;
			glm::vec3 qVec = glm::cross(tVec, edge1);
			v = glm::dot(rayDirection, qVec) * invDet;
			if (v < 0.0f || u + v > 1.0f)
				continue;

			if (vertices[triangle.v1].z >= highestZ)
			{
				highestZ = vertices[triangle.v1].z;
				tX = glm::dot(edge2, qVec) * invDet;
				normal.x = vertices[triangle.v1].normal_x;
				normal.y = vertices[triangle.v1].normal_y;
				normal.z = vertices[triangle.v1].normal_z;
			}
		}

		glm::vec3 minPoint = nearPoint + rayDirection * tX;
		hitPoint.x = minPoint.x;
		hitPoint.y = minPoint.y;
		hitPoint.z = minPoint.z;
		hitPoint.normal_x = normal.x;
		hitPoint.normal_y = normal.y;
		hitPoint.normal_z = normal.z;

		//UNCOMMENT THIS TO SNAP TO VERTEX INSTEAD OF TO TRIANGLE

		/*float highestZ = -99999.0f;

		glm::vec3 v = farPoint - nearPoint;
		double c2 = glm::dot(v, v);
		float minDistance = std::numeric_limits<float>::max();
		for (int i = 0; i < vertices.size(); i += 1)
		{
		glm::vec3 point(vertices[i].x, vertices[i].y, vertices[i].z);

		glm::vec3 w = point - nearPoint;

		double c1 = glm::dot(w, v);

		double b = c1 / c2;
		glm::vec3 dd(v.x * b, v.y * b, v.z * b);
		glm::vec3 Pb = nearPoint + dd;
		float distance = glm::distance(point, Pb);
		if (distance < minDistance)
		{
		minDistance = distance;
		hitPoint.x = point.x;
		hitPoint.y = point.y;
		hitPoint.z = point.z;
		hitPoint.normal_x = vertices[i].normal_x;
		hitPoint.normal_y = vertices[i].normal_y;
		hitPoint.normal_z = vertices[i].normal_z;
		}
		}*/
		return hitPoint;

	}

	#pragma endregion Ray Intersection

	#pragma region

	int MeshContainer::FillHoles(int _maxHoleSize)
	{
		int holeCnt = 0;
		try
		{
			vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
			vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
			
			if (isPlane)
				holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingIntersectionFill<vcg::tri::SelfIntersectionEar<VCGMesh> >(currentMesh, _maxHoleSize, false);
			else
				holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingFill<vcg::tri::MinimumWeightEar<VCGMesh> >(currentMesh, _maxHoleSize, false);
			//int holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingFillAllButLargest<vcg::tri::MinimumWeightEar<VCGMesh> >(currentMesh, _maxHoleSize, false);
			//int holeCnt = vcg::tri::Hole<VCGMesh>::FillHoleEar<vcg::tri::TrivialEar<VCGMesh> >(currentMesh, holeSize, false);

			DebugUtility::DbgOut(L"Closed holes: ", holeCnt);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::FillHoles() with parameter ";
			ss << _maxHoleSize;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
		Clean();
		RemoveNonManifoldFaces();
		return holeCnt;
	}

	void MeshContainer::LaplacianSmooth(int _vertexStep)
	{
		try
		{
			vcg::tri::Smooth<VCGMesh>::VertexCoordLaplacian(currentMesh, _vertexStep, false, false);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::LaplacianSmooth() with parameter ";
			ss << _vertexStep;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	int MeshContainer::MergeCloseVertices(float _distanceThreshold)
	{
		try
		{
			return vcg::tri::Clean<VCGMesh>::MergeCloseVertex(currentMesh, _distanceThreshold);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::MergeCloseVertices() with parameter ";
			ss << _distanceThreshold;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	void MeshContainer::RemoveNonManifoldFaces()
	{
		EnsureAttributesAreCorrect();

		try
		{
			StopWatch stopWatch;
			stopWatch.Start();
			int manifoldFaceCount = vcg::tri::Clean<VCGMesh>::RemoveNonManifoldFace(currentMesh);

			int count;
			while (count = vcg::tri::Clean<VCGMesh>::CountNonManifoldVertexFF(currentMesh) > 0)
			{
				int total = vcg::tri::Clean<VCGMesh>::SplitNonManifoldVertex(currentMesh, 0);
				DebugUtility::DbgOut(L"Removed non manifold vertices: ", total);
				DebugUtility::DbgOut(L"Still remaining: ", count);
			}

			DebugUtility::DbgOut(L"Removed " + to_wstring(manifoldFaceCount) + L" non manifold faces in ", stopWatch.Stop());
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::RemoveNonManifoldFaces()";
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	int MeshContainer::RemoveSmallComponents(int _maxComponentSize)
	{
		std::pair<int, int> compCnt = std::pair<int, int>();
		try
		{
			vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
			vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
			EnsureAttributesAreCorrect();
			compCnt = vcg::tri::Clean<VCGMesh>::RemoveSmallConnectedComponentsSize(currentMesh, _maxComponentSize);

			DebugUtility::DbgOut(L"Removed components:", compCnt.second);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::RemoveSmallComponents() with parameter ";
			ss << _maxComponentSize;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
		Clean();

		return compCnt.second;
	}

	void MeshContainer::UnsharpColor(float _alpha)
	{
		try
		{
			float alphaorig = 1.0f;
			int smoothIter = 5;

			vcg::tri::Allocator<VCGMesh>::CompactVertexVector(currentMesh);
			vector<vcg::Color4f> colorOrig(currentMesh.vn);
			for (int i = 0; i < currentMesh.vn; ++i)
				colorOrig[i].Import(currentMesh.vert[i].C());

			vcg::tri::Smooth<VCGMesh>::VertexColorLaplacian(currentMesh, smoothIter);
			for (int i = 0; i < currentMesh.vn; ++i)
			{
				vcg::Color4f colorDelta = colorOrig[i] - vcg::Color4f::Construct(currentMesh.vert[i].C());
				vcg::Color4f newCol = colorOrig[i] * alphaorig + colorDelta*_alpha;       // Unsharp formula 
				vcg::Clamp(newCol); // Clamp everything in the 0..1 range
				currentMesh.vert[i].C().Import(newCol);

			}
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::UnsharpColor() with parameter ";
			ss << _alpha;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	void MeshContainer::GetVcgData(VCGMesh& _outputMesh)
	{
		VCGMesh temporaryMesh;
		CopyVisibleToVcg(temporaryMesh);
		try
		{
			vcg::tri::Append<VCGMesh, VCGMesh>::MeshCopy(_outputMesh, temporaryMesh);
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::GetVcgData()";
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}
	}

	void MeshContainer::GetAlignedVcgData(VCGMesh& _outputMesh, glm::mat4 _originTranslation, glm::mat4 _groundAlignmentRotation)
	{
		GetVcgData(_outputMesh);

		VCGMesh::VertexIterator vi;

		for (auto& vertex : _outputMesh.vert)
		{
			if (!vertex.IsD())
			{
				glm::vec4 positionVec4 = glm::vec4(vertex.P()[0], vertex.P()[1], vertex.P()[2], 1.0f);
				glm::vec4 normalVec4 = glm::vec4(vertex.N()[0], vertex.N()[1], vertex.N()[2], 0.0f);

				normalVec4 = _groundAlignmentRotation * _originTranslation * normalVec4;
				normalVec4 = glm::normalize(normalVec4);

				positionVec4 = _groundAlignmentRotation * _originTranslation * positionVec4;

				vertex.P() = vcg::Point3f(positionVec4.x, positionVec4.y, positionVec4.z);
				vertex.N() = vcg::Point3f(normalVec4.x, normalVec4.y, normalVec4.z);
			}
		}
	}

	#pragma endregion Filter Operations

	#pragma region

	glm::vec3 MeshContainer::GetBasePoint()
	{
		return glm::vec3(centerPoint.x, lowerBounds.y, centerPoint.z);
	}

	PlaneParameters MeshContainer::GetPlaneParameters()
	{
		return planeParameters;
	}

	std::vector<int> MeshContainer::GetOrientation()
	{
		return orientation;
	}

	OpenGLShaderProgram MeshContainer::GetShaderProgram()
	{
		return meshShaderProgram;
	}

	bool MeshContainer::IsDuplicate()
	{
		return isDuplicate;
	}

	bool MeshContainer::IsDeleted()
	{
		return isDeleted;
	}

	bool MeshContainer::IsPlane()
	{
		return isPlane;
	}

	const std::vector<Vertex>& MeshContainer::GetVertices()
	{
		return vertices;
	}

	const std::vector<Triangle>& MeshContainer::GetTriangles()
	{
		return triangles;
	}

	#pragma endregion Getter

	#pragma region

	void MeshContainer::SetPlaneParameters(PlaneParameters _planeParameters)
	{
		if (_planeParameters.x != -999.0f)
		{
			DebugUtility::DbgOut(L"MeshContainer::SetPlaneParamters::Valid Parameters");
			float paramerArray[4];
			paramerArray[0] = _planeParameters.x;
			paramerArray[1] = _planeParameters.y;
			paramerArray[2] = _planeParameters.z;
			paramerArray[3] = _planeParameters.d;

			int max = distance(paramerArray, max_element(paramerArray, paramerArray + 3));
			int min = distance(paramerArray, min_element(paramerArray, paramerArray + 3));


			int pos = 0;
			if (abs(max) > abs(min))
				pos = max;
			else
				pos = min;

			//orientation = pos;
			if (pos <= 2)
				orientation[pos] = 1;

			planeParameters = _planeParameters;
		}

	}

	void MeshContainer::SetDeleted(bool flag)
	{
		isDeleted = flag;
	}

	void MeshContainer::SetDuplicate(bool flag)
	{
		isDuplicate = flag;
	}

	void MeshContainer::SetPlane(bool _isPlane)
	{
		isPlane = _isPlane;
	}

	void MeshContainer::SetSelected(bool _isSelected)
	{
		isSelected = _isSelected;
		attachToCursor = false;
	}

	#pragma endregion Setter

	void MeshContainer::CleanUp()
	{
		currentMesh.Clear();
		Renderable3D::CleanUp();
	}

}