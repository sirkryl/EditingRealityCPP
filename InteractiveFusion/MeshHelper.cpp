#include "common.h"
#include "MeshHelper.h"
#include "InteractiveFusion.h"
#include "OpenGLShaders.h"
#include "OpenGLCamera.h"
#include<wrap/io_trimesh/export.h>
#include<wrap/io_trimesh/import.h>

void MeshHelper::InitialLoadFromFile(const char* fileName)
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData.clear();
	shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
	mesh->SetColorCode(1);
	mesh->LoadMesh(fileName);
	meshData.push_back(mesh);
}

void MeshHelper::FillHoles(int holeSize)
{
	int cnt = 0;
	int holeCnt = 0;
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		if ((*mI)->GetNumberOfVertices() <= 1000)
		{
			numberOfVertices += (*mI)->GetNumberOfVertices();
			numberOfFaces += (*mI)->GetNumberOfTriangles();
			cDebug::DbgOut(L"no hole filling #", cnt);
			continue;
		}
		cDebug::DbgOut(L"fill hole #", cnt);
		(*mI)->ConvertToVCG();
		holeCnt += (*mI)->FillHoles(holeSize * 100);
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfTriangles();

		openGLWin.ShowStatusBarMessage(L"Filling holes..." + to_wstring(cnt + 1) + L"% of " + to_wstring(meshData.size()));
	}
	openGLWin.ShowStatusBarMessage(L"Filled " + to_wstring(holeCnt) + L" holes in " + to_wstring(cnt) + L"segments.");
}

void MeshHelper::RemoveSmallComponents(int size)
{
	int cnt = 0;
	int cmpCnt = 0;
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		cDebug::DbgOut(L"remove components #", cnt);
		cmpCnt += (*mI)->RemoveSmallComponents(size);
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfTriangles();
	}
	openGLWin.ShowStatusBarMessage(L"Deleted " + to_wstring(cmpCnt) + L"components");
}

void MeshHelper::ResetAll()
{
	std::vector<int> removeIndices;
	int cnt = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if ((*mI)->IsDuplicate())
		{
			removeIndices.push_back(cnt);
			cnt++;
			continue;
		}
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
		cnt++;
	}

	cnt = 0;
	for (int i = 0; i < removeIndices.size(); i++)
	{
		DeleteMesh(removeIndices[i]-cnt);
		cnt++;
	}
}

void MeshHelper::DeleteMesh(int index)
{
	numberOfVertices -= meshData[index]->GetNumberOfVertices();
	numberOfFaces -= meshData[index]->GetNumberOfTriangles();
	meshData[index]->ClearMesh();
	//delete meshData[index];
	meshData.erase(meshData.begin() + index);// std::remove(meshData.begin(), meshData.end(), meshData[index]), meshData.end());
}

int MeshHelper::DuplicateMesh(int index)
{
	shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
	mesh->SetColorCode(meshData.size() + 2);

	mesh->ConvertToVCG(meshData[index]->GetVertices(), meshData[index]->GetIndices());
	mesh->ParseData();
	mesh->SetDuplicate(true);
	mesh->GenerateBOs();
	mesh->GenerateVAO();
	numberOfVertices += mesh->GetNumberOfVertices();
	numberOfFaces += mesh->GetNumberOfTriangles();
	meshData.push_back(mesh);
	return meshData.size()-1;
}

void MeshHelper::CleanMesh()
{
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->CleanMesh();
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();

		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfTriangles();
	}
}

void MeshHelper::CombineAndExport()
{
	VCGMesh combinedMesh;
	vcg::tri::Allocator<VCGMesh>::AddVertices(combinedMesh, numberOfVertices);

	std::vector<int> vertOffset;
	vertOffset.push_back(0);
	int meshCnt = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		std::vector<Vertex> vertices = (*mI)->GetVertices();

		int vertCount = 0;
		for (int i = 0; i < vertices.size(); i += 1)
		{
			combinedMesh.vert[vertOffset[meshCnt] + vertCount].P() = vcg::Point3f(vertices[i].x, vertices[i].y, vertices[i].z);
			combinedMesh.vert[vertOffset[meshCnt] + vertCount].C() = vcg::Color4b((int)(vertices[i].r * 255.0f), (int)(vertices[i].g * 255.0f), (int)(vertices[i].b * 255.0f), 255);
			combinedMesh.vert[vertOffset[meshCnt] + vertCount].N() = vcg::Point3f(vertices[i].normal_x, vertices[i].normal_y, vertices[i].normal_z);
			vertCount++;
		}
		vertOffset.push_back(vertOffset[meshCnt] + vertCount);
		meshCnt++;
	}

	std::vector<int> faceOffset;
	faceOffset.push_back(0);
	meshCnt = 0;
	vcg::tri::Allocator<VCGMesh>::AddFaces(combinedMesh, numberOfFaces);
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		std::vector<Triangle> indices = (*mI)->GetIndices();
		int faceCount = 0;
		for (int i = 0; i < indices.size(); i += 1){
			combinedMesh.face[faceOffset[meshCnt] + faceCount].V(0) = &combinedMesh.vert[vertOffset[meshCnt] + indices[i].v1];
			combinedMesh.face[faceOffset[meshCnt] + faceCount].V(1) = &combinedMesh.vert[vertOffset[meshCnt] + indices[i].v2];
			combinedMesh.face[faceOffset[meshCnt] + faceCount].V(2) = &combinedMesh.vert[vertOffset[meshCnt] + indices[i].v3];
			faceCount++;
		}
		faceOffset.push_back(faceOffset[meshCnt] + faceCount);
		meshCnt++;
	}
	string path = "data\\output\\";
	string fileName = "combined";
	string ext = ".ply";
	struct stat buffer;
	string filePath = path;
	filePath.append(fileName);
	filePath.append(ext);
	int cnt = 1;
	while (stat(filePath.c_str(), &buffer) != -1)
	{
		ostringstream convert;
		convert << cnt;
		fileName = "combined_" + convert.str();
		filePath = path;
		filePath.append(fileName);
		filePath.append(ext);
		cnt++;
	}

	vcg::tri::io::ExporterPLY<VCGMesh>::Save(combinedMesh, filePath.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR);
}

void MeshHelper::GenerateBuffers()
{
	cDebug::DbgOut(L"MeshHelper Generate Buffers");
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateBOs();
		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfTriangles();

	}
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateVAO();
	}

}

int MeshHelper::GetNumberOfVertices()
{
	return numberOfVertices;
}

int MeshHelper::GetNumberOfFaces()
{
	return numberOfFaces;
}

void MeshHelper::HighlightObjects(int index, std::vector<int> triangles, ColorIF color)
{
	meshData[index]->HighlightObjects(triangles, color);
}

void MeshHelper::RemoveHighlightObjects(int index)
{
	meshData[index]->ResetHighlights();
}

void MeshHelper::RemoveAllHighlights()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ResetHighlights();
	}
}
void MeshHelper::DrawAll()
{
	//cDebug::DbgOut(L"Meshhelper DrawAll");
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->Draw();
	}
	//shaderColor.UnUseProgram();
}

void MeshHelper::DrawAllForColorPicking()
{
	//cDebug::DbgOut(L"Meshhelper DrawAllForColorPicking");
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (!(*mI)->IsWall())
			(*mI)->DrawBB();
	}
	//shaderColor.UnUseProgram();
}

glm::vec3 MeshHelper::GetCombinedCenterPoint()
{
	glm::vec3 centerPoint(0.0f, 0.0f, 0.0f);
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		centerPoint += (*mI)->GetCenterPoint();
	}
	centerPoint = centerPoint / (float)meshData.size();
	cDebug::DbgOut(L"centerPoint x: ", centerPoint.x);
	cDebug::DbgOut(L"centerPoint y: ", centerPoint.y);
	cDebug::DbgOut(L"centerPoint z: ", centerPoint.z);
	return centerPoint;
}

void MeshHelper::CleanAndParse(const char* fileName, std::vector<Vertex> &startingVertices, std::vector<Triangle> &startingIndices)
{
	VCGMesh mesh;
	vcg::tri::io::ImporterPLY<VCGMesh>::Open(mesh, fileName);

	if (mesh.vn > 1000)
	{
		int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(mesh);
		cDebug::DbgOut(_T("Removed duplicates:"), dup);
		int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(mesh);
		cDebug::DbgOut(_T("Removed unreferenced:"), unref);
		int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(mesh);
		cDebug::DbgOut(_T("Removed degenerate faces:"), deg);
	}
	vcg::tri::RequirePerVertexNormal(mesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(mesh);
	std::clock_t start;
	double duration;

	start = std::clock();
	VCGMesh::VertexIterator vi;
	std::vector<int> VertexId((mesh).vert.size());
	//std::vector<float> colors;
	int numvert = 0;
	int curNormalIndex = 1;

	for (vi = (mesh).vert.begin(); vi != (mesh).vert.end(); ++vi) if (!(*vi).IsD())
	{
		VertexId[vi - (mesh).vert.begin()] = numvert;
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
		startingVertices.push_back(vertex);

		numvert++;
	}

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (mesh).face.begin(); fi != (mesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		Triangle triangle;
		triangle.v1 = VertexId[vcg::tri::Index((mesh), (*fi).V(0))];
		triangle.v2 = VertexId[vcg::tri::Index((mesh), (*fi).V(1))];
		triangle.v3 = VertexId[vcg::tri::Index((mesh), (*fi).V(2))];

		startingIndices.push_back(triangle);
	}

	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	cDebug::DbgOut(L"Parse duration: ", duration);

}