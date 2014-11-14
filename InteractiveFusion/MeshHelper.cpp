#include "common.h"
#include "MeshHelper.h"
#include "VcgMeshContainer.h"
#include "InteractiveFusion.h"
#include<wrap/io_trimesh/export.h>

int numberOfVertices = 0;
int numberOfFaces = 0;

void MeshHelper::InitialLoadFromFile(const char* fileName)
{
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

void MeshHelper::DrawAll()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->Draw();
	}
}

void MeshHelper::DrawAllForColorPicking()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (!(*mI)->IsWall())
			(*mI)->DrawBB();
	}
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