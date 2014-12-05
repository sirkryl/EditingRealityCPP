#include "common.h"
#include "MeshHelper.h"
#include "InteractiveFusion.h"
#include "OpenGLShaders.h"
#include "OpenGLCamera.h"
#include<wrap/io_trimesh/export.h>
#include<wrap/io_trimesh/import.h>

void MeshHelper::InitialLoadFromFile(const char* fileName)
{
	if (originalMesh->GetNumberOfVertices() != 0)
		originalMesh->ClearMesh();
	
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData.clear();
	shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
	originalMesh->SetColorCode(1);
	originalMesh->LoadMesh(fileName);
	//meshData.push_back(mesh);
}

void MeshHelper::FillHoles(int holeSize)
{
	int cnt = 0;
	int holeCnt = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		cDebug::DbgOut(L"fill hole #", cnt);
		(*mI)->ConvertToVCG();
		holeCnt += (*mI)->FillHoles(holeSize);
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();

		openGLWin.ShowStatusBarMessage(L"Filling holes..." + to_wstring(cnt + 1) + L"% of " + to_wstring(meshData.size()));
	}
	openGLWin.ShowStatusBarMessage(L"Filled " + to_wstring(holeCnt) + L" holes in " + to_wstring(cnt) + L"segments.");
}

void MeshHelper::FillHoles(int index, int holeSize)
{
	int cnt = 0;
	int holeCnt = 0;
	cDebug::DbgOut(L"fill hole #", cnt);
	meshData[index]->ConvertToVCG();
	holeCnt += meshData[index]->FillHoles(holeSize);
	meshData[index]->ParseData();
	meshData[index]->UpdateBuffers();

	openGLWin.ShowStatusBarMessage(L"Filling holes..." + to_wstring(cnt + 1) + L"% of " + to_wstring(meshData.size()));
	openGLWin.ShowStatusBarMessage(L"Filled " + to_wstring(holeCnt) + L" holes in " + to_wstring(cnt) + L"segments.");
}

void MeshHelper::RemoveSmallComponents(int size)
{
	int cnt = 0;
	int cmpCnt = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		cDebug::DbgOut(L"remove components #", cnt);
		cmpCnt += (*mI)->RemoveSmallComponents(size);
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
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
	meshData.push_back(mesh);
	return meshData.size()-1;
}

void MeshHelper::CleanMesh()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->CleanMesh();
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
	}
}

void MeshHelper::CombineAndExport()
{

	OPENFILENAME ofn;

	char szFileName[MAX_PATH] = "";

	ZeroMemory(&ofn, sizeof(ofn));

	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFilter = (LPCWSTR)L"PLY (*.ply)\0*.ply;\0STL (*.stl)\0*.stl\0OBJ (*.obj)\0*.obj\0OFF (*.off)\0*.off\0All Files (*.*)\0*.*\0";
	ofn.lpstrFile = (LPWSTR)szFileName;
	ofn.nMaxFile = MAX_PATH;
	ofn.Flags = OFN_EXPLORER | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT;
	ofn.lpstrDefExt = (LPCWSTR)L"ply";

	if (!GetSaveFileName(&ofn))
		return;

	char fileNameBuffer[500];

	// First arg is the pointer to destination char, second arg is
	// the pointer to source wchar_t, last arg is the size of char buffer
	wcstombs(fileNameBuffer, ofn.lpstrFile, 500);

	VCGMesh combinedMesh;
	vcg::tri::Allocator<VCGMesh>::AddVertices(combinedMesh, GetNumberOfVertices());

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
	vcg::tri::Allocator<VCGMesh>::AddFaces(combinedMesh, GetNumberOfFaces());
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
	
	//without save file dialog
	if (false)
	{ 
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
	}
	
	//cDebug::DbgOut(L"the path is : %s\n", ofn.lpstrFile);
	//getchar();
	//TCHAR iFileName;
	//wcsncpy(iFileName, ofn.lpstrFile, MAX__LENGTH);
	if (strstr(fileNameBuffer, ".ply"))
		vcg::tri::io::ExporterPLY<VCGMesh>::Save(combinedMesh, fileNameBuffer, vcg::tri::io::Mask::IOM_VERTCOLOR);
	else if (strstr(fileNameBuffer, ".obj"))
		vcg::tri::io::ExporterOBJ<VCGMesh>::Save(combinedMesh, fileNameBuffer, vcg::tri::io::Mask::IOM_VERTCOLOR);
	else if (strstr(fileNameBuffer, ".stl"))
		vcg::tri::io::ExporterSTL<VCGMesh>::Save(combinedMesh, fileNameBuffer, vcg::tri::io::Mask::IOM_VERTCOLOR);
	else if (strstr(fileNameBuffer, ".off"))
		vcg::tri::io::ExporterOFF<VCGMesh>::Save(combinedMesh, fileNameBuffer, vcg::tri::io::Mask::IOM_VERTCOLOR);
	else
		vcg::tri::io::ExporterPLY<VCGMesh>::Save(combinedMesh, fileNameBuffer, vcg::tri::io::Mask::IOM_VERTCOLOR);

}

void MeshHelper::GenerateOriginalBuffers()
{
	if (!originalMesh->AreBuffersInitialized())
	{ 
		cDebug::DbgOut(L"INITIALIZE ONCE");
		originalMesh->GenerateBOs();
		originalMesh->GenerateVAO();
	}
}

void MeshHelper::GenerateBuffers()
{
	cDebug::DbgOut(L"MeshHelper Generate Buffers");
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateBOs();

	}
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateVAO();
	}

}

int MeshHelper::GetNumberOfVertices()
{
	int noV = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		noV += (*mI)->GetNumberOfVertices();
	}
	return noV;
}

int MeshHelper::GetNumberOfFaces()
{
	int noF = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		noF += (*mI)->GetNumberOfTriangles();
	}
	return noF;
}

void MeshHelper::HighlightObjectsInOriginal(std::vector<int> triangles, ColorIF color, bool additive)
{
	originalMesh->HighlightObjects(triangles, color, additive);
}

void MeshHelper::HighlightObjects(int index, std::vector<int> triangles, ColorIF color, bool additive)
{
	meshData[index]->HighlightObjects(triangles, color, additive);
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
	originalMesh->ResetHighlights();
}

void MeshHelper::DrawOriginalMesh()
{
	originalMesh->Draw();
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
	if (meshData.size() == 0)
		return originalMesh->GetCenterPoint();

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