#include "OpenGL2DHelper.h"

void OpenGL2DHelper::InitialLoadFromFile(const char* fileName, int colorCode)
{
	shared_ptr<VCGMeshContainer> meshTo(new VCGMeshContainer);
	//meshTo->SetColorCode(100);
	meshTo->Load2DMesh(fileName);
	meshTo->SetColorCode(colorCode);
	meshTo->CleanMesh();
	meshTo->ParseData();
	//mesh->GenerateBOs();
	//mesh->GenerateVAO();
	//cDebug::DbgOut(L"vertices: " + mesh->GetNumberOfVertices());
	meshData2d.push_back(meshTo);
	
}

void OpenGL2DHelper::GenerateBuffers()
{
	cDebug::DbgOut(L"2D Generate Buffers");
	//numberOfVertices = 0;
	//numberOfFaces = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->GenerateBOs();

	}
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->GenerateVAO();
	}
}

void OpenGL2DHelper::DrawAll()
{
	if (!isOpen)
		meshData2d[0]->Draw();
	else
		meshData2d[1]->Draw();
	//cDebug::DbgOut(L"2D DrawAll");
	/*for (vector <shared_ptr<Mesh2D>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->Draw();
	}*/
}

void OpenGL2DHelper::DrawAllBB()
{
	//cDebug::DbgOut(L"2D DrawAll");
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->DrawBB();
	}
}

void OpenGL2DHelper::CleanUp()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData2d.begin(); mI != meshData2d.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData2d.clear();
}