#include "common.h"
#include "colorCoding.h"
#include "vcgMesh.h"
#include "openGLShaders.h"
#include "openGLWin.h"
#include "openGLCamera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/clustering.h>
#include<wrap/io_trimesh/import.h>
#include<wrap/io_trimesh/export.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/complex/algorithms/update/normal.h>


VCGMeshContainer::VCGMeshContainer() { }

VCGMeshContainer::~VCGMeshContainer() { }

void VCGMeshContainer::LoadMesh(const char* filename)
{
	vcg::tri::io::ImporterPLY<VCGMesh>::Open(currentMesh, filename);

	/*
	vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
	vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromFF(currentMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalizedPerFace(currentMesh);
	vcg::tri::UpdateBounding<VCGMesh>::Box(currentMesh);
	vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
	vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromNone(currentMesh);*/
	
	RemoveNonManifoldFace();
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);

	float threshold = 0.005f;
	int total = MergeCloseVertices(threshold);

	cDebug::DbgOut(_T("Merged close vertices: "), total);
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Merged close vertices in ", elapsedTime);

	//int stepSmoothNum = 3;
	//size_t cnt = vcg::tri::UpdateSelection<VCGMesh>::VertexFromFaceStrict(currentMesh);
	//vcg::tri::Smooth<VCGMesh>::VertexCoordLaplacian(currentMesh, stepSmoothNum, cnt>0);
	LaplacianSmooth(3);
	CleanMesh();
	RemoveSmallComponents(500);
	CleanMesh();
	ParseData();
}

void VCGMeshContainer::RemoveNonManifoldFace()
{
	vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
	vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromFF(currentMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalizedPerFace(currentMesh);
	vcg::tri::UpdateBounding<VCGMesh>::Box(currentMesh);
	vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
	vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromNone(currentMesh);
	int test = vcg::tri::Clean<VCGMesh>::RemoveNonManifoldFace(currentMesh);
	cDebug::DbgOut(_T("Removed non manifold faces: "), test);
}

void VCGMeshContainer::LaplacianSmooth(int step)
{
	vcg::tri::Smooth<VCGMesh>::VertexCoordLaplacian(currentMesh, step, false, false);
}

int VCGMeshContainer::MergeCloseVertices(float threshold)
{
	return vcg::tri::Clean<VCGMesh>::MergeCloseVertex(currentMesh, threshold);
}

void VCGMeshContainer::LoadMesh(std::vector<float> inputVertices, std::vector<GLuint> inputIndices, std::vector<float> inputNormals)
{
	currentMesh.Clear();
	vertices.clear();
	indices.clear();
	vcg::tri::Allocator<VCGMesh>::AddVertices(currentMesh, inputVertices.size() / 6);

	int vertCount = 0;
	for (int i = 0; i < vertices.size(); i += 6)
	{
		currentMesh.vert[vertCount].P() = vcg::Point3f(inputVertices[i], inputVertices[i + 1], inputVertices[i + 2]);
		currentMesh.vert[vertCount].C() = vcg::Color4b((int)(inputVertices[i + 3] * 255.0f), (int)(inputVertices[i + 4] * 255.0f), (int)(inputVertices[i + 5] * 255.0f), 255);
		vertCount++;

	}

	vcg::tri::Allocator<VCGMesh>::AddFaces(currentMesh, inputIndices.size() / 3);

	int faceCount = 0;
	for (int i = 0; i<inputIndices.size(); i += 3){
		currentMesh.face[faceCount].V(0) = &currentMesh.vert[inputIndices[i]];
		currentMesh.face[faceCount].V(1) = &currentMesh.vert[inputIndices[i + 1]];
		currentMesh.face[faceCount].V(2) = &currentMesh.vert[inputIndices[i + 2]];
		faceCount++;
	}
	CleanMesh();
	ParseData();
}

void VCGMeshContainer::CleanMesh()
{
	//if (currentMesh.vn > 1000)
	//{
		//std::pair<int,int> comps = vcg::tri::Clean<VCGMesh>::RemoveSmallConnectedComponentsDiameter(currentMesh, 0.003f);
		//cDebug::DbgOut(_T("Removed components: "), comps.second);
		vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
		vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromFF(currentMesh);
		vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalizedPerFace(currentMesh);
		vcg::tri::UpdateBounding<VCGMesh>::Box(currentMesh);
		vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
		vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromNone(currentMesh);
		
		//vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalizedPerFace(currentMesh);

		
		
		
		//cDebug::DbgOut(_T("Removed small components: "), delInfo.first);
		//cDebug::DbgOut(_T("Removed small components: "), delInfo.second);
		

		
		//vcg::tri::Smooth<VCGMesh>::FaceNormalLaplacianFF(currentMesh);
		
		//float maniThresh = 1.0f;

		//while (vcg::tri::Clean<VCGMesh>::CountNonManifoldVertexFF(currentMesh) > 0)
		//{
			//cDebug::DbgOut(_T("here "), test);

			//int maniVert = vcg::tri::Clean<VCGMesh>::RemoveNonManifoldVertex(currentMesh);
			//cDebug::DbgOut(_T("Removed non manifold vertices: "), maniVert);
		//}
		
		
		

		int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(currentMesh);
		cDebug::DbgOut(_T("Removed duplicates: "), dup);
		int dupFa = vcg::tri::Clean<VCGMesh>::RemoveDuplicateFace(currentMesh);
		cDebug::DbgOut(_T("Removed duplicate faces: "), dupFa);
		int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(currentMesh);
		cDebug::DbgOut(_T("Removed unreferenced: "), unref);
		int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(currentMesh);
		cDebug::DbgOut(_T("Removed degenerate faces: "), deg);
		int zero = vcg::tri::Clean<VCGMesh>::RemoveZeroAreaFace(currentMesh);
		cDebug::DbgOut(_T("Removed zero area faces: "), zero);

		/*int minCC = 1;
		std::pair<int, int> delInfo = vcg::tri::Clean<VCGMesh>::RemoveSmallConnectedComponentsSize(currentMesh, minCC);
		cDebug::DbgOut(_T("Removed small components: "), delInfo.first);
		cDebug::DbgOut(_T("Removed small components: "), delInfo.second);*/
		//int holes = vcg::tri::Hole<VCGMesh>::EarCuttingFill<vcg::tri::TrivialEar<VCGMesh> >(currentMesh, 20, false);
		//cDebug::DbgOut(_T("Removed holes: "), deg);
		//vcg::tri::UpdateBounding<VCGMesh>::Box(currentMesh);
		/*vcg::tri::Clustering<VCGMesh, vcg::tri::AverageColorCell<VCGMesh> > Grid;
		Grid.DuplicateFaceParam = true;
		Grid.Init(currentMesh.bbox, 10, 10);
		Grid.AddMesh(currentMesh);
		Grid.ExtractMesh(currentMesh);*/
	//}
	vcg::tri::RequirePerVertexNormal(currentMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(currentMesh);
	
}

void VCGMeshContainer::ParseData(std::vector<float> inputVertices, std::vector<GLuint> inputIndices)
{
	//vertices = inputVertices;
	//indices = inputIndices;

	lowerBounds.x = 99999.0f;
	lowerBounds.y = 99999.0f;
	lowerBounds.z = 99999.0f;
	upperBounds.x = -99999.0f;
	upperBounds.y = -99999.0f;
	upperBounds.z = -99999.0f;

	//std::clock_t start;
	//double duration;

	//start = std::clock();

	for (int i = 0; i < inputVertices.size(); i+=6)
	{
		lowerBounds.x = min(inputVertices[i], lowerBounds.x);
		lowerBounds.y = min(inputVertices[i+1], lowerBounds.y);
		lowerBounds.z = min(inputVertices[i+2], lowerBounds.z);
		upperBounds.x = max(inputVertices[i], upperBounds.x);
		upperBounds.y = max(inputVertices[i+1], upperBounds.y);
		upperBounds.z = max(inputVertices[i+2], upperBounds.z);
		vertices.push_back(inputVertices[i]);
		vertices.push_back(inputVertices[i + 1]);
		vertices.push_back(inputVertices[i + 2]);
		vertices.push_back(inputVertices[i + 3]);
		vertices.push_back(inputVertices[i + 4]);
		vertices.push_back(inputVertices[i + 5]);
		//vertices.push_back(inputVertices[i + 6]);
	}

	vertNum = vertices.size() / 6;
	
	for (int i = 0; i < inputIndices.size(); i++)
	{
		indices.push_back(inputIndices[i]);
	}
	//bbox

	glm::vec4 color = colorCoding::IntToColor(colorCode);

	centerPoint.x = (lowerBounds.x + upperBounds.x) / 2.0f;
	centerPoint.y = (lowerBounds.y + upperBounds.y) / 2.0f;
	centerPoint.z = (lowerBounds.z + upperBounds.z) / 2.0f;
	originTransform = glm::translate(glm::mat4(1.0), -centerPoint);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxIndices.push_back(0);
	bBoxIndices.push_back(1);
	bBoxIndices.push_back(2);

	bBoxIndices.push_back(2);
	bBoxIndices.push_back(3);
	bBoxIndices.push_back(0);


	bBoxIndices.push_back(3);
	bBoxIndices.push_back(2);
	bBoxIndices.push_back(6);


	bBoxIndices.push_back(6);
	bBoxIndices.push_back(7);
	bBoxIndices.push_back(3);


	bBoxIndices.push_back(7);
	bBoxIndices.push_back(6);
	bBoxIndices.push_back(5);


	bBoxIndices.push_back(5);
	bBoxIndices.push_back(4);
	bBoxIndices.push_back(7);

	bBoxIndices.push_back(4);
	bBoxIndices.push_back(5);
	bBoxIndices.push_back(1);


	bBoxIndices.push_back(1);
	bBoxIndices.push_back(0);
	bBoxIndices.push_back(4);


	bBoxIndices.push_back(4);
	bBoxIndices.push_back(0);
	bBoxIndices.push_back(3);


	bBoxIndices.push_back(3);
	bBoxIndices.push_back(7);
	bBoxIndices.push_back(4);

	bBoxIndices.push_back(1);
	bBoxIndices.push_back(5);
	bBoxIndices.push_back(6);


	bBoxIndices.push_back(6);
	bBoxIndices.push_back(2);
	bBoxIndices.push_back(1);


	//bbox end
	selectTranslation = glm::vec3(0.0f, 0.0f, 0.0f);
	translation = glm::vec3(0.0f, 0.0f, 0.0f);


	//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	//cDebug::DbgOut(L"parse duration: ", duration);
}

void VCGMeshContainer::ParseData()
{
	vertices.clear();
	indices.clear();
	normals.clear();

	bBoxVertices.clear();
	bBoxIndices.clear();
	lowerBounds.x = 99999.0f;
	lowerBounds.y = 99999.0f;
	lowerBounds.z = 99999.0f;
	upperBounds.x = -99999.0f;
	upperBounds.y = -99999.0f;
	upperBounds.z = -99999.0f;

	std::clock_t start;
	double duration;
	glm::vec4 color = colorCoding::IntToColor(colorCode);
	start = std::clock();
	VCGMesh::VertexIterator vi;
	std::vector<int> VertexId((currentMesh).vert.size());
	//std::vector<float> colors;
	int numvert = 0;
	int curNormalIndex = 1;

	for (vi = (currentMesh).vert.begin(); vi != (currentMesh).vert.end(); ++vi) if (!(*vi).IsD())
	{
		VertexId[vi - (currentMesh).vert.begin()] = numvert;
		int dim = 0;
		while (dim < 3)
		{
			float tmpFloat = (*vi).P()[dim];
			vertices.push_back(tmpFloat);
			normals.push_back((*vi).N()[dim]);
			bBoxVertices.push_back(tmpFloat);
			lowerBounds[dim] = min(lowerBounds[dim], tmpFloat);
			upperBounds[dim] = max(upperBounds[dim], tmpFloat);

			//colors.push_back((*vi).C()[dim] / 255.0f);
			dim++;
		}
		dim = 0;
		bBoxVertices.push_back(color.r);
		bBoxVertices.push_back(color.g);
		bBoxVertices.push_back(color.b);
		while (dim < 3)
		{
			vertices.push_back((*vi).C()[dim] / 255.0f);
			dim++;
		}
		//vertices.push_back(1.0f);
		//colors.push_back(1.0f);

		numvert++;
	}
	vertNum = (currentMesh).vn;

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		for (int k = 0; k<(*fi).VN(); k++)
		{
			int vInd = -1;
			bBoxIndices.push_back(VertexId[vcg::tri::Index((currentMesh), (*fi).V(k))]);
			indices.push_back(VertexId[vcg::tri::Index((currentMesh), (*fi).V(k))]);//index of vertex per face
		}

	}



	//bbox

	

	centerPoint.x = (lowerBounds.x + upperBounds.x) / 2.0f;
	//centerPoint.y = (lowerBounds.y + upperBounds.y) / 2.0f;
	centerPoint.y = lowerBounds.y;
	//centerPoint.z = (lowerBounds.z + upperBounds.z) / 2.0f;
	centerPoint.z = lowerBounds.z;
	originTransform = glm::translate(glm::mat4(1.0), -centerPoint);

	/*bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(upperBounds.z + 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(lowerBounds.y - 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(upperBounds.x + 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxVertices.push_back(lowerBounds.x - 0.01f);
	bBoxVertices.push_back(upperBounds.y + 0.01f);
	bBoxVertices.push_back(lowerBounds.z - 0.01f);
	bBoxVertices.push_back(color.r);
	bBoxVertices.push_back(color.g);
	bBoxVertices.push_back(color.b);
	//bBoxVertices.push_back(color.a);

	bBoxIndices.push_back(0);
	bBoxIndices.push_back(1);
	bBoxIndices.push_back(2);

	bBoxIndices.push_back(2);
	bBoxIndices.push_back(3);
	bBoxIndices.push_back(0);


	bBoxIndices.push_back(3);
	bBoxIndices.push_back(2);
	bBoxIndices.push_back(6);


	bBoxIndices.push_back(6);
	bBoxIndices.push_back(7);
	bBoxIndices.push_back(3);


	bBoxIndices.push_back(7);
	bBoxIndices.push_back(6);
	bBoxIndices.push_back(5);


	bBoxIndices.push_back(5);
	bBoxIndices.push_back(4);
	bBoxIndices.push_back(7);

	bBoxIndices.push_back(4);
	bBoxIndices.push_back(5);
	bBoxIndices.push_back(1);


	bBoxIndices.push_back(1);
	bBoxIndices.push_back(0);
	bBoxIndices.push_back(4);


	bBoxIndices.push_back(4);
	bBoxIndices.push_back(0);
	bBoxIndices.push_back(3);


	bBoxIndices.push_back(3);
	bBoxIndices.push_back(7);
	bBoxIndices.push_back(4);

	bBoxIndices.push_back(1);
	bBoxIndices.push_back(5);
	bBoxIndices.push_back(6);


	bBoxIndices.push_back(6);
	bBoxIndices.push_back(2);
	bBoxIndices.push_back(1);*/

	
	//bbox end
	selectTranslation = glm::vec3(0.0f, 0.0f, 0.0f);
	translation = glm::vec3(0.0f, 0.0f, 0.0f);


	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	
	cDebug::DbgOut(L"parse duration: ", duration);
}

//not really necessary right now
void VCGMeshContainer::ConvertToVCG()
{
	/*currentMesh.Clear();
	VCGMesh newMesh;
	int size = vertices.size();
	VCGMesh::VertexPointer* ivp = new VCGMesh::VertexPointer[size];
	int vertNum = 0;
	for (int i = 0; i < vertices.size(); i += 7)
	{
		ivp[vertNum] = &*vcg::tri::Allocator<VCGMesh>::AddVertex(newMesh, VCGMesh::CoordType(vertices[i], vertices[i + 1], vertices[i + 2]),
			vcg::Color4b(vertices[i + 3], vertices[i + 4], vertices[i + 5], vertices[i + 6]));
		vertNum++;
	}
	for (int i = 0; i < indices.size(); i+=3)
	{
		vcg::tri::Allocator<VCGMesh>::AddFace(newMesh, ivp[indices[i]], ivp[indices[i + 1]], ivp[indices[i + 2]]);
	}*/

	//VCGMesh newMesh;
	//currentMesh.Clear();
	vcg::tri::Allocator<VCGMesh>::AddVertices(currentMesh, vertices.size() / 6);

	int vertCount = 0;
	for (int i = 0; i < vertices.size(); i += 6)
	{
		currentMesh.vert[vertCount].P() = vcg::Point3f(vertices[i], vertices[i + 1], vertices[i + 2]);
		currentMesh.vert[vertCount].C() = vcg::Color4b((int)(vertices[i + 3] * 255.0f), (int)(vertices[i + 4] * 255.0f), (int)(vertices[i + 5] * 255.0f), 255);
		vertCount++;

	}
	
	vcg::tri::Allocator<VCGMesh>::AddFaces(currentMesh, indices.size() / 3);

	int faceCount = 0;
	for (int i = 0; i<indices.size(); i+= 3){
		currentMesh.face[faceCount].V(0) = &currentMesh.vert[indices[i]];
		currentMesh.face[faceCount].V(1) = &currentMesh.vert[indices[i + 1]];
		currentMesh.face[faceCount].V(2) = &currentMesh.vert[indices[i + 2]];
		faceCount++;
	}

	/*vcg::tri::UpdateTopology<VCGMesh>::FaceFace(newMesh);
	vcg::tri::updateflags<vcgmesh>::faceborderfromff(newmesh);
	int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(newMesh);
	cDebug::DbgOut(_T("Removed duplicates: "), dup);
	int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(newMesh);
	cDebug::DbgOut(_T("Removed unreferenced: "), unref);
	int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(newMesh);
	cDebug::DbgOut(_T("Removed degenerate faces: "), deg);
	vcg::tri::RequirePerVertexNormal(newMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(newMesh);*/
	//vcg::tri::io::ExporterPLY<VCGMesh>::Save(newMesh, "saved.ply", vcg::tri::io::Mask::IOM_VERTCOLOR);
	//currentMesh = newMesh.
}

void VCGMeshContainer::ConvertToVCG(std::vector<float> inputVertices, std::vector<GLuint> inputIndices)
{
	/*currentMesh.Clear();
	VCGMesh newMesh;
	int size = vertices.size();
	VCGMesh::VertexPointer* ivp = new VCGMesh::VertexPointer[size];
	int vertNum = 0;
	for (int i = 0; i < vertices.size(); i += 7)
	{
	ivp[vertNum] = &*vcg::tri::Allocator<VCGMesh>::AddVertex(newMesh, VCGMesh::CoordType(vertices[i], vertices[i + 1], vertices[i + 2]),
	vcg::Color4b(vertices[i + 3], vertices[i + 4], vertices[i + 5], vertices[i + 6]));
	vertNum++;
	}
	for (int i = 0; i < indices.size(); i+=3)
	{
	vcg::tri::Allocator<VCGMesh>::AddFace(newMesh, ivp[indices[i]], ivp[indices[i + 1]], ivp[indices[i + 2]]);
	}*/

	//VCGMesh newMesh;
	vcg::tri::Allocator<VCGMesh>::AddVertices(currentMesh, inputVertices.size() / 6);

	int vertCount = 0;
	for (int i = 0; i < inputVertices.size(); i += 6)
	{
		currentMesh.vert[vertCount].P() = vcg::Point3f(inputVertices[i], inputVertices[i + 1], inputVertices[i + 2]);
		currentMesh.vert[vertCount].C() = vcg::Color4b((int)(inputVertices[i + 3] * 255.0f), (int)(inputVertices[i + 4] * 255.0f), (int)(inputVertices[i + 5] * 255.0f), 255);
		vertCount++;

	}

	vcg::tri::Allocator<VCGMesh>::AddFaces(currentMesh, inputIndices.size() / 3);

	int faceCount = 0;
	for (int i = 0; i<inputIndices.size(); i += 3){
		currentMesh.face[faceCount].V(0) = &currentMesh.vert[inputIndices[i]];
		currentMesh.face[faceCount].V(1) = &currentMesh.vert[inputIndices[i + 1]];
		currentMesh.face[faceCount].V(2) = &currentMesh.vert[inputIndices[i + 2]];
		faceCount++;
	}
	/*vcg::tri::UpdateTopology<VCGMesh>::FaceFace(newMesh);
	vcg::tri::updateflags<vcgmesh>::faceborderfromff(newmesh);
	int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(newMesh);
	cDebug::DbgOut(_T("Removed duplicates: "), dup);
	int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(newMesh);
	cDebug::DbgOut(_T("Removed unreferenced: "), unref);
	int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(newMesh);
	cDebug::DbgOut(_T("Removed degenerate faces: "), deg);
	vcg::tri::RequirePerVertexNormal(newMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(newMesh);*/
	//vcg::tri::io::ExporterPLY<VCGMesh>::Save(newMesh, "saved.ply", vcg::tri::io::Mask::IOM_VERTCOLOR);
	//currentMesh = newMesh.
}

void VCGMeshContainer::GenerateVAO()
{
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

	glGenVertexArrays(1, &bbVAO);
	glBindVertexArray(bbVAO);
	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bbIBO);
}

void VCGMeshContainer::GenerateBOs()
{
	glGenBuffers(1, &vbo);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &ibo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenBuffers(1, &bbVBO);

	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glBufferData(GL_ARRAY_BUFFER, bBoxVertices.size() * sizeof(float), &bBoxVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &bbIBO);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bbIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, bBoxIndices.size() * sizeof(GLuint), &bBoxIndices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void VCGMeshContainer::ToggleSelectedColor(bool flag)
{
	if (flag && !colorSelection)
	{
		for (int i = 0; i < vertices.size(); i += 6)
		{
			storedColors.push_back(vertices[i + 3]);
			vertices[i + 3] = min(vertices[i + 3] + 0.1f, 1.0f);
		}
		colorSelection = true;
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	else
	{
		if (colorSelection);
		{
			int cnt = 0;
			for (int i = 0; i < vertices.size(); i += 6)
			{
				vertices[i + 3] = storedColors[cnt];
				cnt++;
			}
			storedColors.clear();
			colorSelection = false;
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
	}
}

void VCGMeshContainer::Draw()
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix;
	if (!isSelected || previewSelection)
	{
		modelMatrix = glm::translate(glm::mat4(1.0), translation);
	}
	else if (!colorSelection)
	{
		modelMatrix = cursorTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
	}
	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(vao);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, (GLvoid*)0);
	glBindVertexArray(0);

	glUseProgram(0);
}

void VCGMeshContainer::DrawBB()
{
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix;
	if (!isSelected || previewSelection)
	{
		modelMatrix = glm::translate(glm::mat4(1.0), translation);
	}
	else if (!colorSelection)
	{
		modelMatrix = cursorTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
	}
	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);
	glBindVertexArray(bbVAO);
	glDrawElements(GL_TRIANGLES, bBoxIndices.size(), GL_UNSIGNED_INT, (GLvoid*)0);
	glBindVertexArray(0);

}

void VCGMeshContainer::AttachToCursor(glm::vec3 nearPoint, glm::vec3 farPoint, int distance)
{
	float carryDistance = (float)distance / 1000.0f;
	selectTranslation.x = nearPoint.x + (carryDistance / 10.0f) * (farPoint.x - nearPoint.x); //(0.50f / 10.0f) * (farPoint.x - nearPoint.x);
	selectTranslation.y = nearPoint.y + (carryDistance / 10.0f) * (farPoint.y - nearPoint.y); //- 1.5f; //(0.50f / 10.0f) * (farPoint.y - nearPoint.y);
	selectTranslation.z = nearPoint.z + (carryDistance / 10.0f) * (farPoint.z - nearPoint.z);//(0.50f / 10.0f) * (farPoint.z - nearPoint.z);

	cursorTranslation = glm::translate(glm::mat4(1.0), selectTranslation);
}

void VCGMeshContainer::TemporaryTranslateVerticesToPoint(glm::vec3 point)
{
	//glm::vec3 oldLowBounds;
	//oldLowBounds.z = lowerBounds.z;
	lowerBounds = glm::vec3(9999.0f, 9999.0f, 9999.0f);
	upperBounds = glm::vec3(-9999.0f, -9999.0f, -9999.0f);

	/*glm::mat4 originTransform = glm::translate(glm::mat4(1.0), -centerPoint);
	glm::mat4 xRotation = glm::mat4(1.0f);
	glm::mat4 yRotation = glm::mat4(1.0f);
	glm::mat4 zRotation = glm::mat4(1.0f);
	glm::mat4 scaleMatrix = glm::mat4(1.0f);
	if (angleX != 0)
	xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
	if (angleY != 0)
	yRotation = glm::rotate(glm::mat4(1.0), angleY, glm::vec3(0.0f, 1.0f, 0.0f));
	if (angleZ != 0)
	zRotation = glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f));
	if (scaleFactor != 1.0f)
	scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(scaleFactor, scaleFactor, scaleFactor));*/

	//originTransform = glm::translate(glm::mat4(1.0), -GetCenterPoint());
	float lowestZ = GetLowestZ();

	for (int i = 0; i < vertices.size(); i += 6)
	{
		glm::vec4 tmp = glm::vec4(vertices[i], vertices[i + 1], vertices[i + 2], 1.0f);


		tmp = (-storedTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) * tmp;

		vertices[i] = tmp.x;
		vertices[i + 1] = tmp.y;
		vertices[i + 2] = tmp.z + abs(centerPoint.z - lowestZ);

		upperBounds.x = max(upperBounds.x, vertices[i]);
		lowerBounds.x = min(lowerBounds.x, vertices[i]);
		upperBounds.y = max(upperBounds.y, vertices[i + 1]);
		lowerBounds.y = min(lowerBounds.y, vertices[i + 2]);
		upperBounds.z = max(upperBounds.z, vertices[i + 2]);
		lowerBounds.z = min(lowerBounds.z, vertices[i + 2]);
		/*vertices[i] = vertices[i] - centerPoint.x;
		vertices[i] = point.x - vertices[i];
		upperBounds.x = max(upperBounds.x, vertices[i]);
		lowerBounds.x = min(lowerBounds.x, vertices[i]);
		vertices[i + 1] = vertices[i + 1] - centerPoint.y;
		vertices[i+1] = point.y - vertices[i+1];
		upperBounds.y = max(upperBounds.y, vertices[i + 1]);
		lowerBounds.y = min(lowerBounds.y, vertices[i + 2]);
		vertices[i + 2] = vertices[i + 2] - centerPoint.z;// + (centerPoint.z - oldLowBounds.z));
		vertices[i + 2] = point.z - vertices[i + 2];
		upperBounds.z = max(upperBounds.z, vertices[i + 2]);
		lowerBounds.z = min(lowerBounds.z, vertices[i + 2]);*/
	}
	for (int i = 0; i < bBoxVertices.size(); i += 6)
	{
		glm::vec4 tmp = glm::vec4(bBoxVertices[i], bBoxVertices[i + 1], bBoxVertices[i + 2], 1.0f);


		tmp = (-storedTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) * tmp;

		bBoxVertices[i] = tmp.x;
		bBoxVertices[i + 1] = tmp.y;
		bBoxVertices[i + 2] = tmp.z + abs(centerPoint.z - lowestZ);

		/*bBoxVertices[i] = bBoxVertices[i] - centerPoint.x;
		bBoxVertices[i] = point.x - bBoxVertices[i];
		bBoxVertices[i + 1] = bBoxVertices[i + 1] - centerPoint.y;
		bBoxVertices[i + 1] = point.y - bBoxVertices[i + 1];
		bBoxVertices[i + 2] = bBoxVertices[i + 2] - centerPoint.z;// +(centerPoint.z - oldLowBounds.z));
		bBoxVertices[i + 2] = point.z - bBoxVertices[i + 2];*/
	}

	angleX = 0;
	angleY = 0;
	angleZ = 0;
	scaleFactor = 1.0f;
	xRotation = glm::mat4(1.0);
	yRotation = glm::mat4(1.0);
	zRotation = glm::mat4(1.0);
	scaleMatrix = glm::mat4(1.0);
	/*upperBounds.x = upperBounds.x - centerPoint.x;
	upperBounds.x = point.x - upperBounds.x;
	upperBounds.y = upperBounds.y - centerPoint.y;
	upperBounds.y = point.y - upperBounds.y;
	upperBounds.z = upperBounds.z - centerPoint.z;
	upperBounds.z = point.z - upperBounds.z;
	lowerBounds.x = lowerBounds.x - centerPoint.x;
	lowerBounds.x = point.x - lowerBounds.x;
	lowerBounds.y = lowerBounds.y - centerPoint.y;
	lowerBounds.y = point.y - lowerBounds.y;
	lowerBounds.z = lowerBounds.z - centerPoint.z;
	lowerBounds.z = point.z - lowerBounds.z;*/

	centerPoint.x = point.x;
	centerPoint.y = point.y;
	centerPoint.z = point.z + abs(centerPoint.z - lowestZ);
	//centerPoint = point;
	originTransform = glm::translate(glm::mat4(1.0), -centerPoint);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glBufferData(GL_ARRAY_BUFFER, bBoxVertices.size() * sizeof(float), &bBoxVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VCGMeshContainer::TranslateVerticesToPoint(glm::vec3 point)
{
	//glm::vec3 oldLowBounds;
	//oldLowBounds.z = lowerBounds.z;
	lowerBounds = glm::vec3(9999.0f, 9999.0f, 9999.0f);
	upperBounds = glm::vec3(-9999.0f, -9999.0f, -9999.0f);

	/*glm::mat4 originTransform = glm::translate(glm::mat4(1.0), -centerPoint);
	glm::mat4 xRotation = glm::mat4(1.0f);
	glm::mat4 yRotation = glm::mat4(1.0f);
	glm::mat4 zRotation = glm::mat4(1.0f);
	glm::mat4 scaleMatrix = glm::mat4(1.0f);
	if (angleX != 0)
		xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
	if (angleY != 0)
		yRotation = glm::rotate(glm::mat4(1.0), angleY, glm::vec3(0.0f, 1.0f, 0.0f));
	if (angleZ != 0)
		zRotation = glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f));
	if (scaleFactor != 1.0f)
		scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(scaleFactor, scaleFactor, scaleFactor));*/

	//originTransform = glm::translate(glm::mat4(1.0), -GetCenterPoint());
	float lowestZ = GetLowestZ();

	for (int i = 0; i < vertices.size(); i+=6)
	{
		glm::vec4 tmp = glm::vec4(vertices[i], vertices[i + 1], vertices[i + 2], 1.0f);
		

		glm::mat4 newTranslation = glm::translate(glm::mat4(1.0), point);
		storedTranslation = newTranslation;
		tmp = (newTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) * tmp;

		vertices[i] = tmp.x;
		vertices[i + 1] = tmp.y;
		vertices[i + 2] = tmp.z + abs(centerPoint.z - lowestZ);

		upperBounds.x = max(upperBounds.x, vertices[i]);
		lowerBounds.x = min(lowerBounds.x, vertices[i]);
		upperBounds.y = max(upperBounds.y, vertices[i + 1]);
		lowerBounds.y = min(lowerBounds.y, vertices[i + 2]);
		upperBounds.z = max(upperBounds.z, vertices[i + 2]);
		lowerBounds.z = min(lowerBounds.z, vertices[i + 2]);
		/*vertices[i] = vertices[i] - centerPoint.x;
		vertices[i] = point.x - vertices[i];
		upperBounds.x = max(upperBounds.x, vertices[i]);
		lowerBounds.x = min(lowerBounds.x, vertices[i]);
		vertices[i + 1] = vertices[i + 1] - centerPoint.y;
		vertices[i+1] = point.y - vertices[i+1];
		upperBounds.y = max(upperBounds.y, vertices[i + 1]);
		lowerBounds.y = min(lowerBounds.y, vertices[i + 2]);
		vertices[i + 2] = vertices[i + 2] - centerPoint.z;// + (centerPoint.z - oldLowBounds.z));
		vertices[i + 2] = point.z - vertices[i + 2];
		upperBounds.z = max(upperBounds.z, vertices[i + 2]);
		lowerBounds.z = min(lowerBounds.z, vertices[i + 2]);*/
	}
	for (int i = 0; i < bBoxVertices.size(); i += 6)
	{
		glm::vec4 tmp = glm::vec4(bBoxVertices[i], bBoxVertices[i + 1], bBoxVertices[i + 2], 1.0f);


		glm::mat4 newTranslation = glm::translate(glm::mat4(1.0), point);
		tmp = (newTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) * tmp;

		bBoxVertices[i] = tmp.x;
		bBoxVertices[i + 1] = tmp.y;
		bBoxVertices[i + 2] = tmp.z + abs(centerPoint.z - lowestZ);

		/*bBoxVertices[i] = bBoxVertices[i] - centerPoint.x;
		bBoxVertices[i] = point.x - bBoxVertices[i];
		bBoxVertices[i + 1] = bBoxVertices[i + 1] - centerPoint.y;
		bBoxVertices[i + 1] = point.y - bBoxVertices[i + 1];
		bBoxVertices[i + 2] = bBoxVertices[i + 2] - centerPoint.z;// +(centerPoint.z - oldLowBounds.z));
		bBoxVertices[i + 2] = point.z - bBoxVertices[i + 2];*/
	}

	angleX = 0;
	angleY = 0;
	angleZ = 0;
	scaleFactor = 1.0f;
	xRotation = glm::mat4(1.0); 
	yRotation = glm::mat4(1.0);
	zRotation = glm::mat4(1.0); 
	scaleMatrix = glm::mat4(1.0); 
	/*upperBounds.x = upperBounds.x - centerPoint.x;
	upperBounds.x = point.x - upperBounds.x;
	upperBounds.y = upperBounds.y - centerPoint.y;
	upperBounds.y = point.y - upperBounds.y;
	upperBounds.z = upperBounds.z - centerPoint.z;
	upperBounds.z = point.z - upperBounds.z;
	lowerBounds.x = lowerBounds.x - centerPoint.x;
	lowerBounds.x = point.x - lowerBounds.x;
	lowerBounds.y = lowerBounds.y - centerPoint.y;
	lowerBounds.y = point.y - lowerBounds.y;
	lowerBounds.z = lowerBounds.z - centerPoint.z;
	lowerBounds.z = point.z - lowerBounds.z;*/

	centerPoint.x = point.x;
	centerPoint.y = point.y;
	centerPoint.z = point.z + abs(centerPoint.z - lowestZ);
	//centerPoint = point;
	originTransform = glm::translate(glm::mat4(1.0), -centerPoint);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glBufferData(GL_ARRAY_BUFFER, bBoxVertices.size() * sizeof(float), &bBoxVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

glm::vec3 VCGMeshContainer::GetCenterPoint()
{
	
	if (!isSelected)
		return centerPoint;
	else
	{
		glm::vec4 tmpVec = (cursorTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) 
			* glm::vec4(centerPoint.x, centerPoint.y, centerPoint.z, 1.0f);
		return glm::vec3(tmpVec.x, tmpVec.y, tmpVec.z);
	}}

float VCGMeshContainer::GetLowestZ()
{
	float lowestZ = std::numeric_limits<float>::max();
	for (int i = 0; i <= vertices.size(); i += 6)
	{
		glm::vec4 currentPoint = glm::vec4(vertices[i], vertices[i + 1], vertices[i + 2], 1.0f);


		//currentPoint = (scaleMatrix * zRotation * yRotation * xRotation) * currentPoint;
		if (currentPoint.z <= lowestZ)
		{
			lowestZ = currentPoint.z;
		}
	}
	//cDebug::DbgOut(L"lowestZ: ", lowestZ);
	return lowestZ;
}

bool VCGMeshContainer::GetHitPoint(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output, bool snapToVertex)
{
	glm::vec3 rayDirection = glm::normalize(farPoint - nearPoint);
	
	/*float tMin = (lowerBounds.x - nearPoint.x) / rayDirection.x;
	float tMax = (upperBounds.x - nearPoint.x) / rayDirection.x;


	if (tMin > tMax)
	{
		float tmp = tMin;
		tMin = tMax;
		tMax = tmp;
	}

	float tyMin = (lowerBounds.y - nearPoint.y) / rayDirection.y;
	float tyMax = (upperBounds.y - nearPoint.y) / rayDirection.y;

	if (tyMin > tyMax)
	{
		float tmp = tyMin;
		tyMin = tyMax;
		tyMax = tyMin;
	}

	if ((tMin > tyMax) || (tyMin > tMax))
		return false;

	if (tyMin > tMin)
		tMin = tyMin;
	if (tyMax < tMax)
		tMax = tyMax;

	float tzMin = (lowerBounds.z - nearPoint.z) / rayDirection.z;
	float tzMax = (upperBounds.z - nearPoint.z) / rayDirection.z;

	if (tzMin < tzMax)
	{
		float tmp = tzMin;
		tzMin = tzMax;
		tzMax = tzMin;
	}

	if ((tMin > tzMax) || (tzMin > tMax))
		return false;

	if (tzMin > tMin)
		tMin = tzMin;

	if (tzMax < tMax)
		tMax = tzMax;*/

	glm::vec3 dirfrac;
	// r.dir is unit direction vector of ray
	dirfrac.x = 1.0f / rayDirection.x;
	dirfrac.y = 1.0f / rayDirection.y;
	dirfrac.z = 1.0f / rayDirection.z;
	// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
	// r.org is origin of ray
	float t;
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
		t = tmax;
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
	{
		t = tmax;
		return false;
	}

	t = tmin;

	//glm::vec3 vA = farPoint - nearPoint;
	//glm::vec3 distance(vA.x * t, vA.y * t, vA.z * t);
	//glm::vec3 rayPoint = nearPoint + distance;
	//output = rayPoint;
	//return true;

	if (snapToVertex)
	{

		glm::vec3 minPoint;
		glm::vec3 v = farPoint - nearPoint;
		double c2 = glm::dot(v, v);
		float minDistance = std::numeric_limits<float>::max();
		int index = -1;
		for (int i = 0; i < vertices.size(); i += 6)
		{
			glm::vec3 point(vertices[i], vertices[i + 1], vertices[i + 2]);
		
			glm::vec3 w = point - nearPoint;

			double c1 = glm::dot(w, v);
		
			double b = c1 / c2;
			glm::vec3 dd(v.x * b, v.y * b, v.z * b);
			glm::vec3 Pb = nearPoint + dd;
			float distance = glm::distance(point, Pb);
			if (distance < minDistance)
			{
				minDistance = distance;
				index = i*6;
				minPoint.x = point.x;
				minPoint.y = point.y;
				minPoint.z = point.z;
			}
		}
		output = minPoint;
	}
	else
	{
		float u, v, tX;
		for (int i = 0; i < indices.size(); i += 3)
		{
			glm::vec3 v0;
			v0.x = vertices[indices[i] * 6];
			v0.y = vertices[indices[i] * 6 + 1];
			v0.z = vertices[indices[i] * 6 + 2];
			glm::vec3 v1;
			v1.x = vertices[indices[i + 1] * 6];
			v1.y = vertices[indices[i + 1] * 6 + 1];
			v1.z = vertices[indices[i + 1] * 6 + 2];
			glm::vec3 v2;
			v2.x = vertices[indices[i + 2] * 6];
			v2.y = vertices[indices[i + 2] * 6 + 1];
			v2.z = vertices[indices[i + 2] * 6 + 2];
			glm::vec3 edge1 = v1 - v0;
			glm::vec3 edge2 = v2 - v0;
			glm::vec3 pVec = glm::cross(rayDirection, edge2);
			float det = glm::dot(edge1, pVec);
			if (det > -0.00001f && det < 0.00001f)
				continue;
			float invDet = 1 / det;
			glm::vec3 tVec = nearPoint - v0;
			u = glm::dot(tVec, pVec) * invDet;
			if (u < 0.0f || u > 1.0f)
				continue;
			glm::vec3 qVec = glm::cross(tVec, edge1);
			v = glm::dot(rayDirection, qVec) * invDet;
			if (v < 0.0f || u + v > 1.0f)
				continue;
			tX = glm::dot(edge2, qVec) * invDet;
		}
		glm::vec3 minPoint = nearPoint + rayDirection * tX;
		output = minPoint;
	}
	return true;

	/*triangle intersection
	int cnnt = 0;
	float u, v, tX;
	for (int i = 0; i < indices.size(); i += 3)
	{
		glm::vec3 v0;
		v0.x = vertices[indices[i]*6];
		v0.y = vertices[indices[i]*6+1];
		v0.z = vertices[indices[i]*6+2];
		glm::vec3 v1;
		v1.x = vertices[indices[i+1]*6];
		v1.y = vertices[indices[i + 1] * 6 + 1];
		v1.z = vertices[indices[i + 1] * 6 + 2];
		glm::vec3 v2;
		v2.x = vertices[indices[i + 2] * 6];
		v2.y = vertices[indices[i + 2] * 6 + 1];
		v2.z = vertices[indices[i + 2] * 6 + 2];
		glm::vec3 edge1 = v1 - v0;
		glm::vec3 edge2 = v2 - v0;
		glm::vec3 pVec = glm::cross(rayDirection, edge2);
		float det = glm::dot(edge1, pVec);
		if (det > -0.00001f && det < 0.00001f)
			continue;
		float invDet = 1 / det;
		glm::vec3 tVec = nearPoint - v0;
		u = glm::dot(tVec, pVec) * invDet;
		if (u < 0.0f || u > 1.0f)
			continue;
		glm::vec3 qVec = glm::cross(tVec, edge1);
		v = glm::dot(rayDirection, qVec) * invDet;
		if (v < 0.0f || u + v > 1.0f)
			continue;
		tX = glm::dot(edge2, qVec) * invDet;

		if (tX > 0.00001f)
		{
			cnnt++;
			break;
		}
	}
	cDebug::DbgOut(L"Count: ", cnnt);
	glm::vec3 minPoint = nearPoint + rayDirection * tX;
	output = minPoint;
	return true;*/
	
}

bool VCGMeshContainer::CheckCollision(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output)
{
	glm::vec3 realCenter;

	realCenter.x = (upperBounds.x + lowerBounds.x) / 2.0f;
	realCenter.y = (upperBounds.y + lowerBounds.y) / 2.0f;
	realCenter.z = (upperBounds.z + lowerBounds.z) / 2.0f;
	glm::vec3 nearToCenter = realCenter - nearPoint;

	glm::vec3 rayDirection = glm::normalize(farPoint - nearPoint);

	float rayLength = glm::distance(nearPoint, farPoint);

	float dotProd = glm::dot(nearToCenter, rayDirection);

	glm::vec3 closestPoint;

	closestPoint = nearPoint + rayDirection*dotProd;

	bool found = true;

	if (closestPoint.x < lowerBounds.x || closestPoint.x > upperBounds.x)
		found = false;
	else if (closestPoint.y < lowerBounds.y || closestPoint.y > upperBounds.y)
		found = false;
	else if (closestPoint.z < lowerBounds.z || closestPoint.z > upperBounds.z)
		found = false;

	if (found)
	{
		glm::vec3 minPoint;
		float minDistance = std::numeric_limits<float>::max();
		for (int i = 0; i < vertices.size(); i+=6)
		{
			glm::vec3 point(vertices[i], vertices[i + 1], vertices[i + 2]);
			glm::vec3 v = farPoint - nearPoint;
			glm::vec3 w = point - nearPoint;

			double c1 = glm::dot(w, v);
			double c2 = glm::dot(v, v);
			double b = c1 / c2;
			glm::vec3 dd(v.x * b, v.y * b, v.z * b);
			glm::vec3 Pb = nearPoint + dd;
			float distance = glm::distance(point, Pb);
			if (distance < minDistance)
			{
				minDistance = distance;
				minPoint.x = point.x;
				minPoint.y = point.y;
				minPoint.z = point.z;
			}
		}
		output = minPoint;
		return true;
	}
	return false;
}

void VCGMeshContainer::ClearMesh()
{
	currentMesh.Clear();
	glDeleteBuffers(1, &vbo);
	glDeleteBuffers(1, &bbVBO);
	glDeleteBuffers(1, &bbIBO);
	glDeleteBuffers(1, &ibo);
	glDeleteVertexArrays(1, &vao);
	glDeleteVertexArrays(1, &bbVAO);
	bBoxVertices.clear();
	bBoxIndices.clear();
	vertices.clear();
	indices.clear();
	vertNum = 0;
}

void VCGMeshContainer::SetScale(bool positive)
{
	if (positive)
		scaleFactor += 0.1f;
	else
		scaleFactor -= 0.1f;

	scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(scaleFactor, scaleFactor, scaleFactor));
}

void VCGMeshContainer::SetAngleX(bool positive)
{
	if (positive)
		angleX+=20;
	else
		angleX-=20;

	xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
}

void VCGMeshContainer::SetAngleY(bool positive)
{
	if (positive)
		angleY+=20;
	else
		angleY-=20;

	yRotation = glm::rotate(glm::mat4(1.0), angleY, glm::vec3(0.0f, 1.0f, 0.0f));
}

void VCGMeshContainer::SetAngleZ(bool positive)
{
	if (positive)
		angleZ += 20;
	else
		angleZ -= 20;
	
	zRotation = glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f));
}

void VCGMeshContainer::SetColorCode(int value)
{
	colorCode = value;
}

void VCGMeshContainer::SetTranslation(glm::vec3 trans)
{
	translation = trans;
}

void VCGMeshContainer::SetSelected(bool val)
{
	isSelected = val;
}

int VCGMeshContainer::RemoveSmallComponents(int compSize)
{
	vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
	vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);


	//vcg::tri::UpdateTopology<VCGMesh>::
	if (vcg::tri::HasFFAdjacency(currentMesh))
		cDebug::DbgOut(_T("adj"), 0);
	//vcg::tri::UpdateTopology<VCGMesh>::VertexEdge(currentMesh);
	//vcg::tri::UpdateTopology<VCGMesh>::PEdgeTex(currentMesh);

	//this throws exception
	std::pair<int, int> compCnt = vcg::tri::Clean<VCGMesh>::RemoveSmallConnectedComponentsSize(currentMesh, compSize);

	cDebug::DbgOut(_T("Removed components:"), compCnt.second);
	//if (currentMesh.vn > 1000)
	//{
		int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(currentMesh);
		cDebug::DbgOut(_T("Removed duplicates:"), dup);
		int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(currentMesh);
		cDebug::DbgOut(_T("Removed unreferenced:"), unref);
		int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(currentMesh);
		cDebug::DbgOut(_T("Removed degenerate faces:"), deg);
	//}
	vcg::tri::RequirePerVertexNormal(currentMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(currentMesh);
	

	

	return compCnt.second;
}

void VCGMeshContainer::UpdateBuffers()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, bbVBO);
	glBufferData(GL_ARRAY_BUFFER, bBoxVertices.size() * sizeof(float), &bBoxVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bbIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, bBoxIndices.size() * sizeof(GLuint), &bBoxIndices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

int VCGMeshContainer::FillHoles(int holeSize)
{
	
	vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
	vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
	
	//cDebug::DbgOut(L"Closed holes2: ", 1);
	int holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingIntersectionFill<vcg::tri::SelfIntersectionEar<VCGMesh> >(currentMesh, holeSize, false);
	
	cDebug::DbgOut(L"Closed holes: ", holeCnt);
	//holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingFill<vcg::tri::MinimumWeightEar< VCGMesh> >(currentMesh, 10000, false);
	//cDebug::DbgOut(L"Closed holes: ", holeCnt);
	//vcg::tri::Hole<VCGMesh>::EarCuttingFill<vcg::tri::TrivialEar<VCGMesh> >(currentMesh, holeSize, false);
	//if (currentMesh.vn > 1000)
	//{
	int dup = vcg::tri::Clean<VCGMesh>::RemoveDuplicateVertex(currentMesh);
	cDebug::DbgOut(_T("Removed duplicates:"), dup);
	int unref = vcg::tri::Clean<VCGMesh>::RemoveUnreferencedVertex(currentMesh);
	cDebug::DbgOut(_T("Removed unreferenced:"), unref);
	int deg = vcg::tri::Clean<VCGMesh>::RemoveDegenerateFace(currentMesh);
	cDebug::DbgOut(_T("Removed degenerate faces:"), deg);
	//}
	vcg::tri::RequirePerVertexNormal(currentMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(currentMesh);
	RemoveNonManifoldFace();
	LaplacianSmooth(3);
	return holeCnt;
}

void VCGMeshContainer::ResetSelectedTransformation()
{
	scaleFactor = 1.0f;
	angleX = 0;
	angleY = 0;
	angleZ = 0;
	xRotation = glm::mat4(1.0f);
	yRotation = glm::mat4(1.0f);
	zRotation = glm::mat4(1.0f);
	scaleMatrix = glm::mat4(1.0f);
}

int VCGMeshContainer::GetColorCode()
{
	return colorCode;
}

void VCGMeshContainer::TogglePreviewSelection(bool flag)
{
	previewSelection = flag;
}

glm::vec3 VCGMeshContainer::GetUpperBounds()
{
	return upperBounds;
}

glm::vec3 VCGMeshContainer::GetLowerBounds()
{
	return lowerBounds;
}

std::vector<float> VCGMeshContainer::GetVertices()
{
	return vertices;
}

std::vector<float> VCGMeshContainer::GetNormals()
{
	return normals;
}

std::vector<GLuint> VCGMeshContainer::GetIndices()
{
	return indices;
}

int VCGMeshContainer::GetNumberOfVertices()
{
	return vertNum;
}

int VCGMeshContainer::GetNumberOfIndices()
{
	return indices.size();
}

void VCGMeshContainer::CleanAndParse(std::vector<float> &startingVertices, std::vector<GLuint> &startingIndices, std::vector<float> &startingNormals)
{
	startingVertices.clear();
	startingIndices.clear();
	startingNormals.clear();
	CleanMesh();
	std::clock_t start;
	double duration;

	start = std::clock();
	VCGMesh::VertexIterator vi;
	std::vector<int> VertexId((currentMesh).vert.size());
	//std::vector<float> colors;
	int numvert = 0;
	int curNormalIndex = 1;

	for (vi = (currentMesh).vert.begin(); vi != (currentMesh).vert.end(); ++vi) if (!(*vi).IsD())
	{
		VertexId[vi - (currentMesh).vert.begin()] = numvert;
		int dim = 0;
		while (dim < 3)
		{
			float tmpFloat = (*vi).P()[dim];
			startingVertices.push_back(tmpFloat);
			startingNormals.push_back((*vi).N()[dim]);

			//colors.push_back((*vi).C()[dim] / 255.0f);
			dim++;
		}
		dim = 0;
		while (dim < 3)
		{
			startingVertices.push_back((*vi).C()[dim] / 255.0f);
			dim++;
		}
		//startingVertices.push_back(1.0f);
		//colors.push_back(1.0f);

		numvert++;
	}

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		for (int k = 0; k<(*fi).VN(); k++)
		{
			int vInd = -1;

			startingIndices.push_back(VertexId[vcg::tri::Index((currentMesh), (*fi).V(k))]);//index of vertex per face
		}
	}

	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	cDebug::DbgOut(L"Parse duration: ", duration);

}

void CombineAndExport()
{
	VCGMesh combinedMesh;
	vcg::tri::Allocator<VCGMesh>::AddVertices(combinedMesh, numberOfVertices);

	std::vector<int> vertOffset;
	vertOffset.push_back(0);
	int meshCnt = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		std::vector<float> vertices = (*mI)->GetVertices();

		int vertCount = 0;
		for (int i = 0; i < vertices.size(); i += 6)
		{
			combinedMesh.vert[vertOffset[meshCnt] + vertCount].P() = vcg::Point3f(vertices[i], vertices[i + 1], vertices[i + 2]);
			combinedMesh.vert[vertOffset[meshCnt] + vertCount].C() = vcg::Color4b((int)(vertices[i + 3] * 255.0f), (int)(vertices[i + 4] * 255.0f), (int)(vertices[i + 5] * 255.0f), 255);
			vertCount++;
		}
		vertOffset.push_back(vertOffset[meshCnt] + vertCount);
		meshCnt++;
	}

	std::vector<int> faceOffset;
	faceOffset.push_back(0);
	meshCnt = 0;
	vcg::tri::Allocator<VCGMesh>::AddFaces(combinedMesh, numberOfFaces);
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		std::vector<GLuint> indices = (*mI)->GetIndices();
		int faceCount = 0;
		for (int i = 0; i < indices.size(); i += 3){
			combinedMesh.face[faceOffset[meshCnt] + faceCount].V(0) = &combinedMesh.vert[vertOffset[meshCnt] + indices[i]];
			combinedMesh.face[faceOffset[meshCnt] + faceCount].V(1) = &combinedMesh.vert[vertOffset[meshCnt] + indices[i + 1]];
			combinedMesh.face[faceOffset[meshCnt] + faceCount].V(2) = &combinedMesh.vert[vertOffset[meshCnt] + indices[i + 2]];
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

void CleanAndParse(const char* fileName, std::vector<float> &startingVertices, std::vector<GLuint> &startingIndices, std::vector<float> &startingNormals)
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
		int dim = 0;
		while (dim < 3)
		{
			float tmpFloat = (*vi).P()[dim];
			startingVertices.push_back(tmpFloat);
			startingNormals.push_back((*vi).N()[dim]);

			//colors.push_back((*vi).C()[dim] / 255.0f);
			dim++;
		}
		dim = 0;
		while (dim < 3)
		{
			startingVertices.push_back((*vi).C()[dim] / 255.0f);
			dim++;
		}
		//startingVertices.push_back(1.0f);
		//colors.push_back(1.0f);

		numvert++;
	}

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (mesh).face.begin(); fi != (mesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		for (int k = 0; k<(*fi).VN(); k++)
		{
			int vInd = -1;

			startingIndices.push_back(VertexId[vcg::tri::Index((mesh), (*fi).V(k))]);//index of vertex per face
		}
	}

	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	cDebug::DbgOut(L"Parse duration: ", duration);

}