#include "colorCoding.h"
#include "VcgMeshContainer.h"
#include "OpenGLShaders.h"
#include "InteractiveFusion.h"
#include "OpenGLCamera.h"
#include "SelectionHelper.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/clustering.h>
#include<wrap/io_trimesh/import.h>
#include<wrap/io_trimesh/export.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/complex/algorithms/update/normal.h>

VCGMeshContainer::VCGMeshContainer() {
	orientation.reserve(3);
	orientation.push_back(0);
	orientation.push_back(0);
	orientation.push_back(0);
	snapOrientation.reserve(3);
	snapOrientation.push_back(0);
	snapOrientation.push_back(0);
	snapOrientation.push_back(0);

	if (openGLWin.GetDeviceClass() == IF_DEVICE_PC)
	{
		rotateBy = 20;
		scaleBy = 0.1f;
	}
	else if (openGLWin.GetDeviceClass() == IF_DEVICE_TABLET)
	{
		rotateBy = 5;
		scaleBy = 0.025f;
	}
}

VCGMeshContainer::~VCGMeshContainer() { }

void VCGMeshContainer::Load2DMesh(const char* filename)
{
	vcg::tri::io::ImporterPLY<VCGMesh>::Open(currentMesh, filename);
	is2D = true;
}

void VCGMeshContainer::LoadMesh(const char* filename)
{
	statusMsg = L"Loading mesh data";
	vcg::tri::io::ImporterPLY<VCGMesh>::Open(currentMesh, filename);

	/*
	vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);
	vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromFF(currentMesh);
	vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalizedPerFace(currentMesh);
	vcg::tri::UpdateBounding<VCGMesh>::Box(currentMesh);
	vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
	vcg::tri::UpdateFlags<VCGMesh>::FaceBorderFromNone(currentMesh);*/
	statusMsg = L"Simplifying mesh";
	//RemoveNonManifoldFace();
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);

	float threshold = 0.0005f;
	int total = MergeCloseVertices(threshold);

	//cDebug::DbgOut(_T("Merged close vertices: "), total);
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Merged close vertices in ", elapsedTime);

	//int stepSmoothNum = 3;
	//size_t cnt = vcg::tri::UpdateSelection<VCGMesh>::VertexFromFaceStrict(currentMesh);
	//vcg::tri::Smooth<VCGMesh>::VertexCoordLaplacian(currentMesh, stepSmoothNum, cnt>0);
	statusMsg = L"Smoothing mesh";
	LaplacianSmooth(3);
	//UnsharpColor(0.3f);
	statusMsg = L"Cleaning mesh";
	CleanMesh();
	statusMsg = L"Removing small components";
	RemoveSmallComponents(currentMesh.vn / 100);
	CleanMesh();
	statusMsg = L"Parsing data for interaction";
	ParseData();

	isLoaded = true;
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

	int count;
	while (count = vcg::tri::Clean<VCGMesh>::CountNonManifoldVertexFF(currentMesh) > 0)
	{
	
		int total = vcg::tri::Clean<VCGMesh>::SplitNonManifoldVertex(currentMesh, 0);
		cDebug::DbgOut(_T("Removed non manifold vertices: "), total);
		cDebug::DbgOut(_T("Still remaining: "), count);
	}
	
	//int testV = vcg::tri::Clean<VCGMesh>::RemoveNonManifoldVertex(currentMesh);
	cDebug::DbgOut(_T("Removed non manifold faces: "), test);
	int fCount = vcg::tri::Clean<VCGMesh>::CountNonManifoldEdgeFF(currentMesh);
	cDebug::DbgOut(_T("Non manifold faces remaining: "), fCount);
	count = vcg::tri::Clean<VCGMesh>::CountNonManifoldVertexFF(currentMesh);
	cDebug::DbgOut(_T("Non manifold vertices remaining: "), count);
}

void VCGMeshContainer::LaplacianSmooth(int step)
{
	vcg::tri::Smooth<VCGMesh>::VertexCoordLaplacian(currentMesh, step, false, false);
}

int VCGMeshContainer::MergeCloseVertices(float threshold)
{
	return vcg::tri::Clean<VCGMesh>::MergeCloseVertex(currentMesh, threshold);
}

void VCGMeshContainer::HighlightObjects(std::vector<int> objTriangles, ColorIF color, bool additive)
{
	if (verticesWithHighlights.size() == 0)
		verticesWithHighlights.insert(verticesWithHighlights.begin(), vertices.begin(), vertices.end());
	for (int i = 0; i < objTriangles.size(); i++)
	{
		if (additive)
		{ 
			verticesWithHighlights[objTriangles[i]].r = min(verticesWithHighlights[objTriangles[i]].r + color.r, 1.0f);
			verticesWithHighlights[objTriangles[i]].g = min(verticesWithHighlights[objTriangles[i]].g + color.g, 1.0f);
			verticesWithHighlights[objTriangles[i]].b = min(verticesWithHighlights[objTriangles[i]].b + color.b, 1.0f);
		}
		else
		{ 
			verticesWithHighlights[objTriangles[i]].r = color.r;
			verticesWithHighlights[objTriangles[i]].g = color.g;
			verticesWithHighlights[objTriangles[i]].b = color.b;
		}
		//verticesWithHighlights[objTriangles[i]].r = min(verticesWithHighlights[objTriangles[i]].r + color.r, 1.0f);
		//verticesWithHighlights[objTriangles[i]].g = min(verticesWithHighlights[objTriangles[i]].g + color.g, 1.0f);
		//verticesWithHighlights[objTriangles[i]].b = min(verticesWithHighlights[objTriangles[i]].b + color.b, 1.0f);
	}
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, verticesWithHighlights.size() * sizeof(Vertex), &verticesWithHighlights[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VCGMeshContainer::ResetHighlights()
{
	verticesWithHighlights.clear();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VCGMeshContainer::SetVerticesInPlane(glm::vec3 planeCenter)
{
	indicesOnPlane.clear();

	for (int i = 0; i < vertices.size(); i++)
	{
		if (abs(vertices[i].x - planeCenter.x) <= 0.0050f)
			indicesOnPlane.push_back(i);
		//verticesWithHighlights[objTriangles[i]].r = min(verticesWithHighlights[objTriangles[i]].r + color.r, 1.0f);
		//verticesWithHighlights[objTriangles[i]].g = min(verticesWithHighlights[objTriangles[i]].g + color.g, 1.0f);
		//verticesWithHighlights[objTriangles[i]].b = min(verticesWithHighlights[objTriangles[i]].b + color.b, 1.0f);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesOnPlane.size() * sizeof(GLuint), &indicesOnPlane[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void VCGMeshContainer::ShowVerticesInPlane(bool flag)
{
	showPlaneVertices = flag;
}

void VCGMeshContainer::LoadMesh(std::vector<Vertex> inputVertices, std::vector<Triangle> inputIndices)
{
	currentMesh.Clear();
	vertices.clear();
	indices.clear();
	vcg::tri::Allocator<VCGMesh>::AddVertices(currentMesh, inputVertices.size());

	for (int i = 0; i < vertices.size(); i += 1)
	{
		currentMesh.vert[i].P() = vcg::Point3f(inputVertices[i].x, inputVertices[i].y, inputVertices[i].z);
		currentMesh.vert[i].C() = vcg::Color4b((int)(inputVertices[i].r * 255.0f), (int)(inputVertices[i].g * 255.0f), (int)(inputVertices[i].b * 255.0f), 255);
		currentMesh.vert[i].N() = vcg::Point3f(inputVertices[i].normal_x, inputVertices[i].normal_y, inputVertices[i].normal_z);
	}

	vcg::tri::Allocator<VCGMesh>::AddFaces(currentMesh, inputIndices.size());

	for (int i = 0; i<inputIndices.size(); i += 1){
		currentMesh.face[i].V(0) = &currentMesh.vert[inputIndices[i].v1];
		currentMesh.face[i].V(1) = &currentMesh.vert[inputIndices[i].v2];
		currentMesh.face[i].V(2) = &currentMesh.vert[inputIndices[i].v3];
	}
	CleanMesh();
	ParseData();
}

void VCGMeshContainer::UnsharpColor(float alpha)
{
	float alphaorig = 1.0f;
	int smoothIter = 5;

	vcg::tri::Allocator<VCGMesh>::CompactVertexVector(currentMesh);
	vector<vcg::Color4f> colorOrig(currentMesh.vn);
	for (int i = 0; i<currentMesh.vn; ++i)
		colorOrig[i].Import(currentMesh.vert[i].C());

	vcg::tri::Smooth<VCGMesh>::VertexColorLaplacian(currentMesh, smoothIter);
	for (int i = 0; i<currentMesh.vn; ++i)
	{
		vcg::Color4f colorDelta = colorOrig[i] - vcg::Color4f::Construct(currentMesh.vert[i].C());
		vcg::Color4f newCol = colorOrig[i] * alphaorig + colorDelta*alpha;       // Unsharp formula 
		vcg::Clamp(newCol); // Clamp everything in the 0..1 range
		currentMesh.vert[i].C().Import(newCol);

	}
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

void VCGMeshContainer::ParseData()
{
	vertices.clear();
	indices.clear();

	lowerBounds.x = 99999.0f;
	lowerBounds.y = 99999.0f;
	lowerBounds.z = 99999.0f;
	upperBounds.x = -99999.0f;
	upperBounds.y = -99999.0f;
	upperBounds.z = -99999.0f;

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

		lowerBounds[0] = min(lowerBounds[0], vertex.x);
		lowerBounds[1] = min(lowerBounds[1], vertex.y);
		lowerBounds[2] = min(lowerBounds[2], vertex.z);
		upperBounds[0] = max(upperBounds[0], vertex.x);
		upperBounds[1] = max(upperBounds[1], vertex.y);
		upperBounds[2] = max(upperBounds[2], vertex.z);

		numvert++;
	}

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		Triangle triangle;
		triangle.v1 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(0))];
		triangle.v2 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(1))];
		triangle.v3 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(2))];

		indices.push_back(triangle);
	}



	//bbox

	centerPoint.x = (lowerBounds.x + upperBounds.x) / 2.0f;
	centerPoint.y = (lowerBounds.y + upperBounds.y) / 2.0f;
	centerPoint.z = (lowerBounds.z + upperBounds.z) / 2.0f;

	originTransform = glm::translate(glm::mat4(1.0), -centerPoint);

	offSet.x = (centerPoint.x - lowerBounds.x);
	offSet.y = (centerPoint.y - lowerBounds.y);
	offSet.z = (centerPoint.z - lowerBounds.z);

	selectTranslation = glm::vec3(0.0f, 0.0f, 0.0f);
	translation = glm::vec3(0.0f, 0.0f, 0.0f);

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
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	cDebug::DbgOut(L"parse duration: ", duration);
}

//not really necessary right now
void VCGMeshContainer::ConvertToVCG()
{
	currentMesh.Clear();
	vcg::tri::Allocator<VCGMesh>::AddVertices(currentMesh, vertices.size());

	for (int i = 0; i < vertices.size(); i += 1)
	{
		currentMesh.vert[i].P() = vcg::Point3f(vertices[i].x, vertices[i].y, vertices[i].z);
		currentMesh.vert[i].C() = vcg::Color4b((int)(vertices[i].r * 255.0f), (int)(vertices[i].g * 255.0f), (int)(vertices[i].b * 255.0f), 255);
		currentMesh.vert[i].N() = vcg::Point3f(vertices[i].normal_x, vertices[i].normal_y, vertices[i].normal_z);
	}

	vcg::tri::Allocator<VCGMesh>::AddFaces(currentMesh, indices.size());

	for (int i = 0; i<indices.size(); i += 1){
		currentMesh.face[i].V(0) = &currentMesh.vert[indices[i].v1];
		currentMesh.face[i].V(1) = &currentMesh.vert[indices[i].v2];
		currentMesh.face[i].V(2) = &currentMesh.vert[indices[i].v3];
	}
}

void VCGMeshContainer::ConvertToVCG(std::vector<Vertex> inputVertices, std::vector<Triangle> inputIndices)
{
	vcg::tri::Allocator<VCGMesh>::AddVertices(currentMesh, inputVertices.size());

	for (int i = 0; i < inputVertices.size(); i += 1)
	{
		currentMesh.vert[i].P() = vcg::Point3f(inputVertices[i].x, inputVertices[i].y, inputVertices[i].z);
		currentMesh.vert[i].C() = vcg::Color4b((int)(inputVertices[i].r * 255.0f), (int)(inputVertices[i].g * 255.0f), (int)
			(inputVertices[i].b * 255.0f), 255);
		currentMesh.vert[i].N() = vcg::Point3f(inputVertices[i].normal_x, inputVertices[i].normal_y, inputVertices[i].normal_z);
	}

	

	vcg::tri::Allocator<VCGMesh>::AddFaces(currentMesh, inputIndices.size());

	for (int i = 0; i<inputIndices.size(); i += 1){
		currentMesh.face[i].V(0) = &currentMesh.vert[inputIndices[i].v1];
		currentMesh.face[i].V(1) = &currentMesh.vert[inputIndices[i].v2];
		currentMesh.face[i].V(2) = &currentMesh.vert[inputIndices[i].v3];
	}

}

void VCGMeshContainer::GenerateBOs()
{
	glGenBuffers(1, &vbo);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &ibo);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(Triangle), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenBuffers(1, &planeIBO);
}

void VCGMeshContainer::GenerateVAO()
{
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 3));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 6));
}

void VCGMeshContainer::ToggleSelectedColor(bool flag)
{
	if (flag && !colorSelection)
	{
		for (int i = 0; i < vertices.size(); i += 1)
		{
			storedColors.push_back(vertices[i].r);
			vertices[i].r = min(vertices[i].r + 0.1f, 1.0f);
		}
		colorSelection = true;
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	else
	{
		if (colorSelection);
		{
			int cnt = 0;
			for (int i = 0; i < vertices.size(); i += 1)
			{
				vertices[i].r = storedColors[cnt];
				cnt++;
			}
			storedColors.clear();
			colorSelection = false;
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
	}
}

void VCGMeshContainer::Draw()
{
	if (!isDeleted)
	{ 
		shaderColor.UseProgram();
		shaderColor.SetUniform("colorPicking", false);
		shaderColor.SetUniform("alpha", 1.0f);
		glm::mat4 modelMatrix;
		if (is2D)
		{
			shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
			shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));
			float hRatio = (float)openGLWin.glControl.GetViewportHeight() / (float)openGLWin.glControl.GetViewportWidth();
			glm::mat4 twoDScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(hRatio, 1.0f, 1.0f));

			modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.95f - hRatio*0.15f, -0.65f, 0.0f)) * twoDScaleMatrix;
		}
		else
		{
			shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
			shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	
			/*if (colorSelection && glSelector.GetManipulationMode() != MANIPULATION_NONE)
			{
				modelMatrix = glm::translate(glm::mat4(1.0), centerPoint) * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
			}*/
			if (!isSelected || previewSelection)
			{
				//if (previewSelection)
				//modelMatrix = snapTransform * glm::translate(glm::mat4(1.0), translation);
				//else
				modelMatrix = glm::translate(glm::mat4(1.0), translation);
			}
			else if (!colorSelection)
			{
				//if (isOverTrash)
				//	modelMatrix = cursorTranslation * trashScaleMatrix * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
				//else
					modelMatrix = cursorTranslation * selectScaleMatrix * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
			}
		}
		shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

		//if (isOverTrash)
			//glDisable(GL_DEPTH_TEST);

		
		glBindVertexArray(vao);
		
		if (showPlaneVertices)
		{ 
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeIBO);
			glDrawElements(GL_POINTS, indicesOnPlane.size(), GL_UNSIGNED_INT, (GLvoid*)0);
		}
		else
		{
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
			glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, (GLvoid*)0);
		}
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);
		//if (isOverTrash)
			//glEnable(GL_DEPTH_TEST);
	}
}

void VCGMeshContainer::DrawBB()
{
	if (!isDeleted)
	{
		shaderColor.UseProgram();
		shaderColor.SetUniform("colorPicking", true);
		shaderColor.SetUniform("alpha", 1.0f);
		shaderColor.SetUniform("pickColor", colorCoding::IntToColor(colorCode));
		glm::mat4 modelMatrix;
		if (is2D)
		{
			shaderColor.SetUniform("matrices.projectionMatrix", glm::mat4(1.0f));
			shaderColor.SetUniform("matrices.viewMatrix", glm::mat4(1.0f));
			float hRatio = (float)openGLWin.glControl.GetViewportHeight() / (float)openGLWin.glControl.GetViewportWidth();
			glm::mat4 twoDScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(hRatio, 1.0f, 1.0f));

			modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.95f - hRatio*0.15f, -0.65f, 0.0f)) * twoDScaleMatrix;
		}
		else
		{
			shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
			shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());

			/*if (colorSelection && glSelector.GetManipulationMode() != MANIPULATION_NONE)
			{
			modelMatrix = glm::translate(glm::mat4(1.0), centerPoint) * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
			} else */
			if (!isSelected || previewSelection)
			{
				//if (previewSelection)
				//modelMatrix = snapTransform * glm::translate(glm::mat4(1.0), translation);
				//else
				modelMatrix = glm::translate(glm::mat4(1.0), translation);
			}
			else if (!colorSelection)
			{
				//if (isOverTrash)
				//	modelMatrix = cursorTranslation * trashScaleMatrix * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
				//else
				modelMatrix = cursorTranslation * selectScaleMatrix * scaleMatrix * zRotation * yRotation * xRotation * originTransform;
			}
		}
		shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

		glBindVertexArray(vao);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
		glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, (GLvoid*)0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);
	}
}

void VCGMeshContainer::AttachToCursor(glm::vec3 nearPoint, glm::vec3 farPoint, int distance)
{
	float carryDistance = (float)distance / 1000.0f;
	selectTranslation.x = nearPoint.x + (carryDistance / 10.0f) * (farPoint.x - nearPoint.x); //(0.50f / 10.0f) * (farPoint.x - nearPoint.x);
	selectTranslation.y = nearPoint.y + (carryDistance / 10.0f) * (farPoint.y - nearPoint.y); //- 1.5f; //(0.50f / 10.0f) * (farPoint.y - nearPoint.y);
	selectTranslation.z = nearPoint.z + (carryDistance / 10.0f) * (farPoint.z - nearPoint.z);//(0.50f / 10.0f) * (farPoint.z - nearPoint.z);

	cursorTranslation = glm::translate(glm::mat4(1.0), selectTranslation);
}

glm::vec3 VCGMeshContainer::GetLowerBounds()
{
	return lowerBounds;
}

glm::vec3 VCGMeshContainer::GetUpperBounds()
{
	return upperBounds;
}

void VCGMeshContainer::SetSnapTransform(std::vector<int> orien)
{
	snapPoint.x = 0.0f;
	snapPoint.y = 0.0f;
	snapPoint.z = 0.0f;
	if (orien[0] != 0)
		snapPoint.x = orien[0] * offSet.x;
	if (orien[1] != 0)
		snapPoint.y = orien[1] * offSet.y;
	if (orien[2] != 0)
		snapPoint.z = orien[2] * offSet.z;

	snapOrientation = orien;
	snapTransform = glm::translate(glm::mat4(1.0), snapPoint);
}

bool VCGMeshContainer::IsLoaded()
{
	return isLoaded;
}

glm::vec3 VCGMeshContainer::GetBasePoint()
{
	basePoint = glm::vec3(centerPoint.x, lowerBounds.y, centerPoint.z);
	return basePoint;
}
void VCGMeshContainer::TranslateVerticesToPoint(glm::vec3 point, std::vector<int> orien)
{
	//glm::vec3 oldLowBounds;
	//oldLowBounds.z = lowerBounds.z;
	glm::mat4 pointTranslation = glm::translate(glm::mat4(1.0), point);
	glm::mat4 snapReverse = glm::mat4(1.0);
	//if (snapOrientation != orien)
	//{
	snapReverse = glm::translate(glm::mat4(1.0), -snapPoint);
	SetSnapTransform(orien);
	//}
	lowerBounds = glm::vec3(9999.0f, 9999.0f, 9999.0f);
	upperBounds = glm::vec3(-9999.0f, -9999.0f, -9999.0f);

	glm::mat4 combinedTranslation = (snapTransform * pointTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform);
	glm::mat4 normalTranslation = glm::transpose(glm::inverse(glm::mat4(combinedTranslation)));
	int normalCount = 0;
	for (int i = 0; i < vertices.size(); i += 1)
	{
		glm::vec4 tmp = glm::vec4(vertices[i].x, vertices[i].y, vertices[i].z, 1.0f);
		glm::vec4 tmpNormal = glm::vec4(vertices[i].normal_x, vertices[i].normal_y, vertices[i].normal_z, 0.0f);

		//tmp = (newTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) * tmp;
		//if (orien == -1)

		tmpNormal = normalTranslation * tmpNormal;
		tmpNormal = glm::normalize(tmpNormal);
		
		tmp = combinedTranslation * tmp;

		//if (doSnap)
		//tmp = snapTransform * snapReverse * tmp;
		//tmp = snapTransform * tmp;

		vertices[i].x = tmp.x;
		vertices[i].y = tmp.y;// abs(centerPoint.y - snapPoint.y);
		vertices[i].z = tmp.z;// +abs(snapPoint.z - lowestZ);
		vertices[i].normal_x = tmpNormal.x;
		vertices[i].normal_y = tmpNormal.y;
		vertices[i].normal_z = tmpNormal.z;
		upperBounds.x = max(upperBounds.x, tmp.x);
		lowerBounds.x = min(lowerBounds.x, tmp.x);
		upperBounds.y = max(upperBounds.y, tmp.y);
		lowerBounds.y = min(lowerBounds.y, tmp.y);
		upperBounds.z = max(upperBounds.z, tmp.z);
		lowerBounds.z = min(lowerBounds.z, tmp.z);

		normalCount += 3;
	}

	angleX = 0;
	angleY = 0;
	angleZ = 0;
	scaleFactor = 1.0f;
	xRotation = glm::mat4(1.0);
	yRotation = glm::mat4(1.0);
	zRotation = glm::mat4(1.0);
	scaleMatrix = glm::mat4(1.0);

	centerPoint.x = (lowerBounds.x + upperBounds.x) / 2.0f;
	centerPoint.y = (lowerBounds.y + upperBounds.y) / 2.0f;
	centerPoint.z = (lowerBounds.z + upperBounds.z) / 2.0f;

	offSet.x = (centerPoint.x - lowerBounds.x);
	offSet.y = (centerPoint.y - lowerBounds.y);
	offSet.z = (centerPoint.z - lowerBounds.z);

	originTransform = glm::translate(glm::mat4(1.0), -centerPoint);

	if ((upperBounds.y - lowerBounds.y) > 0.3f)
	{
		float selectScaleFactor = 0.3f / (upperBounds.y - lowerBounds.y);
		selectScaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3((selectScaleFactor, selectScaleFactor, selectScaleFactor)));
	}

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}



void VCGMeshContainer::ApplyRotation(float degree, glm::vec3 axis)
{
	glm::mat4 pointRotation = glm::rotate(glm::mat4(1.0), degree, axis);

	for (int i = 0; i < vertices.size(); i += 1)
	{
		glm::vec4 tmp = glm::vec4(vertices[i].x, vertices[i].y, vertices[i].z, 1.0f);
		glm::vec4 tmpNormal = glm::vec4(vertices[i].normal_x, vertices[i].normal_y, vertices[i].normal_z, 0.0f);

		//tmp = (newTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) * tmp;
		//if (orien == -1)

		tmpNormal = pointRotation * tmpNormal;
		tmpNormal = glm::normalize(tmpNormal);

		tmp = pointRotation * tmp;

		//if (doSnap)
		//tmp = snapTransform * snapReverse * tmp;
		//tmp = snapTransform * tmp;

		vertices[i].x = tmp.x;
		vertices[i].y = tmp.y;// abs(centerPoint.y - snapPoint.y);
		vertices[i].z = tmp.z;// +abs(snapPoint.z - lowestZ);
		vertices[i].normal_x = tmpNormal.x;
		vertices[i].normal_y = tmpNormal.y;
		vertices[i].normal_z = tmpNormal.z;
	}
}

glm::vec3 VCGMeshContainer::GetCenterPoint()
{

	if (!isSelected)
		return centerPoint;
	else
	{
		//glm::vec4 tmpVec = (cursorTranslation * scaleMatrix * zRotation * yRotation * xRotation * originTransform) 
		//	* glm::vec4(centerPoint.x, centerPoint.y, centerPoint.z, 1.0f);
		glm::vec4 tmpVec = (scaleMatrix * zRotation * yRotation * xRotation) * glm::vec4(centerPoint.x, centerPoint.y, centerPoint.z, 1.0f);
		return glm::vec3(tmpVec.x, tmpVec.y, tmpVec.z);
	}
}

bool VCGMeshContainer::GetHitPoint(glm::vec3 nearPoint, glm::vec3 farPoint, glm::vec3 &output, glm::vec3 &outputNormal, bool snapToVertex)
{
	glm::vec3 rayDirection = glm::normalize(farPoint - nearPoint);

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

	if (snapToVertex)
	{
		float highestZ = -99999.0f;
		glm::vec3 minPoint;
		glm::vec3 v = farPoint - nearPoint;
		double c2 = glm::dot(v, v);
		float minDistance = std::numeric_limits<float>::max();
		int index = -1;
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
				index = i * 6;
				minPoint.x = point.x;
				minPoint.y = point.y;
				minPoint.z = point.z;
			}
		}
		output = minPoint;
	}
	else
	{
		float highestZ = -99999.0f;
		float u, v, tX;
		glm::vec3 normal;
		for (int i = 0; i < indices.size(); i += 1)
		{
			glm::vec3 v0;
			v0.x = vertices[indices[i].v1].x;
			v0.y = vertices[indices[i].v1].y;
			v0.z = vertices[indices[i].v1].z;
			glm::vec3 v1;
			v1.x = vertices[indices[i].v2].x;
			v1.y = vertices[indices[i].v2].y;
			v1.z = vertices[indices[i].v2].z;
			glm::vec3 v2;
			v2.x = vertices[indices[i].v3].x;
			v2.y = vertices[indices[i].v3].y;
			v2.z = vertices[indices[i].v3].z;
			glm::vec3 edge1 = v1 - v0;
			glm::vec3 edge2 = v2 - v0;
			glm::vec3 pVec = glm::cross(rayDirection, edge2);
			float det = glm::dot(edge1, pVec);
			if (det > -0.00000000000000000000000000000001f && det < 0.0000000000000000000000000000001f)
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
			if (vertices[indices[i].v1 + 2].z >= highestZ)
			{
				highestZ = vertices[indices[i].v1 + 2].z;
				tX = glm::dot(edge2, qVec) * invDet;
				normal.x = vertices[indices[i].v1].normal_x;
				normal.y = vertices[indices[i].v1].normal_y;
				normal.z = vertices[indices[i].v1].normal_z;
			}
		}
		glm::vec3 minPoint = nearPoint + rayDirection * tX;
		output = minPoint;
		outputNormal = normal;

	}
	return true;

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
		for (int i = 0; i < vertices.size(); i += 1)
		{
			glm::vec3 point(vertices[i].x, vertices[i].y, vertices[i].z);
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
	if (vbo != 0)
	{ 
		currentMesh.Clear();
		glDeleteBuffers(1, &vbo);
		glDeleteBuffers(1, &ibo);
		glDeleteVertexArrays(1, &vao);
		vbo = 0;
	}
	vertices.clear();
	indices.clear();
	verticesWithHighlights.clear();
	isLoaded = false;
}

void VCGMeshContainer::SetScale(bool positive)
{
	if (positive)
		scaleFactor += scaleBy;
	else
		scaleFactor -= scaleBy;

	scaleMatrix = glm::scale(glm::mat4(1.0), glm::vec3(scaleFactor, scaleFactor, scaleFactor));
}

void VCGMeshContainer::SetAngleX(bool positive)
{
	if (positive)
		angleX += rotateBy;
	else
		angleX -= rotateBy;

	xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
}

void VCGMeshContainer::SetAngleY(bool positive)
{
	cDebug::DbgOut(L"yes?");
	if (positive)
		angleY += rotateBy;
	else
		angleY -= rotateBy;

	yRotation = glm::rotate(glm::mat4(1.0), angleY, glm::vec3(0.0f, 1.0f, 0.0f));
}

void VCGMeshContainer::SetAngleZ(bool positive)
{
	if (positive)
		angleZ += rotateBy;
	else
		angleZ -= rotateBy;

	zRotation = glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f));
}

void VCGMeshContainer::RotateX(int angle)
{
	angleX = (float)angle;
	xRotation = glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
}

void VCGMeshContainer::RotateX(int angle, glm::vec3 axis)
{
	angleX = (float)angle;
	xRotation = glm::rotate(glm::mat4(1.0), angleX, axis);
}

void VCGMeshContainer::RotateY(int angle)
{
	angleY = (float)angle;
	yRotation = glm::rotate(glm::mat4(1.0), angleY, glm::vec3(0.0f, 1.0f, 0.0f));
}

void VCGMeshContainer::RotateY(int angle, glm::vec3 axis)
{
	angleY = (float)angle;
	yRotation = glm::rotate(glm::mat4(1.0), angleY, axis);
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
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(Triangle), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

int VCGMeshContainer::FillHoles(int holeSize)
{

	vcg::tri::UpdateTopology<VCGMesh>::VertexFace(currentMesh);
	vcg::tri::UpdateTopology<VCGMesh>::FaceFace(currentMesh);

	//cDebug::DbgOut(L"Closed holes2: ", 1);
	//int holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingIntersectionFill<vcg::tri::SelfIntersectionEar<VCGMesh> >(currentMesh, holeSize, false);
	int holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingFillAllButLargest<vcg::tri::MinimumWeightEar<VCGMesh> >(currentMesh, holeSize, false);
	//int holeCnt = vcg::tri::Hole<VCGMesh>::FillHoleEar<vcg::tri::TrivialEar<VCGMesh> >(currentMesh, holeSize, false);
	//FillHoleEar< TrivialEar >(mesh, *this, local_facePointer)
	
	//int holeCnt = vcg::tri::Hole<VCGMesh>::EarCuttingFill<vcg::tri::MinimumWeightEar< VCGMesh> >(currentMesh, 10000322, false);
	cDebug::DbgOut(L"Closed holes: ", holeCnt);
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
	//LaplacianSmooth(3);
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
	selectTranslation = glm::vec3(0.0f, 0.0f, 0.0f);
	translation = glm::vec3(0.0f, 0.0f, 0.0f);
}

void VCGMeshContainer::SetPlaneParameters(float x, float y, float z, float d)
{
	planeParameters[0] = x;
	planeParameters[1] = y;
	planeParameters[2] = z;
	planeParameters[3] = d;

	int max = distance(planeParameters, max_element(planeParameters, planeParameters + 3));
	int min = distance(planeParameters, min_element(planeParameters, planeParameters + 3));

	/*cDebug::DbgOut(L"max Value: ", planeParameters[max]);
	cDebug::DbgOut(L"min Value: ", planeParameters[min]);
	cDebug::DbgOut(L"max Position: ", max);
	cDebug::DbgOut(L"min Position: ", min);*/

	int pos = 0;
	if (abs(max) > abs(min))
		pos = max;
	else
		pos = min;

	//orientation = pos;
	if (pos <= 2)
		orientation[pos] = 1;

	/*glm::vec3 normalVector = glm::normalize(glm::vec3(x, y, z));
	glm::vec3 yAxis = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));

	glm::vec3 rotationAxis = glm::cross(normalVector, yAxis);

	//float xRotation = glm::acos(glm::dot(normalVector, glm::vec3(1.0f, 0.0f, 0.0f)));
	float rotY = glm::acos(glm::dot(normalVector, yAxis));
	rotY = rotY * 180.0f / 3.14159265359f;
	//float zRotation = glm::acos(glm::dot(normalVector, glm::vec3(0.0f, 0.0f, 1.0f)));

	groundAlignmentRotation = glm::rotate(glm::mat4(1.0), rotY, rotationAxis);

	cDebug::DbgOut(L"rotationAxis x: ", rotationAxis.x);
	cDebug::DbgOut(L"rotationAxis y: ", rotationAxis.y);
	cDebug::DbgOut(L"rotationAxis z: ", rotationAxis.z);
	cDebug::DbgOut(L"rotY: ", rotY);

	cDebug::DbgOut(L"normalVector x: ", normalVector.x);
	cDebug::DbgOut(L"normalVector y: ", normalVector.y);
	cDebug::DbgOut(L"normalVector z: ", normalVector.z);
	glm::vec3 newVector = glm::rotate(normalVector, rotY, rotationAxis);
	//glm::vec4 tmpVector = glm::vec4(normalVector.x, normalVector.y, normalVector.z, 1.0f);
	//tmpVector = rotation * tmpVector;
	cDebug::DbgOut(L"result x: ", newVector.x);
	cDebug::DbgOut(L"result y: ", newVector.y);
	cDebug::DbgOut(L"result z: ", newVector.z);

	//float otherAngle = glm::rot(normalVector, yAxis, yAxis);*/

}

std::vector<int> VCGMeshContainer::GetOrientation()
{
	if (isWall)
	{
		/*switch (orientation)
		{
		case 0: cDebug::DbgOut(L"x Direction", 1);
		break;
		case 1: cDebug::DbgOut(L"y Direction", 1);
		break;
		case 2: cDebug::DbgOut(L"z Direction", 1);
		break;
		}*/
		return orientation;
	}
	return orientation;
}


void VCGMeshContainer::SetDuplicate(bool flag)
{
	isDuplicate = flag;
}

bool VCGMeshContainer::IsDuplicate()
{
	return isDuplicate;
}

bool VCGMeshContainer::IsDeleted()
{
	return isDeleted;
}

void VCGMeshContainer::Set2D(bool flag)
{
	is2D = flag;
}

void VCGMeshContainer::SetWall(bool flag)
{
	isWall = flag;
}

void VCGMeshContainer::SetGround(bool flag)
{
	isGround = flag;
}

void VCGMeshContainer::SetDeleted(bool flag)
{
	isDeleted = flag;
}


bool VCGMeshContainer::IsWall()
{
	return isWall;
}


bool VCGMeshContainer::IsGround()
{
	return isGround;
}

int VCGMeshContainer::GetColorCode()
{
	return colorCode;
}

void VCGMeshContainer::TogglePreviewSelection(bool flag)
{
	previewSelection = flag;
}

void VCGMeshContainer::IsOverTrash(bool flag)
{
	isOverTrash = flag;
}

std::vector<Vertex> VCGMeshContainer::GetVertices()
{
	return vertices;
}

std::vector<Triangle> VCGMeshContainer::GetIndices()
{
	return indices;
}

int VCGMeshContainer::GetNumberOfVertices()
{
	return vertices.size();
}

int VCGMeshContainer::GetNumberOfTriangles()
{
	return indices.size();
}

bool VCGMeshContainer::AreBuffersInitialized()
{
	return vbo != 0;
}

void VCGMeshContainer::CleanAndParse(std::vector<Vertex> &startingVertices, std::vector<Triangle> &startingIndices)
{
	startingVertices.clear();
	startingIndices.clear();
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

		//startingVertices.push_back(1.0f);
		//colors.push_back(1.0f);

		numvert++;
	}

	//vertices.insert(vertices.end(), colors.begin(), colors.end());

	int mem_index = 0; //var temporany
	for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
	{
		Triangle triangle;
		triangle.v1 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(0))];
		triangle.v2 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(1))];
		triangle.v3 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(2))];

		startingIndices.push_back(triangle);
	}

	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	cDebug::DbgOut(L"Parse duration: ", duration);

}