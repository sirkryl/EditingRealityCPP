#include "SegmentationHelper.h"
#include "InteractiveFusion.h"
#include "PCLProcessor.h"
#include "VcgMeshContainer.h"
#include "OpenGLShaders.h"
#include "OpenGLCamera.h"
#include "OpenGLText.h"
#include "MeshHelper.h"
#include <random>
bool isPreviewInitialized = false;
int currentPlaneIndex = -1;
PCLProcessor pclProcessor;
std::vector<Vertex> startingVertices;
std::vector<Triangle> startingIndices;

shared_ptr<VCGMeshContainer> previewMesh(new VCGMeshContainer);
glm::vec4 planeC;

std::vector<shared_ptr<VCGMeshContainer>> meshData_segTmp;

bool SegmentationHelper::IsCloudReady()
{
	if(GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
		return pclProcessor.GetRegionClusterCount() > 0;
	else if (GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
		return pclProcessor.GetInlierIndices().size() > 0;
	return false;
}

bool SegmentationHelper::IsPreviewInitialized()
{
	return isPreviewInitialized;
}

void SegmentationHelper::ResetInitializedStatus()
{
	isPreviewInitialized = false;
}

void SegmentationHelper::ClearPreviewVertices()
{
	previewVertices.clear();
	isPreviewInitialized = false;
}

void SegmentationHelper::LoadClusterData()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
		//delete *mI;
	}
	meshData.clear();
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		//if (tmp != glSelector.selectedIndex-1)
		meshData.push_back(*mI);
		//meshData.(*mI)->Draw();
		//tmp++;
	}

	/*for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
	(*mI)->ClearMesh();
	}*/
	meshData_segTmp.clear();

	meshHelper.GenerateBuffers();

	//segFinished = false;
	//openGLWin.state = DEFAULT;
	pclProcessor.coloredCloudReady = false;
}

int WINAPI SegThreadMain()
{

	if (meshData_segTmp.size() > 0 && openGLWin.segmentValuesChanged)
	{
		int cnt = 0;
		for (int i = pclProcessor.GetPlaneClusterCount(); i < meshData_segTmp.size(); i++)
		{
			meshData_segTmp[i]->ClearMesh();
		}
		meshData_segTmp.erase(meshData_segTmp.begin() + pclProcessor.GetPlaneClusterCount(), meshData_segTmp.end());
	}
	//UNCOMMENT FRO HERE FOR SEGMENTATION

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	if (!pclProcessor.IsMainCloudInitialized())
		originalMesh->CleanAndParse(startingVertices, startingIndices);

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	if (!pclProcessor.IsMainCloudInitialized())
	{
		statusMsg = L"Converting mesh to point cloud";
		openGLWin.SetProgressionText(L"Converting mesh to point cloud");
		openGLWin.SetProgressionPercent(L"");
		pclProcessor.ConvertToCloud(startingVertices, startingIndices);
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Converted to cloud in ", elapsedTime);

	cDebug::DbgOut(L"before plane segmentation ", elapsedTime);

	if (!pclProcessor.IsPlaneSegmented())
	{
		statusMsg = L"Trying to find planes";
		glSegmentation.SetSegmentationState(IF_SEGSTATE_PLANE_SEGMENTATION);
		pclProcessor.PlaneSegmentation();
		pclProcessor.PlaneIndexEstimation();
		for (int i = 0; i < pclProcessor.GetPlaneClusterCount(); i++)
		{
			std::vector<Vertex> clusterVertices;
			std::vector<Triangle> clusterIndices;
			pclProcessor.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices, true);

			//pclProcessor.ConfirmPlaneIndices(i);
			cDebug::DbgOut(L"Converted Plane Mesh #", i);
			shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
			mesh->SetColorCode(i + 1);
			mesh->SetWall(true);
			mesh->ConvertToVCG(clusterVertices, clusterIndices);
			float threshold = 0.005f;
			int total = mesh->MergeCloseVertices(threshold);
			cDebug::DbgOut(_T("Merged close vertices: "), total);

			cDebug::DbgOut(_T("Clean PlaneSeg #1"));
			mesh->CleanMesh();
			mesh->RemoveSmallComponents(clusterVertices.size() / 5);

			mesh->RemoveNonManifoldFace();
			//mesh->FillHoles(100000);
			total = mesh->MergeCloseVertices(threshold);
			cDebug::DbgOut(_T("Merged close vertices 22: "), total);
			mesh->CleanMesh();

			//mesh->RemoveSmallComponents(clusterVertices.size() / 3);
			//mesh->CleanMesh();
			mesh->ParseData();
			mesh->SetPlaneParameters(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1],
				pclProcessor.planeCoefficients[i]->values[2], pclProcessor.planeCoefficients[i]->values[3]);
			planeC = glm::vec4(pclProcessor.planeCoefficients[i]->values[0], pclProcessor.planeCoefficients[i]->values[1], pclProcessor.planeCoefficients[i]->values[2], pclProcessor.planeCoefficients[i]->values[3]);

			meshData_segTmp.push_back(mesh);
		}
		//pclProcessor.PrepareForObjectSegmentation();

	}
	glSegmentation.SetSegmentationState(IF_SEGSTATE_OBJECT_SEGMENTATION);
	
	if (glSegmentation.GetSegmentationMode() == SEGMENTATION_REGIONGROWTH)
	{
		if (openGLWin.previewMode || openGLWin.segmentValuesChanged)
		{
			pclProcessor.RegionGrowingSegmentation();

			openGLWin.segmentValuesChanged = false;
		}
		if (openGLWin.previewMode)
		{
			//openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
			openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
			cDebug::DbgOut(L"Number of clusters: ", pclProcessor.GetClusterCount());
			isPreviewInitialized = false;
			return 0;
		}

	}
	if (glSegmentation.GetSegmentationMode() == SEGMENTATION_EUCLIDEAN)
	{
		if (openGLWin.previewMode || openGLWin.segmentValuesChanged)
		{
			pclProcessor.EuclideanSegmentation();
			openGLWin.segmentValuesChanged = false;
		}

		if (openGLWin.previewMode)
		{
			//openGLWin.SetWindowState(SEGMENTATION_PREVIEW);
			openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
			cDebug::DbgOut(L"Number of clusters: ", pclProcessor.GetClusterCount());
			isPreviewInitialized = false;
			return 0;
		}
	}

	pclProcessor.IndexEstimation();
	cDebug::DbgOut(L"Clustered");


	//elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	//cDebug::DbgOut(L"Time for map creation ", elapsedTime);
	cDebug::DbgOut(L"Number of clusters: ", pclProcessor.GetRegionClusterCount());
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	for (int i = 0; i < pclProcessor.GetRegionClusterCount(); i++)
	{
		std::vector<Vertex> clusterVertices;
		std::vector<Triangle> clusterIndices;
		if (!pclProcessor.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices, false))
			continue;
		//cDebug::DbgOut(L"Converted Triangle Mesh #",i);
		shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
		mesh->SetColorCode(i + pclProcessor.GetPlaneClusterCount() + 1);
		//mesh->ParseData(clusterVertices, clusterIndices);
		mesh->ConvertToVCG(clusterVertices, clusterIndices);
		cDebug::DbgOut(_T("Clean Seg #1"));
		mesh->CleanMesh();
		mesh->RemoveNonManifoldFace();

		//mesh->FillHoles((clusterVertices.size()) / 10);
		//mesh->CleanMesh();
		mesh->ParseData();
		//glm::vec3 cP = mesh->GetCenterPoint();
		//float iO = cP.x*planeC.x + cP.y*planeC.y + cP.z * planeC.z + planeC.w;
		//cDebug::DbgOut(L"mesh #" + to_wstring(i) + L" iO: ", iO);
		//if (iO >= 0.0f)
			meshData_segTmp.push_back(mesh);
		
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Filled MeshData in ", elapsedTime);
	SetViewportPercentMsg(L"");
	openGLWin.SetProgressionPercent(L"");
	//UNCOMMENT UNTIL HERE FOR SEGMENTATION
	glSegmentation.SetSegmentationState(IF_SEGSTATE_FINISHED);
	//openGLWin.SetWindowState(SEGMENTATION_FINISHED);
	//openGLWin.SetWindowMode(MODE_INTERACTION);
	//showColoredSegments = false;
	//segFinished = true;
	openGLWin.ShowStatusBarMessage(L"Segmented mesh in " + to_wstring(pclProcessor.GetRegionClusterCount()) + L"clusters.");
	//openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
	return 0;

}

void SegmentationHelper::StartSegmentation()
{
	openGLWin.SetWindowMode(IF_MODE_SEGMENTATION);
	openGLWin.SetWindowBusyState(IF_BUSYSTATE_BUSY);
	segmentationThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&SegThreadMain, 0, 0, &sThreadId);
}

void SegmentationHelper::GeneratePreviewBuffers()
{
	glGenBuffers(1, &segmentVBO);

	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glBufferData(GL_ARRAY_BUFFER, previewVertices.size() * sizeof(Vertex), &previewVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &segmentIBO);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, segmentIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, previewIndices.size() * sizeof(Triangle), &previewIndices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glGenVertexArrays(1, &segmentVAO);
	glBindVertexArray(segmentVAO);
	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 3));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(sizeof(float)* 6));
}

bool SegmentationHelper::IsWallReady()
{
	return pclProcessor.GetInlierIndices().size() > 0;
}

bool SegmentationHelper::InitializePreview()
{
	if (GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
	{
		ColorIF color = { 0.4f, 0.0f, 0.0f };
		meshHelper.RemoveAllHighlights();
		meshHelper.HighlightObjectsInOriginal(pclProcessor.GetInlierIndices(), color, true);
		isPreviewInitialized = true;
	}
	else if (GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0.3f, 0.7f);
		meshHelper.RemoveAllHighlights();
		//meshHelper.GenerateBuffers();
		for (int i = 0; i < pclProcessor.GetRegionClusterCount(); i++)
		{
			ColorIF color = { dis(gen), dis(gen), dis(gen) };
			meshHelper.HighlightObjectsInOriginal(pclProcessor.GetColoredCloudIndices(i), color, false);
		}
		isPreviewInitialized = true;
	}
	openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
	return true;
}

bool SegmentationHelper::InitializePreview(std::vector<Vertex> vertices, std::vector<Triangle> triangles)
{
	previewVertices.clear();
	previewIndices.clear();
	for (int i = 0; i < vertices.size(); i++)
	{
		previewVertices.push_back(vertices[i]);
	}
	for (int i = 0; i < triangles.size(); i++)
	{
		previewIndices.push_back(triangles[i]);
	}

	if (GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
		openGLWin.ShowStatusBarMessage(L"Showing wall preview.");
	if (GetSegmentationState() == IF_SEGSTATE_OBJECT_SEGMENTATION)
	{
		openGLWin.ShowStatusBarMessage(L"Showing region growth segmentation preview with " + to_wstring(pclProcessor.GetClusterCount()) + L"clusters");
		pclProcessor.coloredCloudReady = false;
	}
	return true;
}

void SegmentationHelper::RenderPreview()
{
	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	previewMesh->Draw();
	glText.PrepareForRender();
	glText.RenderText(L"Clusters: ", pclProcessor.GetClusterCount(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);
	glEnable(GL_DEPTH_TEST);
	return;
}

glm::vec3 SegmentationHelper::GetPreviewCenterPoint()
{
	glm::vec3 centerPoint = glm::vec3(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < previewVertices.size(); i++)
	{
		centerPoint.x += previewVertices[i].x;
		centerPoint.y += previewVertices[i].y;
		centerPoint.z += previewVertices[i].z;
	}
	centerPoint = centerPoint / (float)previewVertices.size();
	return centerPoint;
}

SegmentationState SegmentationHelper::GetSegmentationState()
{
	return segState;
}

void SegmentationHelper::SetSegmentationState(SegmentationState state)
{
	segState = state;
	
	openGLWin.UpdateSegmentationUI();
}

SegmentationMode SegmentationHelper::GetSegmentationMode()
{
	return segMode;
}

void SegmentationHelper::SetSegmentationMode(SegmentationMode mode)
{
	segMode = mode;
}


void SegmentationHelper::ClearForResume()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData_segTmp.clear();
	SetSegmentationState(IF_SEGSTATE_NONE);
	pclProcessor.ClearAll();
	isPreviewInitialized = false;
}

void SegmentationHelper::CleanUp()
{
	glDeleteBuffers(1, &segmentVBO);
	glDeleteVertexArrays(1, &segmentVAO);
	glDeleteBuffers(1, &segmentIBO);
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData_segTmp.clear();
}