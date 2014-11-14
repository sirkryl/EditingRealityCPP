#include "common.h"

#include "colorCoding.h"
#include "openGLCamera.h"
#include "openGLShaders.h"
#include "openGLSelector.h"
#include "openGLHelper.h"
#include "openGLText.h"
#include "openGLWin.h"
#include "PCLProcessing.h"
#include <boost/timer.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "Resource.h"
#include "vcgMesh.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#pragma region

//thread related
HANDLE segmentationThread;
DWORD sThreadId;

wstring statusMsg = L"";
//mesh storage
std::vector<shared_ptr<VCGMeshContainer>> meshData;
std::vector<shared_ptr<VCGMeshContainer>> meshData_segTmp;

std::vector<Vertex> startingVertices;
std::vector<Triangle> startingIndices;

//gl related objects
OpenGLSelector glSelector;
OpenGLText glText;
OpenGLCamera glCamera;
OpenGLHelper glHelper;

//pcl segmenter
PCLProcessor pclSegmenter;

//vaos and vbos for helper visualizations
GLuint segmentVBO{ 0 }, segmentVAO{ 0 };

//vertices for helper visualizations
std::vector<Vertex> previewVertices;
//std::vector<Vertex> wallVertices;


//for info text
int numberOfVertices = 0;
int numberOfFaces = 0;

//for status messages (appear 'busy' to the user)
int dotCount = 0;
const std::vector<wstring> dots = { L".", L"..", L"..." };

#pragma endregion variables

#pragma region
void RemoveSelectionColor()
{
	if (glSelector.selectedIndex != -1)
	{
		meshData[glSelector.selectedIndex - 1]->ToggleSelectedColor(false);
	}
}

void SelectWallObject()
{
	glSelector.SelectWallObject();
}

void ResetWallObject()
{
	glSelector.ResetWallObject();
}

void ResetCameraPosition()
{
	glCamera.ResetCameraPosition();
}

void ToggleCameraMode()
{
	if (glCamera.mode == CAMERA_FREE)
		glCamera.mode = CAMERA_SENSOR;
	else
		glCamera.mode = CAMERA_FREE;
}

#pragma endregion calls from UI

void GenerateBuffers()
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

#pragma region

void LoadClusterData()
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

	GenerateBuffers();

	//segFinished = false;
	openGLWin.state = DEFAULT;
	pclSegmenter.coloredCloudReady = false;
}

int WINAPI SegThreadMain()
{
	if (meshData_segTmp.size() > 0 && openGLWin.segmentValuesChanged)
	{
		int cnt = 0;
		for (int i = pclSegmenter.GetPlaneClusterCount(); i < meshData_segTmp.size(); i++)
		{
			meshData_segTmp[i]->ClearMesh();
			//delete meshData_segTmp[i];
		}
		meshData_segTmp.erase(meshData_segTmp.begin() + pclSegmenter.GetPlaneClusterCount(), meshData_segTmp.end());
		/*for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
		if (pclSegmenter.IsPlaneSegmented())
		if (cnt < pclSegmenter.GetPlaneClusterCount())
		continue;
		(*mI)->ClearMesh();
		cnt++;
		}
		meshData_segTmp.clear();*/
	}
	//UNCOMMENT FRO HERE FOR SEGMENTATION

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	if (meshData.size() == 1)
	{
		if (!pclSegmenter.IsMainCloudInitialized())
			meshData[0]->CleanAndParse(startingVertices, startingIndices);
	}

	else
	{
		openGLWin.ShowStatusBarMessage(L"Can't segment mesh, as it is already segmented.");
		return 0;
		switch (openGLWin.testMode)
		{
		case 0:
			//CombineAndExport();
			CleanAndParse("data\\models\\output.ply", startingVertices, startingIndices);
			break;
		case 1:
			CleanAndParse("data\\models\\testScene.ply", startingVertices, startingIndices);
			break;
		case 2:
			CleanAndParse("data\\models\\cube.ply", startingVertices, startingIndices);
			break;
		}
	}
	//CleanAndParse("data\\models\\output.ply", startingVertices, startingIndices, startingNormals);
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	if (!pclSegmenter.IsMainCloudInitialized())
	{
		statusMsg = L"Converting mesh to point cloud";
		pclSegmenter.ConvertToCloud(startingVertices, startingIndices);
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Converted to cloud in ", elapsedTime);

	//pclSegmenter.PlaneSegmentation();
	//pclSegmenter.EuclideanSegmentation();
	cDebug::DbgOut(L"before plane segmentation ", elapsedTime);
	

	if (!pclSegmenter.IsPlaneSegmented())
	{
		pclSegmenter.PlaneSegmentation();
		pclSegmenter.PlaneIndexEstimation();
		for (int i = 0; i < pclSegmenter.GetPlaneClusterCount(); i++)
		{
			std::vector<Vertex> clusterVertices;
			std::vector<Triangle> clusterIndices;
			pclSegmenter.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices);
			cDebug::DbgOut(L"Converted Plane Mesh #", i);
			shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
			mesh->SetColorCode(i + 1);
			//mesh->ParseData(clusterVertices, clusterIndices);
			mesh->SetWall(true);
			mesh->ConvertToVCG(clusterVertices, clusterIndices);

			//int total = mesh->MergeCloseVertices(0.005f);
			//cDebug::DbgOut(_T("Merged close vertices: "), total);
			//cDebug::DbgOut(L"indices: ", (int)mesh->GetVertices().size());
			//cDebug::DbgOut(L"vertices: " + (int)mesh->GetIndices().size());
			mesh->RemoveSmallComponents(1000);
			mesh->CleanMesh();
			mesh->RemoveNonManifoldFace();
			mesh->FillHoles((clusterVertices.size()) / 10);
			mesh->CleanMesh();
			mesh->ParseData();
			mesh->SetPlaneParameters(pclSegmenter.planeCoefficients[i]->values[0], pclSegmenter.planeCoefficients[i]->values[1],
				pclSegmenter.planeCoefficients[i]->values[2], pclSegmenter.planeCoefficients[i]->values[3]);
			//cDebug::DbgOut(L"vertices: ", (int)mesh->GetNumberOfVertices());
			//cDebug::DbgOut(L"indices: " + (int)mesh->GetNumberOfTriangles());
			meshData_segTmp.push_back(mesh);
		}
	}

	if (openGLWin.segmentationMode == REGION_GROWTH_SEGMENTATION)
	{
		if (openGLWin.state != SEGMENTATION_PREVIEW || openGLWin.previewMode || openGLWin.segmentValuesChanged)
		{

			//meshData_segTmp[0]->CleanAndParse(startingVertices, startingIndices, startingNormals);
			//pclSegmenter.ConvertToCloud(startingVertices, startingIndices, startingNormals);
			pclSegmenter.RegionGrowingSegmentation();

			openGLWin.segmentValuesChanged = false;
		}
		if (openGLWin.previewMode)
		{
			openGLWin.state = SEGMENTATION_PREVIEW;
			cDebug::DbgOut(L"Number of clusters: ", pclSegmenter.GetClusterCount());
			return 0;
		}

	}
	if (openGLWin.segmentationMode == EUCLIDEAN_SEGMENTATION)
	{
		pclSegmenter.EuclideanSegmentation();
	}

	pclSegmenter.IndexEstimation();
	cDebug::DbgOut(L"Clustered");


	//elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	//cDebug::DbgOut(L"Time for map creation ", elapsedTime);
	cDebug::DbgOut(L"Number of clusters: ", pclSegmenter.GetRegionClusterCount());
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	for (int i = 0; i < pclSegmenter.GetRegionClusterCount(); i++)
	{
		std::vector<Vertex> clusterVertices;
		std::vector<Triangle> clusterIndices;
		pclSegmenter.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices);
		//cDebug::DbgOut(L"Converted Triangle Mesh #",i);
		shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
		mesh->SetColorCode(i + pclSegmenter.GetPlaneClusterCount() + 1);
		//mesh->ParseData(clusterVertices, clusterIndices);
		mesh->ConvertToVCG(clusterVertices, clusterIndices);

		mesh->CleanMesh();
		mesh->RemoveNonManifoldFace();

		mesh->FillHoles((clusterVertices.size()) / 10);
		mesh->CleanMesh();
		mesh->ParseData();
		meshData_segTmp.push_back(mesh);
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Filled MeshData in ", elapsedTime);
	//UNCOMMENT UNTIL HERE FOR SEGMENTATION
	openGLWin.state = SEGMENTATION_FINISHED;
	//showColoredSegments = false;
	//segFinished = true;
	openGLWin.ShowStatusBarMessage(L"Segmented mesh in " + to_wstring(pclSegmenter.GetRegionClusterCount()) + L"clusters.");

	return 0;

}

void StartSegmentation()
{
	//segmenting = true;
	openGLWin.state = SEGMENTATION;
	segmentationThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&SegThreadMain, 0, 0, &sThreadId);
}

void GeneratePreviewBuffers()
{
	glGenBuffers(1, &segmentVBO);
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

bool InitializePreview()
{
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud;
	if (openGLWin.state == WALL_SELECTION)
	{
		pointCloud = pclSegmenter.wallSegmentCloud;
	}
	else if (openGLWin.state == SEGMENTATION_PREVIEW)
	{
		pointCloud = pclSegmenter.coloredSegmentedCloud;
	}
	else return false;

	previewVertices.clear();
	for (int i = 0; i < pointCloud->points.size(); i++)
	{
		Vertex vertex;
		vertex.x = pointCloud->points[i].x;
		vertex.y = pointCloud->points[i].y;
		vertex.z = pointCloud->points[i].z;
		vertex.r = pointCloud->points[i].r / 255.0f;
		vertex.g = pointCloud->points[i].g / 255.0f;
		vertex.b = pointCloud->points[i].b / 255.0f;
		vertex.normal_x = 0;
		vertex.normal_y = 0;
		vertex.normal_z = 0;
		previewVertices.push_back(vertex);
	}

	if (segmentVBO == 0)
		GeneratePreviewBuffers();

	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glBufferData(GL_ARRAY_BUFFER, previewVertices.size() * sizeof(Vertex), &previewVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if (openGLWin.state == WALL_SELECTION)
		openGLWin.ShowStatusBarMessage(L"Showing wall preview.");
	else if (openGLWin.state == SEGMENTATION_PREVIEW)
	{
		openGLWin.ShowStatusBarMessage(L"Showing region growth segmentation preview with " + to_wstring(pclSegmenter.GetClusterCount()) + L"clusters");
		pclSegmenter.coloredCloudReady = false;
	}

}

void RenderPreview()
{
	openGLWin.SetBackgroundColor(0, 0, 0);

	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix = glm::mat4(1.0);

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(segmentVAO);
	glPointSize(1.5f);
	glDrawArrays(GL_POINTS, 0, previewVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);

	if (openGLWin.state == SEGMENTATION_PREVIEW)
	{
		glText.PrepareForRender();
		glText.RenderText(L"Clusters: ", pclSegmenter.GetClusterCount(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);
	}

	glEnable(GL_DEPTH_TEST);
}

#pragma endregion segmentation related

#pragma region

void FillHoles(int holeSize)
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

void RemoveSmallComponents(int size)
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

void CleanMesh()
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
#pragma endregion mesh optimizations

void SetViewportStatusMessage(wstring message)
{
	statusMsg = message;
}

void ShowStatusMsg()
{
	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	wstring loadString = statusMsg + L"" + dots[floor(dotCount / 100)];
	float xPos = 0.0f - statusMsg.size()*0.01f;
	if (dotCount == 299)
		dotCount = 0;
	else
		dotCount++;

	glText.PrepareForRender();
	glText.RenderText(loadString, 25, xPos, 0.05f, 2.0f / w, 2.0f / h);
}

void HandleKeyInput()
{
	if (Keys::GetKeyStateOnce(VK_F10))
	{
		openGLWin.ToggleDebugControls();
	}
}

void LoadInput()
{
	shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
	mesh->SetColorCode(1);

	switch (openGLWin.testMode)
	{
	case 0:
		mesh->LoadMesh("data\\models\\output.ply");
		break;
	case 1:
		mesh->LoadMesh("data\\models\\testScene.ply");

		//mesh->LoadMesh("data\\models\\testScene.ply");
		break;
	case 2:
		mesh->LoadMesh("data\\models\\cube.ply");
		break;
	}
	meshData.push_back(mesh);
	glCamera.SetRotationPoint(mesh->GetCenterPoint());
	//openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
	
	
	openGLWin.segmentationMode = EUCLIDEAN_SEGMENTATION;
	openGLWin.previewMode = false;
	StartSegmentation();

	//openGLWin.state = BUFFERS;
	
	
}

void Initialize(LPVOID lpParam)
{
	if (!PrepareShaderPrograms())
	{
		PostQuitMessage(0);
		return;
	}

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glDepthFunc(GL_LESS);
	glDepthRange(-1.0f, 1000.0f);

	glText.Initialize("FreeSans.ttf");

	glCamera = OpenGLCamera(glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), 1.0f, 0.1f);
}

void Render(LPVOID lpParam)
{
	//clear window
	glClearColor(openGLWin.bgRed, openGLWin.bgGreen, openGLWin.bgBlue, 1.0f);
	glClearDepth(1.0);

	HandleKeyInput();

	int viewportWidth = openGLWin.glControl.GetViewportWidth();
	int viewportHeight = openGLWin.glControl.GetViewportHeight();

	openGLWin.glControl.ResizeOpenGLViewportFull(viewportWidth, viewportHeight);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (openGLWin.wireFrameMode && openGLWin.state != SEGMENTATION_PREVIEW && openGLWin.state != WALL_SELECTION)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//for initializing without segmentation..debug purposes
	if (openGLWin.state == BUFFERS)
	{
		GenerateBuffers();
		openGLWin.state = DEFAULT;
		return;
	}

	//USER SELECTS WALL OR SEES SOME OTHER PREVIEW
	if (openGLWin.state == WALL_SELECTION || openGLWin.state == SEGMENTATION_PREVIEW)
	{
		if (previewVertices.size() == 0)
		{
			if (openGLWin.state == SEGMENTATION_PREVIEW)
			{
				if (pclSegmenter.coloredCloudReady)
					InitializePreview();
				else
					return;
			}
			InitializePreview();
		}
		else
		{
			RenderPreview();
			glCamera.mode = CAMERA_FREE;
		}

		openGLWin.glControl.SwapBuffers();
		return;
	}
	else if (previewVertices.size() > 0)
	{
		previewVertices.clear();
		glCamera.ResetCameraPosition();
		glCamera.mode = CAMERA_SENSOR;
	}

	//SEGMENTATION IS FINISHED, LOAD RELEVANT DATA
	if (openGLWin.state == SEGMENTATION_FINISHED)
	{
		LoadClusterData();
		return;
	}

	//WHENEVER MESSAGES SHOULD BE DISPLAYED
	if (openGLWin.state == INITIALIZING || openGLWin.state == SEGMENTATION)
	{
		ShowStatusMsg();
		openGLWin.glControl.SwapBuffers();
		return;
	}

	//if user clicks with left mouse button -> picking/placing
	if (Keys::GetKeyStateOnce(VK_LBUTTON) && glCamera.mode == CAMERA_SENSOR && openGLWin.IsMouseInOpenGLWindow())
	{
		if (glSelector.selectedIndex == -1)
			glSelector.ProcessPicking();
		else
			if (openGLWin.colorSelection)
				glSelector.ProcessPicking();
			else
				glSelector.ProcessPlacing();
	}

	//process currently selected object (attach to cursor, rotation/scaling etc.)
	if (glSelector.selectedIndex != -1 && !openGLWin.colorSelection)
	{
		glSelector.ProcessSelectedObject();
	}

	//draw every mesh
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->Draw();
	}

	//draw helper visuals
	if (openGLWin.helpingVisuals && glSelector.firstRayCast)
	{
		glHelper.RenderHelpingVisuals();
	}

	//show bounding boxes on/off
	if (openGLWin.showBB)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
		{
			if (!(*mI)->IsWall())
				(*mI)->DrawBB();
		}
	}

	//render info text on screen
	glText.PrepareForRender();
	glText.RenderText(L"FPS: ", openGLWin.glControl.GetFPS(), 20, -0.98f, 0.85f, 2.0f / viewportWidth, 2.0f / viewportHeight);
	glText.RenderText(L"Meshs: ", meshData.size(), 15, -0.98f, 0.75f, 2.0f / viewportWidth, 2.0f / viewportHeight);
	glText.RenderText(L"Verts: ", numberOfVertices, 15, -0.98f, 0.70f, 2.0f / viewportWidth, 2.0f / viewportHeight);
	glText.RenderText(L"Faces: ", numberOfFaces, 15, -0.98f, 0.65f, 2.0f / viewportWidth, 2.0f / viewportHeight);
	glText.RenderText(L"Sel: ", glSelector.selectedIndex - 1, 15, -0.98f, 0.6f, 2.0f / viewportWidth, 2.0f / viewportHeight);

	glEnable(GL_DEPTH_TEST);

	//swap buffers to actually display the changes
	openGLWin.glControl.SwapBuffers();
}

void Release(LPVOID lpParam)
{
	shaderColor.DeleteProgram();
	shaderFont.DeleteProgram();
	glText.CleanUp();
	glHelper.CleanUp();
	glDeleteBuffers(1, &segmentVBO);
	glDeleteVertexArrays(1, &segmentVAO);

	for (int i = 0; i < 4; i++)shaders[i].DeleteShader();

	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData.clear();
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData_segTmp.clear();

}
