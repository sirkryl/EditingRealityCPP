#include "common.h"

#include "colorCoding.h"
#include "openGLCamera.h"
#include "openGLShaders.h"
#include "openGLText.h"
#include "openGLWin.h"
#include "PCLProcessing.h"
#include "Resource.h"
#include "vcgMesh.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/gl/trimesh.h>
#include <wrap/io_trimesh/export.h>

#pragma region
//thread related
HANDLE segmentationThread;
DWORD sThreadId;

//mesh storage
std::vector<VCGMeshContainer*> meshData;
std::vector<VCGMeshContainer*> meshData_segTmp;

//helper objects
VCGMeshContainer mesh;

//gl related objects
OpenGLText glText;
OpenGLCamera glCamera;

//pcl segmenter
PCLProcessor pclSegmenter;

//vaos and vbos for helper visualizations
GLuint segmentVBO{ 0 }, segmentVAO{ 0 };
GLuint rayVBO{ 0 }, rayVAO{ 0 };
GLuint pointVBO{ 0 }, pointVAO{ 0 };

//vertices for helper visualizations
std::vector<float> segVertices;
std::vector<float> rayVertices;
std::vector<float> pointVertices;

//objects for helper visualizations
glm::vec3 hitPoint{ -1, -1, -1 };
glm::vec3 nearPoint;


//background color
float bgRed = 0.8f; 
float bgGreen = 0.8f;
float bgBlue = 0.8f;

//for info text
int numberOfVertices = 0;
int numberOfFaces = 0;

//currently selected object (or -1 if none selected)
int selectedIndex = -1;

//index of wall object
int wallIndex = -1;

//first time something happened flags
bool firstRayCast = false;

//initialized flags
bool initSegmentVBO = false;
bool initRayVBO = false;
bool initPointVBO = false;
bool initColoredSegments = false;

//toggle different modes
bool showBB = false;
bool wireFrameMode = false;

//segmentation flags
bool segFinished = false;
bool showColoredSegments = false;

#pragma endregion variables

#pragma region
void ToggleColorSelectedObject()
{
	openGLWin.colorSelection = !openGLWin.colorSelection;
	if (!openGLWin.colorSelection)
	{
		if (selectedIndex != -1)
		{
			meshData[selectedIndex - 1]->ToggleSelectedColor(false);
		}
	}
}

void ToggleWireFrame()
{
	wireFrameMode = !wireFrameMode;
}

void ToggleBoundingBoxes()
{
	showBB = !showBB;
}

void SetBackgroundColor(int redValue, int greenValue, int blueValue)
{
	bgRed = redValue / 255.0f;
	bgGreen = greenValue / 255.0f;
	bgBlue = blueValue / 255.0f;
}

void ResetCameraPosition()
{
	glCamera.ResetCameraPosition();
}
#pragma endregion calls from UI

#pragma region
int GetColorUnderCursor()
{
	POINT cursorPos;
	GetCursorPos(&cursorPos);
	ScreenToClient(openGLWin.glWindowHandle, &cursorPos);
	RECT rect;
	GetClientRect(openGLWin.glWindowHandle, &rect);
	cursorPos.y = rect.bottom - cursorPos.y;
	BYTE colorByte[4];
	glReadPixels(cursorPos.x, cursorPos.y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, colorByte);
	int output = colorCoding::ColorToInt(colorByte[0], colorByte[1], colorByte[2]);
	if (output == RGB_BLACK)
		return -1;
	return output;
}

void InitializeRayVisual()
{
	rayVertices.clear();
	rayVertices.push_back(nearPoint.x);
	rayVertices.push_back(nearPoint.y);
	rayVertices.push_back(nearPoint.z);
	rayVertices.push_back(1.0f);
	rayVertices.push_back(0.0f);
	rayVertices.push_back(0.0f);
	if (!initRayVBO)
	{
		glGenBuffers(1, &rayVBO);
		glGenVertexArrays(1, &rayVAO);
		initRayVBO = true;
	}
	glBindBuffer(GL_ARRAY_BUFFER, rayVBO);
	glBufferData(GL_ARRAY_BUFFER, rayVertices.size() * sizeof(float), &rayVertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(rayVAO);

	glBindBuffer(GL_ARRAY_BUFFER, rayVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));
}

void RayCast(glm::vec3* v1, glm::vec3* v2)
{
	POINT cursorPos;
	GetCursorPos(&cursorPos);
	ScreenToClient(openGLWin.glWindowHandle, &cursorPos);
	RECT rect;
	GetClientRect(openGLWin.glWindowHandle, &rect);
	cursorPos.y = rect.bottom - cursorPos.y;

	glm::vec4 viewport = glm::vec4(0.0f, 0.0f, openGLWin.glControl.GetViewportWidth(), openGLWin.glControl.GetViewportHeight());

	*v1 = glm::unProject(glm::vec3(float(cursorPos.x), float(cursorPos.y), 0.0f), glCamera.GetViewMatrix(), *openGLWin.glControl.GetProjectionMatrix(), viewport);
	*v2 = glm::unProject(glm::vec3(float(cursorPos.x), float(cursorPos.y), 1.0f), glCamera.GetViewMatrix(), *openGLWin.glControl.GetProjectionMatrix(), viewport);
	nearPoint = *v1;
	if (openGLWin.helpingVisuals)
	{
		InitializeRayVisual();
		if (!firstRayCast)
			firstRayCast = true;
	}

}

void ProcessSelectedObject()
{
	glm::vec3 v1, v2;
	RayCast(&v1, &v2);
	meshData[selectedIndex - 1]->AttachToCursor(v1, v2);
	if (Keys::GetKeyState('X'))
	{
		//cDebug::DbgOut(L"Wheel: ", openGLWin.GetWheelDelta());
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex - 1]->SetAngleX(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex - 1]->SetAngleX(true);
		}
	}
	if (Keys::GetKeyState('Y'))
	{
		//cDebug::DbgOut(L"Wheel: ", openGLWin.GetWheelDelta());
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex - 1]->SetAngleY(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex - 1]->SetAngleY(true);
		}

	}
	if (Keys::GetKeyState('Z'))
	{
		//cDebug::DbgOut(L"Wheel: ", openGLWin.GetWheelDelta());
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex - 1]->SetAngleZ(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex - 1]->SetAngleZ(true);
		}
	}
	if (Keys::GetKeyState('U'))
	{
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex - 1]->SetScale(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex - 1]->SetScale(true);
		}
		openGLWin.wheelDelta = 0;
	}
	openGLWin.wheelDelta = 0;

}

void ProcessPicking()
{
	std::wstringstream ws;
	int cnt = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (cnt != wallIndex - 1)
			(*mI)->DrawBB();
		cnt++;
	}
	int tmpIndex = GetColorUnderCursor();
	if (tmpIndex > -1 && tmpIndex <= meshData.size())
	{
		if (selectedIndex != -1)
		{
			if (openGLWin.colorSelection)
				meshData[selectedIndex - 1]->ToggleSelectedColor(false);
			meshData[selectedIndex - 1]->SetSelected(false);
		}
		ws << L"Selecting object #";
		ws << tmpIndex;
		meshData[tmpIndex - 1]->SetSelected(true);
		if (openGLWin.colorSelection)
			meshData[tmpIndex - 1]->ToggleSelectedColor(true);
		selectedIndex = tmpIndex;
	}
	else
	{
		if (selectedIndex != -1)
		{
			ws << L"Unselecting object #";
			ws << selectedIndex;
			if (openGLWin.colorSelection)
				meshData[selectedIndex - 1]->ToggleSelectedColor(false);
			meshData[selectedIndex - 1]->SetSelected(false);
			selectedIndex = -1;
		}
	}
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void ProcessPlacing()
{
	glm::vec3 v1, v2;
	RayCast(&v1, &v2);
	int index = 0;
	float maxZ = -1000.0f;
	int sIndex = -1;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (index != selectedIndex - 1)
		{
			glm::vec3 tmpHit;
			if ((*mI)->CheckCollision(v1, v2, tmpHit))
			{
				//float cMaxZ = (*mI)->GetUpperBounds().z;
				float cMaxZ = tmpHit.z;
				cDebug::DbgOut(L"Collision found", index);
				cDebug::DbgOut(L"zWert ist ", cMaxZ);
				if (cMaxZ > maxZ)
				{

					hitPoint = tmpHit;
					sIndex = index;
					maxZ = cMaxZ;
				}

			}
		}
		index++;
	}
	if (sIndex != -1)
	{
		cDebug::DbgOut(L"Collision chosen", sIndex);
		meshData[selectedIndex - 1]->SetSelected(false);
		//hitPoint.z = meshData[sIndex]->GetUpperBounds().z;
		meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint);
		selectedIndex = -1;
	}
	else
	{
		meshData[selectedIndex - 1]->SetSelected(false);
		meshData[selectedIndex - 1]->ResetSelectedTransformation();
		selectedIndex = -1;
	}
}

#pragma endregion picking, placing, raycast

#pragma region
void SelectWallObject()
{
	if (selectedIndex != -1)
	{
		wallIndex = selectedIndex;
		if (openGLWin.colorSelection)
			meshData[selectedIndex - 1]->ToggleSelectedColor(false);
		meshData[selectedIndex - 1]->SetSelected(false);
		selectedIndex = -1;
	}
}

void ResetWallObject()
{
	if (wallIndex != -1)
		wallIndex = -1;
}
#pragma endregion wall object

#pragma region
void FillPointsToVisualize()
{
	pointVertices.clear();
	pointVertices.push_back(glCamera.GetPosition().x);
	pointVertices.push_back(glCamera.GetPosition().y);
	pointVertices.push_back(glCamera.GetPosition().z);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(1.0f);
	pointVertices.push_back(nearPoint.x);
	pointVertices.push_back(nearPoint.y);
	pointVertices.push_back(nearPoint.z);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(0.0f);
	pointVertices.push_back(1.0f);
	if (hitPoint.x != -1)
	{
		pointVertices.push_back(hitPoint.x);
		pointVertices.push_back(hitPoint.y);
		pointVertices.push_back(hitPoint.z);
		pointVertices.push_back(0.0f);
		pointVertices.push_back(0.0f);
		pointVertices.push_back(1.0f);
	}
	/*if (selectedIndex != -1)
	{
		pointVertices.push_back(meshData[selectedIndex - 1]->GetCenterPoint().x);
		pointVertices.push_back(meshData[selectedIndex - 1]->GetCenterPoint().y);
		pointVertices.push_back(meshData[selectedIndex - 1]->GetCenterPoint().z);
		pointVertices.push_back(0.0f);
		pointVertices.push_back(1.0f);
		pointVertices.push_back(0.0f);
	}*/

	if (!initPointVBO)
	{
		glGenBuffers(1, &pointVBO);
		glGenVertexArrays(1, &pointVAO);
		initPointVBO = true;
	}
	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	glBufferData(GL_ARRAY_BUFFER, pointVertices.size() * sizeof(float), &pointVertices[0], GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(pointVAO);

	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));
}

void RenderHelpingVisuals()
{
	//RENDER HELPING POINTS
	FillPointsToVisualize();
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix = glm::mat4(1.0);

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(pointVAO);
	glPointSize(100.0f);
	glDrawArrays(GL_POINTS, 0, pointVertices.size());
	glBindVertexArray(0);

	//RENDER RAY
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glBindVertexArray(rayVAO);
	glLineWidth(10.0f);
	glDrawArrays(GL_LINES, 0, rayVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);
	glDisable(GL_LINE_SMOOTH);
}
#pragma endregion helping visualizations

#pragma region
int WINAPI SegThreadMain()
{
	if (meshData_segTmp.size() > 0)
	{
		for (vector <VCGMeshContainer*>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
		{
			(*mI)->ClearMesh();
		}
		meshData_segTmp.clear();
	}
	//UNCOMMENT FRO HERE FOR SEGMENTATION
	std::vector<float> startingVertices;
	std::vector<GLuint> startingIndices;
	std::vector<float> startingNormals;

	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;
	if (meshData.size() == 1)
	{
		meshData[0]->CleanAndParse(startingVertices, startingIndices, startingNormals);
	}

	else
	{
		switch (openGLWin.testMode)
		{
		case 0:
			//CombineAndExport();
			CleanAndParse("data\\models\\output.ply", startingVertices, startingIndices, startingNormals);
			break;
		case 1:
			CleanAndParse("data\\models\\testScene.ply", startingVertices, startingIndices, startingNormals);
			break;
		case 2:
			CleanAndParse("data\\models\\cube.ply", startingVertices, startingIndices, startingNormals);
			break;
		}
	}
	//CleanAndParse("data\\models\\output.ply", startingVertices, startingIndices, startingNormals);
	cDebug::DbgOut(L"Parsed");
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	pclSegmenter.ConvertToCloud(startingVertices, startingIndices, startingNormals);
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Converted to cloud in ", elapsedTime);

	//pclSegmenter.PlaneSegmentation();
	//pclSegmenter.EuclideanSegmentation();


	if (openGLWin.segmentationMode == REGION_GROWTH_SEGMENTATION)
	{
		if (!showColoredSegments || openGLWin.previewMode || openGLWin.segmentValuesChanged)
		{
			pclSegmenter.RegionGrowingSegmentation();
			openGLWin.segmentValuesChanged = false;
		}
		if (openGLWin.previewMode)
		{
			showColoredSegments = true;
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
	VCGMeshContainer *mesh;
	cDebug::DbgOut(L"Number of clusters: ", pclSegmenter.GetClusterCount());
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	for (int i = 0; i < pclSegmenter.GetClusterCount(); i++)
	{
		std::vector<float> clusterVertices;
		std::vector<GLuint> clusterIndices;
		pclSegmenter.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices);
		//cDebug::DbgOut(L"Converted Triangle Mesh #",i);
		mesh = new VCGMeshContainer;
		mesh->SetColorCode(i + 1);
		//mesh->ParseData(clusterVertices, clusterIndices);
		mesh->ConvertToVCG(clusterVertices, clusterIndices);
		mesh->CleanMesh();
		mesh->ParseData();
		meshData_segTmp.push_back(mesh);
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Filled MeshData in ", elapsedTime);
	//UNCOMMENT UNTIL HERE FOR SEGMENTATION
	showColoredSegments = false;
	segFinished = true;
	std::wstringstream ws;
	ws << L"Segmented mesh in ";
	ws << pclSegmenter.GetClusterCount();
	ws << L" clusters.";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
	return 0;

}

void StartSegmentation()
{
	segmentationThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&SegThreadMain, 0, 0, &sThreadId);
}

void InitializeSegments()
{
	segVertices.clear();
	//segIndices[pclSegmenter.coloredSegmentedCloud->points.size()] = {};
	for (int i = 0; i < pclSegmenter.coloredSegmentedCloud->points.size(); i++)
	{
		segVertices.push_back(pclSegmenter.coloredSegmentedCloud->points[i].x);
		segVertices.push_back(pclSegmenter.coloredSegmentedCloud->points[i].y);
		segVertices.push_back(pclSegmenter.coloredSegmentedCloud->points[i].z);
		segVertices.push_back(pclSegmenter.coloredSegmentedCloud->points[i].r / 255.0f);
		segVertices.push_back(pclSegmenter.coloredSegmentedCloud->points[i].g / 255.0f);
		segVertices.push_back(pclSegmenter.coloredSegmentedCloud->points[i].b / 255.0f);
		//segIndices[i] = i;
	}

	if (!initSegmentVBO)
	{
		glGenBuffers(1, &segmentVBO);
		glGenVertexArrays(1, &segmentVAO);
		initSegmentVBO = true;
	}
	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glBufferData(GL_ARRAY_BUFFER, segVertices.size() * sizeof(float), &segVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(segmentVAO);

	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));

	std::wstringstream ws;
	ws << L"Showing region growth segmentation preview with ";
	ws << pclSegmenter.GetClusterCount();
	ws << L" clusters.";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
	initColoredSegments = true;
	pclSegmenter.coloredCloudReady = false;
}

void RenderSegments()
{
	SetBackgroundColor(0, 0, 0);
	//glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	//if (!openGLWin.initPreview)
	//	InitializeSegments();
	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	shaderColor.UseProgram();
	shaderColor.SetUniform("matrices.projectionMatrix", openGLWin.glControl.GetProjectionMatrix());
	shaderColor.SetUniform("matrices.viewMatrix", glCamera.GetViewMatrix());
	glm::mat4 modelMatrix = glm::mat4(1.0);

	shaderColor.SetUniform("matrices.modelMatrix", modelMatrix);

	glBindVertexArray(segmentVAO);
	//glPointSize(1.5f);
	glDrawArrays(GL_POINTS, 0, segVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);

	glText.PrepareForRender();
	glText.RenderText(L"Clusters: ", pclSegmenter.GetClusterCount(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);

	if (openGLWin.freeCameraControls)
		glCamera.Update();

	glEnable(GL_DEPTH_TEST);

	//swap buffers to actually display the changes
	openGLWin.glControl.SwapBuffers();
}

void LoadClusterData()
{
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData.clear();
	for (vector <VCGMeshContainer*>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
		//if (tmp != selectedIndex-1)
		meshData.push_back(*mI);
		//meshData.(*mI)->Draw();
		//tmp++;
	}

	/*for (vector <VCGMeshContainer*>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
	{
	(*mI)->ClearMesh();
	}*/
	meshData_segTmp.clear();

	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		//(*mI)->ConvertToVCG();
		(*mI)->GenerateBOs();
		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfIndices() / 3;

	}
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateVAO();

	}

	segFinished = false;
	showColoredSegments = false;
	pclSegmenter.coloredCloudReady = false;
}

void ShowPCLViewer()
{
	if (pclSegmenter.GetClusterCount() > 0)
		pclSegmenter.ShowViewer();
}

#pragma endregion segmentation related

#pragma region
void FillHoles()
{
	int cnt = 0;
	int holeCnt = 0;
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		if ((*mI)->GetNumberOfVertices() <= 1000)
		{
			cDebug::DbgOut(L"no hole filling #", cnt);
			continue;
		}
		cDebug::DbgOut(L"fill hole #", cnt);
		holeCnt += (*mI)->FillHoles(openGLWin.holeSize * 100);

		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfIndices() / 3;

		std::wstringstream ws;
		ws << L"Filling holes... ";
		ws << cnt + 1;
		ws << L"% of ";
		ws << meshData.size();
		wstring statusMsg(ws.str());
		const TCHAR *c_str = statusMsg.c_str();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
	}
	std::wstringstream ws;
	ws << L"Filled ";
	ws << holeCnt;
	ws << L" holes in ";
	ws << cnt;
	ws << L" segments.";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
}

//unfortunately not yet working
void RemoveSmallComponents()
{
	int cnt = 0;
	int cmpCnt = 0;
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		cDebug::DbgOut(L"remove components #", cnt);
		cmpCnt += (*mI)->RemoveSmallComponents(300);

		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfIndices() / 3;
	}
	std::wstringstream ws;
	ws << L"Deleted ";
	ws << cmpCnt;
	ws << L" components";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
}
#pragma endregion mesh optimizations

void Initialize(LPVOID lpParam)
{
	VCGMeshContainer *mesh = new VCGMeshContainer;
	mesh->SetColorCode(1);

	switch (openGLWin.testMode)
	{
	case 0: 
		mesh->LoadMesh("data\\models\\output.ply");
		break;
	case 1:
		mesh->LoadMesh("data\\models\\testScene.ply");
		break;
	case 2:
		mesh->LoadMesh("data\\models\\cube.ply");
		break;
	}
	meshData.push_back(mesh);

	if (!PrepareShaderPrograms())
	{
		PostQuitMessage(0);
		return;
	}

	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateBOs();
		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfIndices() / 3;
		
	}
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->GenerateVAO();

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
	glClearColor(bgRed, bgGreen, bgBlue, 1.0f);
	glClearDepth(1.0);

	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();

	// Typecast lpParam to OpenGLControl pointer
	OpenGLControl* oglControl = (OpenGLControl*)lpParam;
	oglControl->ResizeOpenGLViewportFull(w, h);
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (wireFrameMode && !showColoredSegments)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else 
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//START SEGMENT
	if (segFinished)
	{
		LoadClusterData();
	}

	if (showColoredSegments)
	{
		if (pclSegmenter.coloredCloudReady)
			InitializeSegments();
		if (initColoredSegments)
		{
			RenderSegments();
			return;
		}	
	}
	//END SEGMENT

	//if user clicks with left mouse button -> picking/placing
	if (Keys::GetKeyStateOnce(VK_LBUTTON) && !openGLWin.freeCameraControls && openGLWin.IsMouseInOpenGLWindow())
	{
		if (selectedIndex == -1)
		{
			ProcessPicking();
		}
		else
		{
			if (openGLWin.colorSelection)
				ProcessPicking();
			else
				ProcessPlacing();
		}
	}

	//process currently selected object (attach to cursor, rotation/scaling etc.)
	if (selectedIndex != -1 && !openGLWin.colorSelection)
	{
		ProcessSelectedObject();
	}

	//draw every mesh
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->Draw();
	}

	if (openGLWin.helpingVisuals && firstRayCast)
	{
		RenderHelpingVisuals();
	}

	//show bounding boxes on/off
	if (showBB)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		int cnt = 0;
		for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
		{
			if (cnt != wallIndex-1)
				(*mI)->DrawBB();
			cnt++;
		}
	}

	if (openGLWin.freeCameraControls)
		glCamera.Update();

	//render info text on screen
	glText.PrepareForRender();
	glText.RenderText(L"FPS: ", oglControl->GetFPS(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Meshs: ", meshData.size(), 15, -0.98f, 0.75f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Verts: ", numberOfVertices, 15, -0.98f, 0.70f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Faces: ", numberOfFaces, 15, -0.98f, 0.65f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Sel: ", selectedIndex - 1, 15, -0.98f, 0.6f, 2.0f / w, 2.0f / h);
	

	//handle keyboard input

	//quit window/thread
	if (Keys::GetKeyStateOnce(VK_ESCAPE))
		PostQuitMessage(0);

	//toggle show bounding box mode
	if (Keys::GetKeyStateOnce('B'))
		ToggleBoundingBoxes();

	//toggle wireframe mode
	if (Keys::GetKeyStateOnce('Q'))
		ToggleWireFrame();

	//save combined mesh
	if (Keys::GetKeyStateOnce('L'))
	{
		CombineAndExport();
	}
	if (Keys::GetKeyStateOnce('P'))
	{
		ResetCameraPosition();
	}

	glEnable(GL_DEPTH_TEST);

	//swap buffers to actually display the changes
	oglControl->SwapBuffers();
}

void Release(LPVOID lpParam)
{
	shaderColor.DeleteProgram();
	shaderFont.DeleteProgram();
	glText.CleanUp();
	glDeleteBuffers(1, &segmentVBO);
	glDeleteBuffers(1, &rayVBO);
	glDeleteBuffers(1, &pointVBO);
	glDeleteVertexArrays(1, &segmentVAO);
	glDeleteVertexArrays(1, &rayVAO);
	glDeleteVertexArrays(1, &pointVAO);
	for (int i = 0; i < 4; i++)shaders[i].DeleteShader();
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
}
