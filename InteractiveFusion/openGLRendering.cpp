#include "common.h"

#include "colorCoding.h"
#include "openGLCamera.h"
#include "openGLShaders.h"
#include "openGLText.h"
#include "openGLWin.h"
#include "PCLProcessing.h"
#include <boost/timer.hpp>
#include "Resource.h"
#include "vcgMesh.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#pragma region

//thread related
HANDLE segmentationThread;
DWORD sThreadId;
boost::thread preview_Thread;

bool previewThreadActive = false;
bool initialLoading = false;
wstring statusMsg = L"";
//mesh storage
std::vector<VCGMeshContainer*> meshData;
std::vector<VCGMeshContainer*> meshData_segTmp;

std::vector<float> startingVertices;
std::vector<GLuint> startingIndices;
std::vector<float> startingNormals;


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
std::vector<float> wallVertices;
std::vector<float> rayVertices;
std::vector<float> pointVertices;

//objects for helper visualizations
glm::vec3 hitPoint{ -1, -1, -1 };
glm::vec3 nearPoint;


//background color
float bgRed = 0.0f; 
float bgGreen = 0.0f;
float bgBlue = 0.0f;

//for info text
int numberOfVertices = 0;
int numberOfFaces = 0;

//currently selected object (or -1 if none selected)
int selectedIndex = -1;

//index of wall object
int wallIndex = -1;

int previewIndex = -1;

int dotCount = 0;
const std::vector<wstring> dots = { L".", L"..", L"..." };

//first time something happened flags
bool firstRayCast = false;

//initialized flags
bool initSegmentVBO = false;
bool initRayVBO = false;
bool initPointVBO = false;
bool initColoredSegments = false;
bool initWallSegment = false;

//toggle different modes
bool showBB = false;
bool wireFrameMode = false;
bool snapToVertex = false;
bool placeWithRaycast = false;

//segmentation flags
bool segFinished = false;
bool showColoredSegments = false;
bool segmenting = false;

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

void ToggleRaycastPlacing()
{
	placeWithRaycast = !placeWithRaycast;
}

void ToggleSnapToVertex()
{
	snapToVertex = !snapToVertex;
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
	if (colorByte[0] == bgRed && colorByte[1] == bgGreen && colorByte[2] == bgBlue)
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

void GetRayOrientation(glm::vec3 v1, glm::vec3 v2, glm::vec3 normal, std::vector<int> &orientation)
{
	orientation.push_back(0);
	orientation.push_back(0);
	orientation.push_back(0);
	if (abs(normal.z) > 0.5f)
	{
		if (abs(v2.z - v1.z) > 700)
		{
			if (v2.z < v1.z)
				orientation[2] = 1;
			else
				orientation[2] = -1;
		}
	}
	if (normal.y > 0.5f)
		orientation[1] = 1;
	//else if (normal.y < -0.5f)
	//	orientation[1] = -1;
	//else
		orientation[1] = 1;

	if (abs(normal.x) > 0.5f)
	{
		if (abs(v2.x - v1.x) > 900)
		{
			if (v2.x < v1.x)
				orientation[0] = 1;
			else
				orientation[0] = -1;
		}
	}
}

void DeleteMesh(int index)
{
	numberOfVertices -= meshData[index]->GetNumberOfVertices();
	numberOfFaces -= meshData[index]->GetNumberOfIndices();
	meshData[index]->ClearMesh();
	meshData.erase(std::remove(meshData.begin(), meshData.end(), meshData[index]), meshData.end());
}

int DuplicateMesh(int index)
{
	VCGMeshContainer* mesh = new VCGMeshContainer;
	mesh->SetColorCode(meshData.size() + 1);

	mesh->ConvertToVCG(meshData[index]->GetVertices(), meshData[index]->GetIndices());
	mesh->ParseData();
	mesh->GenerateBOs();
	mesh->GenerateVAO();
	numberOfVertices += mesh->GetNumberOfVertices();
	numberOfFaces += mesh->GetNumberOfIndices() / 3;
	meshData.push_back(mesh);
	return meshData.size();
}

bool PlacingPreview()
{
	glm::vec3 tmpNormal;
	if (!placeWithRaycast)
	{
		bool result = false;
		std::wstringstream ws;
		int cnt = 1;
		for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
		{
			if (cnt != selectedIndex)
				(*mI)->DrawBB();
			cnt++;
		}
		int tmpIndex = GetColorUnderCursor();
		if (tmpIndex > -1 && tmpIndex <= meshData.size())
		{
			ws << L"Selecting object #";
			ws << tmpIndex;
			glm::vec3 v1, v2;
			RayCast(&v1, &v2);
			glm::vec3 tmpPoint;
			std::vector<int> orientation = meshData[tmpIndex - 1]->GetOrientation();
			
			
			meshData[tmpIndex - 1]->GetHitPoint(v1, v2, tmpPoint, tmpNormal,snapToVertex);
			GetRayOrientation(v1, v2, tmpNormal, orientation);
			hitPoint = tmpPoint;
			/*if (tmpPoint.z <= hitPoint.z || previewIndex == tmpIndex)
			{
				hitPoint = tmpPoint;
				previewIndex = tmpIndex;
			}*/
			meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint, orientation);
			meshData[selectedIndex - 1]->TogglePreviewSelection(true);
			result = true;
		}
		wstring statusMsg(ws.str());
		const TCHAR *c_str = statusMsg.c_str();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		return result;
	}
	else
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
				
				if ((*mI)->GetHitPoint(v1, v2, tmpHit, tmpNormal, snapToVertex))
				{
					//float cMaxZ = (*mI)->GetUpperBounds().z;
					float cMaxZ = tmpHit.z;
					//cDebug::DbgOut(L"Collision found", index);
					//cDebug::DbgOut(L"zWert ist ", cMaxZ);
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
			//cDebug::DbgOut(L"Collision chosen", sIndex);
			//hitPoint.z = meshData[sIndex]->GetUpperBounds().z;
			std::vector<int> orientation = meshData[sIndex - 1]->GetOrientation();
			GetRayOrientation(v1, v2, tmpNormal, orientation);
			meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint, orientation);
			meshData[selectedIndex - 1]->TogglePreviewSelection(true);
			return true;
		}
		return false;
	}
}

void ProcessSelectedObject()
{
	//while (previewThreadActive)
	//{
		//cDebug::DbgOut(L"ahoi", 1);
		//boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		if (!PlacingPreview())
		{
			meshData[selectedIndex - 1]->TogglePreviewSelection(false);
			
			glm::vec3 v1, v2;
			RayCast(&v1, &v2);
			meshData[selectedIndex - 1]->AttachToCursor(v1, v2, openGLWin.carryDistance);
		}
		if (Keys::GetKeyState(VK_DELETE))
		{
			DeleteMesh(selectedIndex-1);
			selectedIndex = -1;
			cDebug::DbgOut(L"pressed ENTF alright");
		}
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
	//}
}

void ProcessPicking()
{
	std::wstringstream ws;
	int cnt = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		//if (cnt != wallIndex - 1 && !(*mI)->IsWall())
			(*mI)->DrawBB();
		cnt++;
	}
	int tmpIndex = GetColorUnderCursor();
	if (tmpIndex > -1 && tmpIndex <= meshData.size())
	{
		if (meshData[tmpIndex - 1]->IsWall())
			return;
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
	glm::vec3 tmpNormal;
	if (!placeWithRaycast)
	{
		std::wstringstream ws;
		int cnt = 1;
		for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
		{
			if (cnt != selectedIndex)
				(*mI)->DrawBB();
			cnt++;
		}
		int tmpIndex = GetColorUnderCursor();
		if (tmpIndex > -1 && tmpIndex <= meshData.size())
		{
			ws << L"Selecting object #";
			ws << tmpIndex;
			glm::vec3 v1, v2;
			RayCast(&v1, &v2);
			glm::vec3 tmpPoint;
			
			std::vector<int> orientation = meshData[tmpIndex - 1]->GetOrientation();
			
			meshData[tmpIndex - 1]->GetHitPoint(v1, v2, tmpPoint, tmpNormal, snapToVertex);
			GetRayOrientation(v1, v2, tmpNormal, orientation);
			hitPoint = tmpPoint;
			/*if (tmpPoint.z <= hitPoint.z || previewIndex == tmpIndex)
			{
			hitPoint = tmpPoint;
			previewIndex = tmpIndex;
			}*/
			meshData[selectedIndex - 1]->SetSelected(false);
			meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint, orientation);
			if (Keys::GetKeyState('D'))
			{
				int newIndex = DuplicateMesh(selectedIndex - 1);
				selectedIndex = newIndex;
				meshData[selectedIndex - 1]->SetSelected(true);
				cDebug::DbgOut(L"pressed D alright");
			}
			else selectedIndex = -1;
		}
		else
		{
			meshData[selectedIndex - 1]->SetSelected(false);
			meshData[selectedIndex - 1]->ResetSelectedTransformation();
			selectedIndex = -1;
		}
		wstring statusMsg(ws.str());
		const TCHAR *c_str = statusMsg.c_str();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else
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
				if ((*mI)->GetHitPoint(v1, v2, tmpHit, tmpNormal, snapToVertex))
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
			std::vector<int> orientation = meshData[sIndex - 1]->GetOrientation();
			GetRayOrientation(v1, v2, tmpNormal, orientation);
			meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint, orientation);
			selectedIndex = -1;
		}
		else
		{
			meshData[selectedIndex - 1]->SetSelected(false);
			meshData[selectedIndex - 1]->ResetSelectedTransformation();
			selectedIndex = -1;
		}
	}
}

#pragma endregion picking, placing, raycast

#pragma region
void SelectWallObject()
{
	if (selectedIndex != -1)
	{
		meshData[selectedIndex - 1]->SetWall(true);
		wallIndex = selectedIndex;
		if (openGLWin.colorSelection)
			meshData[selectedIndex - 1]->ToggleSelectedColor(false);
		meshData[selectedIndex - 1]->SetSelected(false);
		selectedIndex = -1;
	}
}

void ResetWallObject()
{
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->SetWall(false);
	}
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
	glPointSize(10.0f);
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
	if (meshData_segTmp.size() > 0 && openGLWin.segmentValuesChanged)
	{
		int cnt = 0;
		for (int i = pclSegmenter.GetPlaneClusterCount(); i < meshData_segTmp.size(); i++)
		{
			meshData_segTmp[i]->ClearMesh();
		}
		meshData_segTmp.erase(meshData_segTmp.begin() + pclSegmenter.GetPlaneClusterCount(), meshData_segTmp.end());
		/*for (vector <VCGMeshContainer*>::iterator mI = meshData_segTmp.begin(); mI != meshData_segTmp.end(); ++mI)
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
			meshData[0]->CleanAndParse(startingVertices, startingIndices, startingNormals);
	}

	else
	{
		std::wstringstream ws;
		ws << L"Can't segment mesh, as it is already segmented.";
		wstring statusMsg(ws.str());
		const TCHAR *c_str = statusMsg.c_str();
		SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
		return 0;
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
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	if (!pclSegmenter.IsMainCloudInitialized())
	{
		statusMsg = L"Converting mesh to point cloud";
		pclSegmenter.ConvertToCloud(startingVertices, startingIndices, startingNormals);
	}
	QueryPerformanceCounter(&t2);
	elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	cDebug::DbgOut(L"Converted to cloud in ", elapsedTime);

	//pclSegmenter.PlaneSegmentation();
	//pclSegmenter.EuclideanSegmentation();
	cDebug::DbgOut(L"before plane segmentation ", elapsedTime);
	VCGMeshContainer *mesh;

	if (!pclSegmenter.IsPlaneSegmented())
	{
		pclSegmenter.PlaneSegmentation();
		pclSegmenter.PlaneIndexEstimation();
		for (int i = 0; i < pclSegmenter.GetPlaneClusterCount(); i++)
		{
			std::vector<float> clusterVertices;
			std::vector<GLuint> clusterIndices;
			pclSegmenter.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices);
			cDebug::DbgOut(L"Converted Plane Mesh #", i);
			mesh = new VCGMeshContainer;
			mesh->SetColorCode(i + 1);
			//mesh->ParseData(clusterVertices, clusterIndices);
			mesh->SetWall(true);
			
			wallIndex = i + 1;
			mesh->ConvertToVCG(clusterVertices, clusterIndices);
			
			//int total = mesh->MergeCloseVertices(0.005f);
			//cDebug::DbgOut(_T("Merged close vertices: "), total);
			//cDebug::DbgOut(L"indices: ", (int)mesh->GetVertices().size());
			//cDebug::DbgOut(L"vertices: " + (int)mesh->GetIndices().size());
			mesh->RemoveSmallComponents(1000);
			mesh->CleanMesh();
			mesh->RemoveNonManifoldFace();
			mesh->FillHoles((clusterVertices.size()*6)/10);
			mesh->CleanMesh();
			mesh->ParseData();
			mesh->SetPlaneParameters(pclSegmenter.planeCoefficients[i]->values[0], pclSegmenter.planeCoefficients[i]->values[1],
				pclSegmenter.planeCoefficients[i]->values[2], pclSegmenter.planeCoefficients[i]->values[3]);
			//cDebug::DbgOut(L"vertices: ", (int)mesh->GetNumberOfVertices());
			//cDebug::DbgOut(L"indices: " + (int)mesh->GetNumberOfIndices());
			meshData_segTmp.push_back(mesh);
		}
	}

	if (openGLWin.segmentationMode == REGION_GROWTH_SEGMENTATION)
	{
		if (!showColoredSegments || openGLWin.previewMode || openGLWin.segmentValuesChanged)
		{
			
			//meshData_segTmp[0]->CleanAndParse(startingVertices, startingIndices, startingNormals);
			//pclSegmenter.ConvertToCloud(startingVertices, startingIndices, startingNormals);
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
	cDebug::DbgOut(L"Number of clusters: ", pclSegmenter.GetRegionClusterCount());
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	QueryPerformanceCounter(&t2);
	for (int i = 0; i < pclSegmenter.GetRegionClusterCount(); i++)
	{
		std::vector<float> clusterVertices;
		std::vector<GLuint> clusterIndices;
		pclSegmenter.ConvertToTriangleMesh(i, startingVertices, clusterVertices, clusterIndices);
		//cDebug::DbgOut(L"Converted Triangle Mesh #",i);
		mesh = new VCGMeshContainer;
		mesh->SetColorCode(i+pclSegmenter.GetPlaneClusterCount() + 1);
		//mesh->ParseData(clusterVertices, clusterIndices);
		mesh->ConvertToVCG(clusterVertices, clusterIndices);
		
		mesh->CleanMesh();
		mesh->RemoveNonManifoldFace();
		
		mesh->FillHoles((clusterVertices.size() * 6) / 10);
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
	ws << pclSegmenter.GetRegionClusterCount();
	ws << L" clusters.";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);

	return 0;

}

void StartSegmentation()
{
	segmenting = true;
	segmentationThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&SegThreadMain, 0, 0, &sThreadId);
}

void InitializeWallSegment()
{
	wallVertices.clear();
	//segIndices[pclSegmenter.coloredSegmentedCloud->points.size()] = {};
	for (int i = 0; i < pclSegmenter.wallSegmentCloud->points.size(); i++)
	{
		wallVertices.push_back(pclSegmenter.wallSegmentCloud->points[i].x);
		wallVertices.push_back(pclSegmenter.wallSegmentCloud->points[i].y);
		wallVertices.push_back(pclSegmenter.wallSegmentCloud->points[i].z);
		wallVertices.push_back(pclSegmenter.wallSegmentCloud->points[i].r / 255.0f);
		wallVertices.push_back(pclSegmenter.wallSegmentCloud->points[i].g / 255.0f);
		wallVertices.push_back(pclSegmenter.wallSegmentCloud->points[i].b / 255.0f);
		//segIndices[i] = i;
	}
	//cDebug::DbgOut(L"init.. points: ", (int)pclSegmenter.wallSegmentCloud->points.size());
	if (!initSegmentVBO)
	{
		glGenBuffers(1, &segmentVBO);
		glGenVertexArrays(1, &segmentVAO);
		initSegmentVBO = true;
	}
	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glBufferData(GL_ARRAY_BUFFER, wallVertices.size() * sizeof(float), &wallVertices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(segmentVAO);

	glBindBuffer(GL_ARRAY_BUFFER, segmentVBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(0));
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float)* 6, reinterpret_cast<void*>(sizeof(float)* 3));

	std::wstringstream ws;
	ws << L"Showing wall preview ";
	wstring statusMsg(ws.str());
	const TCHAR *c_str = statusMsg.c_str();
	SetDlgItemText(openGLWin.glWindowParent, IDC_IM_STATUS, c_str);
	initWallSegment = true;
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

void RenderWallSegment()
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
	glPointSize(1.5f);
	glDrawArrays(GL_POINTS, 0, wallVertices.size());
	glBindVertexArray(0);

	glUseProgram(0);

	glText.PrepareForRender();
	glText.RenderText(L"Clusters: ", pclSegmenter.GetClusterCount(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);

	//openGLWin.freeCameraControls = true;
	//if (openGLWin.freeCameraControls)
	//	glCamera.Update();

	glEnable(GL_DEPTH_TEST);

	//swap buffers to actually display the changes
	//openGLWin.glControl.SwapBuffers();
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

/*void ShowPCLViewer()
{
	if (pclSegmenter.GetClusterCount() > 0)
		pclSegmenter.ShowViewer();
}*/

#pragma endregion segmentation related

#pragma region

/*
void MLS()
{
	std::vector<float> startingVertices;
	std::vector<GLuint> startingIndices;
	std::vector<float> startingNormals;
	VCGMeshContainer *mesh;

	if (meshData.size() == 1)
	{
		
		meshData_segTmp.push_back(mesh);
		pclSegmenter.MovingLeastSquares(meshData[0]->GetVertices(), meshData[0]->GetNormals(), startingVertices, startingIndices, startingNormals);
		meshData[0]->ClearMesh();
		meshData.clear();
		//meshData[0]->LoadMesh(startingVertices, startingIndices, startingNormals);
		mesh = new VCGMeshContainer;
		mesh->SetColorCode(1);
		//mesh->ParseData(clusterVertices, clusterIndices);
		mesh->ConvertToVCG(startingVertices, startingIndices);
		mesh->CleanMesh();
		mesh->ParseData();
		meshData.push_back(mesh);
		//meshData[0]->LoadMesh(startingVertices, startingIndices, startingNormals);
	}

	numberOfVertices = 0;
	numberOfFaces = 0;
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
}*/

void FillHoles(int holeSize)
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
			numberOfVertices += (*mI)->GetNumberOfVertices();
			numberOfFaces += (*mI)->GetNumberOfIndices() / 3;
			cDebug::DbgOut(L"no hole filling #", cnt);
			continue;
		}
		cDebug::DbgOut(L"fill hole #", cnt);
		(*mI)->ConvertToVCG();
		holeCnt += (*mI)->FillHoles(holeSize * 100);
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
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

void RemoveSmallComponents(int size)
{
	int cnt = 0;
	int cmpCnt = 0;
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		cnt++;
		cDebug::DbgOut(L"remove components #", cnt);
		cmpCnt += (*mI)->RemoveSmallComponents(size);
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();
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

void CleanMesh()
{
	numberOfVertices = 0;
	numberOfFaces = 0;
	for (vector <VCGMeshContainer*>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->CleanMesh();
		(*mI)->ParseData();
		(*mI)->UpdateBuffers();

		numberOfVertices += (*mI)->GetNumberOfVertices();
		numberOfFaces += (*mI)->GetNumberOfIndices() / 3;
	}
}
#pragma endregion mesh optimizations

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

void ShowWallMsg()
{
	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	wstring wallString = L"Is this (part of) a floor/wall?";

	glText.PrepareForRender();
	glText.RenderText(wallString, 25, -0.1f, -0.4f, 2.0f / w, 2.0f / h);
}

void LoadInput()
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

		//mesh->LoadMesh("data\\models\\testScene.ply");
		break;
	case 2:
		mesh->LoadMesh("data\\models\\cube.ply");
		break;
	}
	meshData.push_back(mesh);

	//openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
	openGLWin.segmentationMode = EUCLIDEAN_SEGMENTATION;
	openGLWin.previewMode = false;
	StartSegmentation();
}

void GenerateBuffers()
{
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
	//quit window/thread
	if (Keys::GetKeyStateOnce(VK_ESCAPE))
		PostQuitMessage(0);
	//toggle wireframe mode
	if (Keys::GetKeyStateOnce(VK_F10))
	{
		ToggleDebugControls();
	}
	//clear window
	glClearColor(bgRed, bgGreen, bgBlue, 1.0f);
	glClearDepth(1.0);

	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	openGLWin.glControl.ResizeOpenGLViewportFull(w, h);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (wireFrameMode && !showColoredSegments)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else 
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	if (openGLWin.wallSelection)
	{
		if (!initWallSegment)
			InitializeWallSegment();
		//statusMsg = L"Is this a wall?";
		
		if (initWallSegment)
		{
			RenderWallSegment();
		}
		//ShowWallMsg();
		openGLWin.glControl.SwapBuffers();
		return;
	}
	else if (initWallSegment)
	{
		initWallSegment = false;
		openGLWin.freeCameraControls = false;
	}

	if (!initialLoading)
	{
		if (meshData.size() == 0)
		{
			ShowStatusMsg();
			openGLWin.glControl.SwapBuffers();
			return;
		}
		else if (!initialLoading)
		{
			GenerateBuffers();
			initialLoading = true;
		}
	}

	//START SEGMENT
	if (segFinished)
	{
		//cDebug::DbgOut(L"finished?", 1);
		segmenting = false;
		LoadClusterData();
	}
	if (segmenting)
	{
		//cDebug::DbgOut(L"segmenting", 1);
		ShowStatusMsg();
		openGLWin.glControl.SwapBuffers();
		return;
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
		/*if (!previewThreadActive)
		{
			previewThreadActive = true;
			if (!preview_Thread.joinable())
				preview_Thread = boost::thread(ProcessSelectedObject);
		}*/
		ProcessSelectedObject();
	}
	//else previewThreadActive = false;

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
			if (cnt != wallIndex-1 && !(*mI)->IsWall())
				(*mI)->DrawBB();
			cnt++;
		}
	}

	if (openGLWin.freeCameraControls)
		glCamera.Update();

	//render info text on screen
	glText.PrepareForRender();
	glText.RenderText(L"FPS: ", openGLWin.glControl.GetFPS(), 20, -0.98f, 0.85f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Meshs: ", meshData.size(), 15, -0.98f, 0.75f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Verts: ", numberOfVertices, 15, -0.98f, 0.70f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Faces: ", numberOfFaces, 15, -0.98f, 0.65f, 2.0f / w, 2.0f / h);
	glText.RenderText(L"Sel: ", selectedIndex - 1, 15, -0.98f, 0.6f, 2.0f / w, 2.0f / h);
	

	//handle keyboard input

	
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
	openGLWin.glControl.SwapBuffers();
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
