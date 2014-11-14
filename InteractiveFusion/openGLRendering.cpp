#include "common.h"
#include "OpenGLCamera.h"
#include "SegmentationHelper.h"
#include "openGLShaders.h"
#include "SelectionHelper.h"
#include "VisualizationHelper.h"
#include "OpenGLText.h"
#include "InteractiveFusion.h"
#include "MeshHelper.h"
#include "VcgMeshContainer.h"

#pragma region

wstring statusMsg = L"";
//mesh storage
std::vector<shared_ptr<VCGMeshContainer>> meshData;

//gl related objects
SelectionHelper glSelector;
OpenGLText glText;
OpenGLCamera glCamera;
VisualizationHelper glHelper;
SegmentationHelper glSegmentation;

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

void StartSegmentation()
{
	glSegmentation.StartSegmentation();
}


#pragma endregion calls from UI


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

void InitialLoading()
{
	switch (openGLWin.testMode)
	{
	case 0:
		meshHelper.InitialLoadFromFile("data\\models\\output.ply");
		break;
	case 1:
		meshHelper.InitialLoadFromFile("data\\models\\testScene.ply");
		break;
	case 2:
		meshHelper.InitialLoadFromFile("data\\models\\cube.ply");
		break;
	}
	glCamera.SetRotationPoint(meshHelper.GetCombinedCenterPoint());


	//openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
	
	openGLWin.segmentationMode = EUCLIDEAN_SEGMENTATION;
	openGLWin.previewMode = false;
	glSegmentation.StartSegmentation();

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
		meshHelper.GenerateBuffers();
		openGLWin.state = DEFAULT;
		return;
	}

	//USER SELECTS WALL OR SEES SOME OTHER PREVIEW
	if (openGLWin.state == WALL_SELECTION || openGLWin.state == SEGMENTATION_PREVIEW)
	{
		if (!glSegmentation.IsPreviewInitialized())
		{
			if (openGLWin.state == SEGMENTATION_PREVIEW)
			{
				if (glSegmentation.IsCloudReady())
					glSegmentation.InitializePreview();
				else
					return;
			}
			glSegmentation.InitializePreview();
		}
		else
		{
			glSegmentation.RenderPreview();
			glCamera.mode = CAMERA_FREE;
		}

		openGLWin.glControl.SwapBuffers();
		return;
	}
	else if (glSegmentation.IsPreviewInitialized())
	{
		glSegmentation.ClearPreviewVertices();
		glCamera.ResetCameraPosition();
		glCamera.mode = CAMERA_SENSOR;
	}

	//SEGMENTATION IS FINISHED, LOAD RELEVANT DATA
	if (openGLWin.state == SEGMENTATION_FINISHED)
	{
		glSegmentation.LoadClusterData();
		glCamera.SetRotationPoint(meshHelper.GetCombinedCenterPoint());
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
	meshHelper.DrawAll();
	

	//draw helper visuals
	if (openGLWin.helpingVisuals && glSelector.firstRayCast)
	{
		glHelper.RenderHelpingVisuals();
	}

	//show bounding boxes on/off
	if (openGLWin.showBB)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		meshHelper.DrawAllForColorPicking();
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
	glSegmentation.CleanUp();

	for (int i = 0; i < 4; i++)shaders[i].DeleteShader();

	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData.clear();
	

}
