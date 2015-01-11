#include "common.h"
#include "OpenGLCamera.h"
#include "SegmentationHelper.h"
#include "OpenGLShaders.h"
#include "SelectionHelper.h"
#include "VisualizationHelper.h"
#include "OpenGLText.h"
#include "InteractiveFusion.h"
#include "MeshHelper.h"
#include "VcgMeshContainer.h"
#include "Keys.h"
#include "OpenGL2DHelper.h"

#pragma region

wstring statusMsg = L"";
wstring percentMsg = L"";
wstring helpMsg = L"";

glm::vec3 upperPoint = glm::vec3(999.0f, 999.0f, 999.0f);
glm::vec3 lowerPoint = glm::vec3(-999.0f, -999.0f, -999.0f);

glm::vec3 planeCenter;

bool showMinCut = false;

int currFPS = -1;
//mesh storage
std::vector<shared_ptr<VCGMeshContainer>> meshData;
shared_ptr<VCGMeshContainer> originalMesh(new VCGMeshContainer);
//gl related objects
SelectionHelper glSelector;
OpenGLText glText;
OpenGLCamera glCamera;
VisualizationHelper glHelper;
SegmentationHelper glSegmentation;
OpenGL2DHelper gl2DHelper;
int storedWidth = 0;
int storedHeight = 0;
WindowMode previousMode = IF_MODE_PREPARE_SCAN;

//for status messages (appear 'busy' to the user)
int dotCount = 0;
const std::vector<wstring> dots = { L".", L"..", L"..." };

#pragma endregion variables

#pragma region
void RemoveSelectionColor()
{
	if (glSelector.selectedIndex != -1)
	{
		meshData[glSelector.selectedIndex]->ToggleSelectedColor(false);
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


void SetViewportStatusMessage(wstring message)
{
	statusMsg = message;
}

void SetViewportPercentMsg(wstring percent)
{
	percentMsg = percent;
}


void ShowWallMessage()
{
	glDisable(GL_DEPTH_TEST);
	if (!gl2DHelper.IsRectangleInitialized())
		gl2DHelper.InitializeRectangle();

	if (openGLWin.GetDeviceClass() == IF_DEVICE_PC)
		gl2DHelper.DrawRectangle(0.7f, 0.8f, 0.2f);
	else if (openGLWin.GetDeviceClass() == IF_DEVICE_TABLET)
		gl2DHelper.DrawRectangle(0.64f, 0.8f, 0.25f);

	wstring wallMessage = L"Is the highlighted area (part of) a wall, floor or ceiling?";

	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	float xPos = 0.0f - wallMessage.length() * 0.008f;

	glText.PrepareForRender();
	if (openGLWin.GetDeviceClass() == IF_DEVICE_PC)
		glText.RenderText(wallMessage, 30, xPos, 0.67f, 2.0f / w, 2.0f / h);
	else if (openGLWin.GetDeviceClass() == IF_DEVICE_TABLET)
		glText.RenderText(wallMessage, 30, xPos, 0.60f, 2.0f / w, 2.0f / h);
	glEnable(GL_DEPTH_TEST);
}

void ShowHelpDialog()
{
	glDisable(GL_DEPTH_TEST);
	if (!gl2DHelper.IsRectangleInitialized())
		gl2DHelper.InitializeRectangle();

	gl2DHelper.DrawRectangle(0.0f, 0.5f, 3.0f);

		gl2DHelper.DrawRectangle();

	wstring helpMessage = L"Please do something and be happy about it.";

	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	float xPos = 0.0f - helpMessage.length() * 0.008f;

	glText.PrepareForRender();
	glText.RenderText(helpMessage, 30, xPos, 0.05f, 2.0f / w, 2.0f / h);

	glEnable(GL_DEPTH_TEST);
}

void ShowStatusMsg()
{
	glDisable(GL_DEPTH_TEST);
	if (!gl2DHelper.IsRectangleInitialized())
		gl2DHelper.InitializeRectangle();

	gl2DHelper.DrawRectangle(0.0f, 0.5f, 3.0f);

	if (openGLWin.GetDeviceClass() == IF_DEVICE_PC)
		gl2DHelper.DrawRectangle();
	else if (openGLWin.GetDeviceClass() == IF_DEVICE_TABLET)
		gl2DHelper.DrawRectangle(0.0f, 0.5f, 0.4f);
	
	int w = openGLWin.glControl.GetViewportWidth();
	int h = openGLWin.glControl.GetViewportHeight();
	float xPos = 0.0f - statusMsg.length() * 0.008f;
	wstring loadString = statusMsg + L"" + dots[(int)floor(dotCount / 100)];
	if (dotCount == 299)
		dotCount = 0;
	else
		dotCount++;

	glText.PrepareForRender();
	

	if (percentMsg.size() > 0)
	{
		if (glSegmentation.GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION)
		{
			glText.RenderText(loadString, 25, xPos, 0.05f, 2.0f / w, 2.0f / h);
			float xPercentPos = -0.05f;
			glText.RenderText(percentMsg, 20, xPercentPos, -0.11f, 2.0f / w, 2.0f / h);
		}
		else
		{
			glText.RenderText(loadString, 25, xPos, 0.05f, 2.0f / w, 2.0f / h);
			float xPercentPos = -0.05f;
			glText.RenderText(percentMsg, 20, xPercentPos, -0.08f, 2.0f / w, 2.0f / h);
		}
	}
	else
	{
		glText.RenderText(loadString, 25, xPos, 0.00f, 2.0f / w, 2.0f / h);
	}
	glEnable(GL_DEPTH_TEST);
}

void HandleKeyInput()
{
	if (Keys::GetKeyStateOnce(VK_F10))
	{
		openGLWin.ToggleDebugControls();
	}
	if (Keys::GetKeyStateOnce('P'))
	{
		if (glHelper.GetVisualizationMode() == IF_VISUALIZATION_NONE)
			glHelper.SetVisualizationMode(IF_VISUALIZATION_PLANE);
		else
			glHelper.SetVisualizationMode(IF_VISUALIZATION_NONE);
	}
}

void ResetForResume()
{
	glSegmentation.ClearForResume();
	meshData.clear();
	originalMesh->ClearMesh();
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

	gl2DHelper.InitialLoadFromFile("data\\models\\trash.ply", "data\\models\\trash_open.ply", TRASH_BIN_COLOR);
	glCamera.SetRotationPoint(meshHelper.GetCombinedCenterPoint());


	//openGLWin.segmentationMode = REGION_GROWTH_SEGMENTATION;
	
	//openGLWin.segmentationMode = EUCLIDEAN_SEGMENTATION;
	statusMsg = L"Segmenting mesh";
	
	//openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
	//openGLWin.SetWindowMode(IF_MODE_DEBUG);
	openGLWin.previewMode = true;
	glSegmentation.StartSegmentation();

	//openGLWin.SetWindowState(BUFFERS);
	
	
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

	glText.Initialize("data\\fonts\\OpenSans-Regular.ttf");

	glCamera = OpenGLCamera(glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), 1.0f, 0.1f);

	//ColorIF defaultColor = { 0.08f, 0.08f, 0.08f, 1.0f };
	//ColorIF pressedColor = { 0.2f, 0.2f, 0.2f, 1.0f };
	//gl2DHelper.InitializeButton(OPENGL_BUTTON_OK, 0.0f, -0.125f, 0.2f, 0.10f, defaultColor, pressedColor, L"ALRIGHT");
}

void RenderSegmentation()
{
	if (openGLWin.GetReset() == IF_RESET)
	{
		ResetForResume();
		openGLWin.SetReset(IF_NO_RESET);
	}
	if (!glSegmentation.IsPreviewInitialized())
	{
		if (openGLWin.GetWindowBusyState() != IF_BUSYSTATE_BUSY)
		{
			if (glSegmentation.IsCloudReady())
			{
				meshHelper.GenerateOriginalBuffers();
				glSegmentation.InitializePreview();
			}
		}

	}
	if (originalMesh->AreBuffersInitialized())
	{
		meshHelper.DrawOriginalMesh();

		/*	if (!glHelper.IsPlaneInitialized())
		glHelper.InitializePlane(originalMesh->GetLowerBounds(), originalMesh->GetUpperBounds());

		if (glHelper.GetVisualizationMode() == IF_VISUALIZATION_PLANE)
		glHelper.DrawPlane();*/
	}




	if (glSegmentation.GetSegmentationState() == IF_SEGSTATE_PLANE_SEGMENTATION && openGLWin.GetWindowBusyState() != IF_BUSYSTATE_BUSY)
		ShowWallMessage();

	if (glSegmentation.GetSegmentationState() == IF_SEGSTATE_FINISHED)
	{
		glSegmentation.LoadClusterData();
		glCamera.SetRotationPoint(meshHelper.GetCombinedCenterPoint());
		glCamera.ResetCameraPosition();
		glSegmentation.ResetInitializedStatus();
		glSegmentation.SetSegmentationState(IF_SEGSTATE_NONE);
		openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);

		openGLWin.SetWindowMode(IF_MODE_PROCESSING);
		//openGLWin.SetWindowMode(IF_MODE_MINCUT);

		//return;
	}
}

void CheckForChangedValues()
{
	if (storedWidth != openGLWin.glControl.GetViewportWidth() ||
		storedHeight != openGLWin.glControl.GetViewportHeight())
	{
		storedWidth = openGLWin.glControl.GetViewportWidth();
		storedHeight = openGLWin.glControl.GetViewportHeight();
		openGLWin.glControl.ResizeOpenGLViewportFull();
		openGLWin.glControl.SetProjection3D(45.0f, float(openGLWin.glControl.GetViewportWidth()) / float(openGLWin.glControl.GetViewportHeight()), 0.1f, 1000.0f);
	}

	if (previousMode != openGLWin.GetWindowMode())
	{
		previousMode = openGLWin.GetWindowMode();
		glSelector.Unselect();
	}
}

void RenderMinCut()
{
	if (!glHelper.IsPlaneInitialized())
		glHelper.InitializePlane(originalMesh->GetLowerBounds(), originalMesh->GetUpperBounds());


	if (glHelper.GetVisualizationMode() == IF_VISUALIZATION_PLANE)
	{
		glHelper.DrawPlane();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		//glDisable(GL_BLEND);
		//glDisable(GL_CULL_FACE);
		//glDisable(GL_DEPTH_TEST);
		glPointSize(5.0f);

		if (Keys::GetKeyState(VK_LBUTTON) &&
			openGLWin.IsMouseInOpenGLWindow() && glSelector.selectedIndex != -1)
		{
			planeCenter = glSelector.GetCursorRayCastHitPoint();
			glHelper.SetPlaneTranslation(planeCenter);

		}

		if (Keys::GetKeyStateOnce('S'))
		{

			meshHelper.SetVerticesInPlane(glSelector.selectedIndex, planeCenter);
		}
		//if (glSelector.selectedIndex != -1)
		//	meshHelper.Draw(glSelector.selectedIndex);

		meshHelper.DrawAll();
		/*if (Keys::GetKeyStateOnce(VK_LBUTTON) &&
		openGLWin.IsMouseInOpenGLWindow() && glSelector.selectedIndex != -1)
		{
		if (Keys::GetKeyState('F'))
		{
		bool success = false;
		glm::vec3 hit = glSelector.GetCursorRayCastHitPoint(glSelector.selectedIndex, success);
		if (success)
		{
		glSegmentation.AddMinCutForegroundPoint(hit);
		cDebug::DbgOut(L"Adding foreground point");
		}
		}
		else if (Keys::GetKeyState('B'))
		{
		bool success = false;
		glm::vec3 hit = glSelector.GetCursorRayCastHitPoint(glSelector.selectedIndex, success);
		if (success)
		{
		glSegmentation.AddMinCutBackgroundPoint(hit);
		cDebug::DbgOut(L"Adding background point");
		}
		}
		else if (Keys::GetKeyState('U'))
		{
		bool success = false;
		glm::vec3 hit = glSelector.GetCursorRayCastHitPoint(glSelector.selectedIndex, success);
		if (success)
		{
		upperPoint = hit;
		cDebug::DbgOut(L"Adding upper point");
		cDebug::DbgOut(L"uP x: ", upperPoint.x);
		cDebug::DbgOut(L"uP y: ", upperPoint.y);
		cDebug::DbgOut(L"uP z: ", upperPoint.z);

		if (lowerPoint.x != -999.0f)
		{
		glSegmentation.DetermineMinCutRadius(upperPoint, lowerPoint);
		}
		}
		}
		else if (Keys::GetKeyState('L'))
		{
		bool success = false;
		glm::vec3 hit = glSelector.GetCursorRayCastHitPoint(glSelector.selectedIndex, success);
		if (success)
		{
		lowerPoint = hit;
		cDebug::DbgOut(L"Adding lower point");
		cDebug::DbgOut(L"lower x: ", lowerPoint.x);
		cDebug::DbgOut(L"lower y: ", lowerPoint.y);
		cDebug::DbgOut(L"lower z: ", lowerPoint.z);

		glSegmentation.ResetMinCutValues();
		glSegmentation.AddMinCutForegroundPoint(lowerPoint);
		if (upperPoint.x != 999.0f)
		{
		glSegmentation.DetermineMinCutRadius(upperPoint, lowerPoint);
		}
		}
		}

		}
		else if (Keys::GetKeyState('N'))
		{
		if (Keys::GetKeyStateOnce(VK_ADD))
		{
		openGLWin.minCutNeighbors += 1;
		openGLWin.ShowStatusBarMessage(L"minCutNeighbors: " + std::to_wstring(openGLWin.minCutNeighbors));
		cDebug::DbgOut(L"MinCutNeighbors: ", openGLWin.minCutNeighbors);
		}
		else if (Keys::GetKeyStateOnce(VK_SUBTRACT))
		{
		openGLWin.minCutNeighbors -= 1;
		openGLWin.ShowStatusBarMessage(L"minCutNeighbors: " + std::to_wstring(openGLWin.minCutNeighbors));
		cDebug::DbgOut(L"MinCutNeighbors: ", openGLWin.minCutNeighbors);
		}
		}
		else if (Keys::GetKeyState('S'))
		{
		if (Keys::GetKeyStateOnce(VK_ADD))
		{
		openGLWin.minCutSigma += 0.01f;
		openGLWin.ShowStatusBarMessage(L"minCutSigma: " + std::to_wstring(openGLWin.minCutSigma));
		cDebug::DbgOut(L"minCutSigma: ", openGLWin.minCutSigma);
		}
		else if (Keys::GetKeyStateOnce(VK_SUBTRACT))
		{
		openGLWin.minCutSigma -= 0.01f;
		openGLWin.ShowStatusBarMessage(L"minCutSigma: " + std::to_wstring(openGLWin.minCutSigma));

		cDebug::DbgOut(L"minCutSigma: ", openGLWin.minCutSigma);
		}
		}
		else if (Keys::GetKeyState('W'))
		{
		if (Keys::GetKeyStateOnce(VK_ADD))
		{
		openGLWin.minCutWeight += 0.1f;
		openGLWin.ShowStatusBarMessage(L"minCutWeight: " + std::to_wstring(openGLWin.minCutWeight));

		cDebug::DbgOut(L"minCutWeight: ", openGLWin.minCutWeight);
		}
		else if (Keys::GetKeyStateOnce(VK_SUBTRACT))
		{
		openGLWin.minCutWeight -= 0.1f;
		openGLWin.ShowStatusBarMessage(L"minCutWeight: " + std::to_wstring(openGLWin.minCutWeight));
		cDebug::DbgOut(L"minCutWeight: ", openGLWin.minCutWeight);
		}
		}
		}
		else
		{
		if (Keys::GetKeyStateOnce(VK_LBUTTON)
		&& openGLWin.IsMouseInOpenGLWindow()
		&& openGLWin.GetWindowBusyState() != IF_BUSYSTATE_BUSY)
		{
		glSelector.ProcessPicking();
		if (glSelector.selectedIndex != -1)
		{
		glSegmentation.SetSegmentationMesh(glSelector.selectedIndex);
		}
		}

		}

		glSegmentation.InitializeMinCutPreview();
		*/
	}
	else
	{
		if (Keys::GetKeyStateOnce(VK_LBUTTON)
			&& openGLWin.IsMouseInOpenGLWindow()
			&& openGLWin.GetWindowBusyState() != IF_BUSYSTATE_BUSY)
		{
			glSelector.ProcessPicking();
			if (glSelector.selectedIndex != -1)
			{
				//glSegmentation.SetSegmentationMesh(glSelector.selectedIndex);
			}
		}
		meshHelper.DrawAll();
	}

	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glEnable(GL_BLEND);
	//glEnable(GL_CULL_FACE);
	//glEnable(GL_DEPTH_TEST);
}

void RenderInteraction()
{
	if (openGLWin.GetDeviceClass() == IF_DEVICE_PC)
	{
		if (Keys::GetKeyStateOnce(VK_LBUTTON) && (glCamera.mode == CAMERA_SENSOR || openGLWin.colorSelection) && openGLWin.IsMouseInOpenGLWindow() && openGLWin.GetWindowBusyState() != IF_BUSYSTATE_BUSY)
		{
			if (glSelector.selectedIndex == -1)
			{
				glSelector.ProcessPicking();
			}
			else
				if (openGLWin.colorSelection)
					glSelector.ProcessPicking();
				else
					glSelector.ProcessPlacing();

			//return;
		}
		if (glSelector.selectedIndex != -1 && !openGLWin.colorSelection && openGLWin.IsMouseInOpenGLWindow())
		{
			glSelector.ProcessSelectedObject();
		}
	}
	else if (openGLWin.GetDeviceClass() == IF_DEVICE_TABLET)
	{
		if ((glCamera.mode == CAMERA_SENSOR || openGLWin.colorSelection) && openGLWin.IsMouseInOpenGLWindow() && openGLWin.GetWindowBusyState() != IF_BUSYSTATE_BUSY)
		{
			if (!openGLWin.colorSelection)
			{
				if (Keys::GetKeyState(VK_LBUTTON))
				{
					if (glSelector.selectedIndex == -1)
						glSelector.ProcessPicking();
					else if (glSelector.selectedIndex != -1 && !openGLWin.colorSelection && openGLWin.IsMouseInOpenGLWindow())
						glSelector.ProcessSelectedObject();
				}
				else if (glSelector.selectedIndex != -1 && glSelector.GetManipulationMode() == MANIPULATION_NONE)
				{
					glSelector.ProcessPlacing();
				}
			}
			else
			{
				if (Keys::GetKeyStateOnce(VK_LBUTTON))
				{
					//glSelector.ResetTransformationBasePoint();
					glSelector.ProcessPicking();
				}
				else if (glSelector.selectedIndex != -1 && glSelector.GetManipulationMode() != MANIPULATION_NONE)
				{
					glSelector.ProcessObjectManipulation();
				}
			}
			/*if (glSelector.GetManipulationMode() != MANIPULATION_NONE && openGLWin.colorSelection)
			{
			if(Keys::GetKeyStateOnce(VK_LBUTTON))
			{
			if (glSelector.selectedIndex == -1)
			glSelector.ProcessPicking();
			else
			glSelector.ProcessPlacing();
			}
			if (glSelector.selectedIndex != -1)
			{
			glSelector.ProcessSelectedObject();
			}
			}
			else if (Keys::GetKeyState(VK_LBUTTON) && !openGLWin.colorSelection)
			{
			if (glSelector.selectedIndex == -1)
			glSelector.ProcessPicking();
			else if (glSelector.selectedIndex != -1 && !openGLWin.colorSelection && openGLWin.IsMouseInOpenGLWindow())
			glSelector.ProcessSelectedObject();
			}
			else if (Keys::GetKeyStateOnce(VK_LBUTTON) && openGLWin.colorSelection)
			{
			glSelector.ProcessPicking();
			}
			else if (glSelector.selectedIndex != -1 && glSelector.GetManipulationMode() == MANIPULATION_NONE && !openGLWin.colorSelection)
			{
			glSelector.ProcessPlacing();
			}*/


		}
	}

	if (openGLWin.GetWindowMode() != IF_MODE_DEBUG)
		meshHelper.DrawAll();
	if (!gl2DHelper.AreBuffersGenerated())
		gl2DHelper.GenerateBuffers();

	gl2DHelper.DrawTrash();
	//render info text on screen
	glText.PrepareForRender();
	//glText.RenderText(L"FPS: ", openGLWin.glControl.GetFPS(), 20, -0.98f, 0.85f, 2.0f / storedWidth, 2.0f / storedHeight);
	glText.RenderText(L"Meshs: ", meshHelper.GetVisibleMeshCount(), 15, -0.98f, 0.75f, 2.0f / storedWidth, 2.0f / storedHeight);
	glText.RenderText(L"Verts: ", meshHelper.GetNumberOfVertices(), 15, -0.98f, 0.70f, 2.0f / storedWidth, 2.0f / storedHeight);
	glText.RenderText(L"Faces: ", meshHelper.GetNumberOfFaces(), 15, -0.98f, 0.65f, 2.0f / storedWidth, 2.0f / storedHeight);
	glText.RenderText(L"Sel: ", glSelector.selectedIndex, 15, -0.98f, 0.6f, 2.0f / storedWidth, 2.0f / storedHeight);

	//if user clicks with left mouse button -> picking/placing
}

void RenderProcessing()
{
	RenderInteraction();
}

void Render(LPVOID lpParam)
{
	if (!IsWindowVisible(openGLWin.glWindowHandle))
		return;
	//clear window
	glClearColor(openGLWin.bgRed, openGLWin.bgGreen, openGLWin.bgBlue, 1.0f);
	glClearDepth(1.0);

	HandleKeyInput();

	CheckForChangedValues();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (openGLWin.wireFrameMode)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	if (Keys::GetKeyState(VK_LBUTTON) && openGLWin.IsMouseInOpenGLWindow())
	{
		glSelector.ProcessButtonClicks();
	}
	else
		glSelector.UnselectButtons();

	//for initializing without segmentation..debug purposes
	/*if (openGLWin.GetWindowState() == BUFFERS)
	{
		//gl2DHelper.GenerateBuffers();
		meshHelper.GenerateBuffers();
		
		openGLWin.SetWindowState(DEFAULT);
		//return;
	}*/

	switch (openGLWin.GetWindowMode())
	{
		case IF_MODE_DEBUG:
			if (originalMesh->IsLoaded() && !originalMesh->AreBuffersInitialized())
				meshHelper.GenerateBuffers();
			else if (originalMesh->IsLoaded() && originalMesh->AreBuffersInitialized())
				meshHelper.DrawOriginalMesh();
			break;
		case IF_MODE_SEGMENTATION:
			RenderSegmentation();
			break;
		case IF_MODE_MINCUT:
			RenderMinCut();
			break;
		case IF_MODE_INTERACTION:
			RenderInteraction();
			break;
		case IF_MODE_PROCESSING:
			RenderProcessing();
			break;
	}
	//draw helper visuals
	if (openGLWin.helpingVisuals && glHelper.IsRayInitialized())
	{
		glHelper.RenderHelpingVisuals();
	}

	//show bounding boxes on/off
	if (openGLWin.showBB)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		meshHelper.DrawAllForColorPicking();
	}

	if (openGLWin.GetWindowBusyState() == IF_BUSYSTATE_BUSY)
	{
		ShowStatusMsg();
		//gl2DHelper.DrawButtons();
	}

	

	glEnable(GL_DEPTH_TEST);
	//swap buffers to actually display the changes
	openGLWin.glControl.SwapBuffers();
	if (currFPS != openGLWin.glControl.GetFPS())
	{
		currFPS = openGLWin.glControl.GetFPS();
		openGLWin.SetFramesPerSecond((float)currFPS);
	}
}

void Release(LPVOID lpParam)
{
	shaderColor.DeleteProgram();
	shaderFont.DeleteProgram();
	shader2d.DeleteProgram();
	glText.CleanUp();
	glHelper.CleanUp();
	glSegmentation.CleanUp();
	gl2DHelper.CleanUp();
	originalMesh->ClearMesh();
	for (int i = 0; i < 8; i++)shaders[i].DeleteShader();

	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->ClearMesh();
	}
	meshData.clear();
	

}
