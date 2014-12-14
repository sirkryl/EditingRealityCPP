#include "SelectionHelper.h"
#include "OpenGLCamera.h"
#include "InteractiveFusion.h"
#include "VisualizationHelper.h"
#include "MeshHelper.h"
#include "colorCoding.h"
#include "OpenGL2DHelper.h"
#include "VcgMeshContainer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "Keys.h"

int oldPosX;
int oldPosY;
bool firstClick = false;
int SelectionHelper::GetColorUnderCursor()
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
	if (colorByte[0] == openGLWin.bgRed && colorByte[1] == openGLWin.bgGreen && colorByte[2] == openGLWin.bgBlue)
		return -1;
	return output;
}

void SelectionHelper::RayCast(glm::vec3* v1, glm::vec3* v2)
{
	POINT cursorPos;
	GetCursorPos(&cursorPos);
	ScreenToClient(openGLWin.glWindowHandle, &cursorPos);
	RECT rect;
	GetClientRect(openGLWin.glWindowHandle, &rect);
	cursorPos.y = rect.bottom - cursorPos.y;

	glm::vec4 viewport = glm::vec4(0.0f, 0.0f, openGLWin.glControl.GetViewportWidth(), openGLWin.glControl.GetViewportHeight());

	*v1 = glm::unProject(glm::vec3(float(cursorPos.x), float(cursorPos.y), 0.0f), glCamera.GetViewMatrix(), openGLWin.glControl.GetProjectionMatrix(), viewport);
	*v2 = glm::unProject(glm::vec3(float(cursorPos.x), float(cursorPos.y), 1.0f), glCamera.GetViewMatrix(), openGLWin.glControl.GetProjectionMatrix(), viewport);
	nearPoint = *v1;
	if (openGLWin.helpingVisuals)
		glHelper.InitializeRayVisual();

}

void SelectionHelper::GetRayOrientation(glm::vec3 v1, glm::vec3 v2, glm::vec3 normal, std::vector<int> &orientation)
{
	/*orientation.clear();
	orientation.push_back(0);
	orientation.push_back(0);
	orientation.push_back(0);*/
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

bool SelectionHelper::ColorPlacing(bool preview)
{
	bool result = false;
	glm::vec3 tmpNormal;
	std::wstringstream ws;
	int cnt = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (cnt != selectedIndex)
			(*mI)->DrawBB();
		cnt++;
	}
	gl2DHelper.DrawTrashBB();
	int tmpIndex = GetColorUnderCursor();

	if (tmpIndex == TRASH_BIN_COLOR)
	{
		gl2DHelper.isTrashOpen = true;
		meshData[selectedIndex]->IsOverTrash(true);
	}
	else
	{
		gl2DHelper.isTrashOpen = false;
		meshData[selectedIndex]->IsOverTrash(false);
	}

	bool found = false;
	int meshIndex = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (tmpIndex == (*mI)->GetColorCode())
		{
			tmpIndex = meshIndex;
			found = true;
			break;
		}
		meshIndex++;
	}

	if (found && tmpIndex != TRASH_BIN_COLOR)
	{
		openGLWin.ShowStatusBarMessage(L"Selecting object #" + to_wstring(tmpIndex));
		glm::vec3 v1, v2;
		RayCast(&v1, &v2);
		glm::vec3 tmpPoint;

		std::vector<int> orientation = meshData[tmpIndex]->GetOrientation();

		meshData[tmpIndex]->GetHitPoint(v1, v2, tmpPoint, tmpNormal, openGLWin.snapToVertex);
		GetRayOrientation(v1, v2, tmpNormal, orientation);
		hitPoint = tmpPoint;

		meshData[selectedIndex]->TranslateVerticesToPoint(hitPoint, orientation);

		if (preview)
			meshData[selectedIndex]->TogglePreviewSelection(true);
		else
		{
			meshData[selectedIndex]->SetSelected(false);

			if ((Keys::GetKeyState('D') || openGLWin.duplicationMode) && openGLWin.GetDeviceClass() == IF_DEVICE_PC)
			{
				int newIndex = meshHelper.DuplicateMesh(selectedIndex);

				selectedIndex = newIndex;
				meshData[selectedIndex]->SetSelected(true);
			}
			else selectedIndex = -1;
		}
		result = true;
	}
	else if (!preview)
	{
		if (tmpIndex == TRASH_BIN_COLOR)
		{
			meshHelper.DeleteMesh(selectedIndex);
			selectedIndex = -1;
			gl2DHelper.isTrashOpen = false;
		}
		else
		{
			meshData[selectedIndex]->SetSelected(false);
			meshData[selectedIndex]->ResetSelectedTransformation();
			selectedIndex = -1;
		}
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	return result;
}

bool SelectionHelper::RayCastPlacing(bool preview)
{
	bool result = false;
	glm::vec3 hitNormal;
	glm::vec3 v1, v2;
	RayCast(&v1, &v2);
	int index = 0;
	float maxZ = -1000.0f;
	int sIndex = -1;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (index != selectedIndex)
		{
			glm::vec3 tmpHit;
			glm::vec3 tmpNormal;
			if ((*mI)->GetHitPoint(v1, v2, tmpHit, tmpNormal, openGLWin.snapToVertex))
			{
				float cMaxZ = tmpHit.z;

				if (cMaxZ > maxZ)
				{

					hitPoint = tmpHit;
					hitNormal = tmpNormal;
					sIndex = index;
					maxZ = cMaxZ;
				}

			}
		}
		index++;
	}
	if (sIndex > -1)
	{
		std::vector<int> orientation = meshData[sIndex]->GetOrientation();
		GetRayOrientation(v1, v2, hitNormal, orientation);
		meshData[selectedIndex]->TranslateVerticesToPoint(hitPoint, orientation);

		if (preview)
			meshData[selectedIndex]->TogglePreviewSelection(true);
		else
		{
			meshData[selectedIndex]->SetSelected(false);
			selectedIndex = -1;
		}
		result = true;
	}
	else if (!preview)
	{
		meshData[selectedIndex]->SetSelected(false);
		meshData[selectedIndex]->ResetSelectedTransformation();
		selectedIndex = -1;
	}
	return result;
}

bool SelectionHelper::PlacingPreview()
{
	if (!openGLWin.placeWithRaycast)
	{
		return ColorPlacing(true);
	}
	else
	{
		return RayCastPlacing(true);
	}
}

void SelectionHelper::ResetTransformationBasePoint()
{
	manipulationPointInitialized = false;
}

void SelectionHelper::ProcessObjectManipulation()
{
	if (!manipulationPointInitialized)
	{
		manipulationPointInitialized = true;
		manipulationBasePoint = meshData[selectedIndex]->GetBasePoint();
	}
	meshData[selectedIndex]->TranslateVerticesToPoint(manipulationBasePoint, { 0, 1, 0 });


	if (Keys::GetKeyState(VK_LBUTTON) &&
		openGLWin.IsMouseInOpenGLWindow())
	{
		POINT pCur;
		GetCursorPos(&pCur);
		int offSetX = (int)((pCur.y - oldPosY)*0.2f);
		int offSetY = (int)((pCur.x - oldPosX)*0.2f);

		if (firstClick)
		{ 
			meshData[selectedIndex]->RotateX(offSetX);
			meshData[selectedIndex]->RotateY(offSetY);
		}
		oldPosX = pCur.x;
		oldPosY = pCur.y;
		firstClick = true;
	}
	else
		firstClick = false;

	if (Keys::GetKeyState(VK_DELETE))
	{
		meshHelper.DeleteMesh(selectedIndex);
		selectedIndex = -1;
		cDebug::DbgOut(L"pressed ENTF alright");
	}
	if (openGLWin.wheelDelta < 0)
	{
		meshData[selectedIndex]->SetScale(false);
	}
	else if (openGLWin.wheelDelta > 0)
	{
		meshData[selectedIndex]->SetScale(true);
	}
	openGLWin.wheelDelta = 0;

}

void SelectionHelper::ProcessSelectedObject()
{
	//while (previewThreadActive)
	//{
	//cDebug::DbgOut(L"ahoi", 1);
	//boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	if (!PlacingPreview())
	{
		meshData[selectedIndex]->TogglePreviewSelection(false);

		glm::vec3 v1, v2;
		RayCast(&v1, &v2);
		meshData[selectedIndex]->AttachToCursor(v1, v2, openGLWin.carryDistance);
	}
	if (Keys::GetKeyState(VK_DELETE))
	{
		meshHelper.DeleteMesh(selectedIndex);
		selectedIndex = -1;
		cDebug::DbgOut(L"pressed ENTF alright");
	}
	if (Keys::GetKeyState('X') || manipMode == MANIPULATION_ROTATE_X)
	{
		//cDebug::DbgOut(L"Wheel: ", openGLWin.GetWheelDelta());
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex]->SetAngleX(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex]->SetAngleX(true);
		}
	}
	if (Keys::GetKeyState('Y') || manipMode == MANIPULATION_ROTATE_Y)
	{
		//cDebug::DbgOut(L"Wheel: ", openGLWin.GetWheelDelta());
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex]->SetAngleY(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex]->SetAngleY(true);
		}

	}
	if (Keys::GetKeyState('Z') || manipMode == MANIPULATION_ROTATE_Z)
	{
		//cDebug::DbgOut(L"Wheel: ", openGLWin.GetWheelDelta());
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex]->SetAngleZ(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex]->SetAngleZ(true);
		}
	}
	if (Keys::GetKeyState('U') || manipMode == MANIPULATION_SCALE)
	{
		if (openGLWin.wheelDelta < 0)
		{
			meshData[selectedIndex]->SetScale(false);
		}
		else if (openGLWin.wheelDelta > 0)
		{
			meshData[selectedIndex]->SetScale(true);
		}
		openGLWin.wheelDelta = 0;
	}
	openGLWin.wheelDelta = 0;
	//}
}

void SelectionHelper::ProcessButtonClicks()
{
	gl2DHelper.DrawButtonsBB();
	int color = GetColorUnderCursor();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (gl2DHelper.SelectButton(color))
	{
		selectedButton = color;
		
	}
	else
	{
		selectedButton = -1;
		gl2DHelper.UnselectButtons();
	}
		
	
}

void SelectionHelper::UnselectButtons()
{
	if (selectedButton != -1)
	{
		if (selectedButton == OPENGL_BUTTON_OK)
		{
			if (openGLWin.GetWindowBusyState() == IF_BUSYSTATE_BUSY)
			{
				openGLWin.SetWindowBusyState(IF_BUSYSTATE_DEFAULT);
			}
		}
	}
	selectedButton = -1;
	gl2DHelper.UnselectButtons();
}

void SelectionHelper::ProcessPicking()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (!(*mI)->IsWall() || (openGLWin.colorSelection && GetManipulationMode() == MANIPULATION_NONE))
			(*mI)->DrawBB();
	}
	int tmpIndex = GetColorUnderCursor();

	
		
	

	bool found = false;
	int meshIndex = 0;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (tmpIndex == (*mI)->GetColorCode())
		{
			tmpIndex = meshIndex;
			found = true;
			break;
		}
		meshIndex++;
	}

	if (found)
	{
		if (selectedIndex != -1)
		{
			if (openGLWin.colorSelection)
			{ 
				meshData[selectedIndex]->ToggleSelectedColor(false);
				ResetTransformationBasePoint();
			}
			meshData[selectedIndex]->SetSelected(false);
		}
		openGLWin.ShowStatusBarMessage(L"Selecting object #" + to_wstring(tmpIndex));

		if ((Keys::GetKeyState('D') || openGLWin.duplicationMode) && openGLWin.GetDeviceClass() == IF_DEVICE_TABLET)
		{
			int newIndex = meshHelper.DuplicateMesh(tmpIndex);

			selectedIndex = newIndex;
			meshData[selectedIndex]->SetSelected(true);

		}
		else
		{ 
			meshData[tmpIndex]->SetSelected(true);
			if (openGLWin.colorSelection)
				meshData[tmpIndex]->ToggleSelectedColor(true);
			selectedIndex = tmpIndex;
		}
	}
	else
	{
		if (selectedIndex != -1)
		{
			openGLWin.ShowStatusBarMessage(L"Unselecting object #" + to_wstring(selectedIndex));

			if (openGLWin.colorSelection)
			{
				if (GetManipulationMode() == MANIPULATION_NONE)
					meshData[selectedIndex]->ToggleSelectedColor(false);
				else
					return;
			}
			meshData[selectedIndex]->SetSelected(false);
			selectedIndex = -1;
		}
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void SelectionHelper::ProcessPlacing()
{
	glm::vec3 tmpNormal;
	if (!openGLWin.placeWithRaycast)
	{
		ColorPlacing(false);
	}
	else
	{
		RayCastPlacing(false);
	}
}

#pragma region
void SelectionHelper::SelectWallObject()
{
	if (selectedIndex != -1)
	{
		meshData[selectedIndex]->SetWall(true);
		if (openGLWin.colorSelection)
			meshData[selectedIndex]->ToggleSelectedColor(false);
		meshData[selectedIndex]->SetSelected(false);
		selectedIndex = -1;
	}
}

void SelectionHelper::ResetWallObject()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		(*mI)->SetWall(false);
	}
}

void SelectionHelper::SetManipulationMode(ManipulationMode mode)
{
	manipMode = mode;
}

void SelectionHelper::Unselect()
{
	if (selectedIndex != -1)
	{
		meshData[selectedIndex]->SetSelected(false);
		meshData[selectedIndex]->ToggleSelectedColor(false);
		ResetTransformationBasePoint();
	}
	selectedIndex = -1;
}

ManipulationMode SelectionHelper::GetManipulationMode()
{
	return manipMode;
}
#pragma endregion wall object