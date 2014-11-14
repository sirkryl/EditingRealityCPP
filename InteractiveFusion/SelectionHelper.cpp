#include "SelectionHelper.h"
#include "OpenGLCamera.h"
#include "InteractiveFusion.h"
#include "VisualizationHelper.h"
#include "MeshHelper.h"
#include "colorCoding.h"
#include "VcgMeshContainer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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
	{
		glHelper.InitializeRayVisual();
		if (!firstRayCast)
			firstRayCast = true;
	}

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
	int cnt = 1;
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (cnt != selectedIndex)
			(*mI)->DrawBB();
		cnt++;
	}
	int tmpIndex = GetColorUnderCursor();

	if (tmpIndex > -1 && tmpIndex <= meshData.size())
	{
		openGLWin.ShowStatusBarMessage(L"Selecting object #" + to_wstring(tmpIndex));
		glm::vec3 v1, v2;
		RayCast(&v1, &v2);
		glm::vec3 tmpPoint;

		std::vector<int> orientation = meshData[tmpIndex - 1]->GetOrientation();

		meshData[tmpIndex - 1]->GetHitPoint(v1, v2, tmpPoint, tmpNormal, openGLWin.snapToVertex);
		GetRayOrientation(v1, v2, tmpNormal, orientation);
		hitPoint = tmpPoint;

		meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint, orientation);

		if (preview)
			meshData[selectedIndex - 1]->TogglePreviewSelection(true);
		else
		{
			meshData[selectedIndex - 1]->SetSelected(false);

			if (Keys::GetKeyState('D'))
			{
				int newIndex = DuplicateMesh(selectedIndex - 1);
				selectedIndex = newIndex;
				meshData[selectedIndex - 1]->SetSelected(true);
			}
			else selectedIndex = -1;
		}
		result = true;
	}
	else if (!preview)
	{
		meshData[selectedIndex - 1]->SetSelected(false);
		meshData[selectedIndex - 1]->ResetSelectedTransformation();
		selectedIndex = -1;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	return result;
}

void SelectionHelper::DeleteMesh(int index)
{
	numberOfVertices -= meshData[index]->GetNumberOfVertices();
	numberOfFaces -= meshData[index]->GetNumberOfTriangles();
	meshData[index]->ClearMesh();
	//delete meshData[index];
	meshData.erase(meshData.begin() + index);// std::remove(meshData.begin(), meshData.end(), meshData[index]), meshData.end());
}

int SelectionHelper::DuplicateMesh(int index)
{
	shared_ptr<VCGMeshContainer> mesh(new VCGMeshContainer);
	mesh->SetColorCode(meshData.size() + 1);

	mesh->ConvertToVCG(meshData[index]->GetVertices(), meshData[index]->GetIndices());
	mesh->ParseData();
	mesh->GenerateBOs();
	mesh->GenerateVAO();
	numberOfVertices += mesh->GetNumberOfVertices();
	numberOfFaces += mesh->GetNumberOfTriangles();
	meshData.push_back(mesh);
	return meshData.size();
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
		if (index != selectedIndex - 1)
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
	if (sIndex > 0)
	{
		std::vector<int> orientation = meshData[sIndex - 1]->GetOrientation();
		GetRayOrientation(v1, v2, hitNormal, orientation);
		meshData[selectedIndex - 1]->TranslateVerticesToPoint(hitPoint, orientation);

		if (preview)
			meshData[selectedIndex - 1]->TogglePreviewSelection(true);
		else
		{
			meshData[selectedIndex - 1]->SetSelected(false);
			selectedIndex = -1;
		}
		result = true;
	}
	else if (!preview)
	{
		meshData[selectedIndex - 1]->SetSelected(false);
		meshData[selectedIndex - 1]->ResetSelectedTransformation();
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

void SelectionHelper::ProcessSelectedObject()
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
		DeleteMesh(selectedIndex - 1);
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

void SelectionHelper::ProcessPicking()
{
	for (vector <shared_ptr<VCGMeshContainer>>::iterator mI = meshData.begin(); mI != meshData.end(); ++mI)
	{
		if (!(*mI)->IsWall())
			(*mI)->DrawBB();
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
		openGLWin.ShowStatusBarMessage(L"Selecting object #" + to_wstring(tmpIndex));
		meshData[tmpIndex - 1]->SetSelected(true);
		if (openGLWin.colorSelection)
			meshData[tmpIndex - 1]->ToggleSelectedColor(true);
		selectedIndex = tmpIndex;
	}
	else
	{
		if (selectedIndex != -1)
		{
			openGLWin.ShowStatusBarMessage(L"Unselecting object #" + to_wstring(selectedIndex));

			if (openGLWin.colorSelection)
				meshData[selectedIndex - 1]->ToggleSelectedColor(false);
			meshData[selectedIndex - 1]->SetSelected(false);
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
		meshData[selectedIndex - 1]->SetWall(true);
		if (openGLWin.colorSelection)
			meshData[selectedIndex - 1]->ToggleSelectedColor(false);
		meshData[selectedIndex - 1]->SetSelected(false);
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
#pragma endregion wall object