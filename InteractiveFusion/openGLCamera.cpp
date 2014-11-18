#include "common.h"
#include "InteractiveFusion.h"
#include "OpenGLCamera.h"
#include <glm/ext.hpp>
#include "Keys.h"
float angleX = 5.0f;
float zoomFactor = 1.0f;
float strafeX = 0.0f;
float strafeY = 0.0f;
float cumAngle = 5.0f;
int oldX;
int oldY;
float theta = 0;
float phi = 0;
const float PI = float(atan(1.0)*4.0);

glm::vec3 camDirection;
glm::vec3 camRight;

OpenGLCamera::OpenGLCamera()
{
	camPosition = glm::vec3(0.0f, 0.0f, 0.0f);
	camLookAt = glm::vec3(0.0f, 0.0, -1.0f);
	camUpDirection = glm::vec3(0.0f, 1.0f, 0.0f);
	moveBy = glm::vec3(0.0f, 0.0f, 0.0f);
	moveSpeed = 25.0f;
	rotationSensitivity = 0.1f;
}

OpenGLCamera::OpenGLCamera(glm::vec3 position, glm::vec3 lookAt, glm::vec3 upDirection, float mvSpeed, float sensitivity)
{
	camPosition = position;
	camLookAt = lookAt;
	camUpDirection = upDirection;
	moveSpeed = mvSpeed;
	rotationSensitivity = sensitivity;
	moveBy = glm::vec3(0.0f, 0.0f, 0.0f);
}

void OpenGLCamera::GetRotation(glm::mat4 &rotation)
{
	rotation = glm::mat4(GetViewMatrix());
	rotation[0][3] = rotation[1][3] = rotation[2][3] =
		rotation[3][0] = rotation[3][1] = rotation[3][2] = 0.0f;
	rotation[3][3] = 1.0f;
	rotation = glm::inverse(rotation);
}

void OpenGLCamera::ResetCameraPosition()
{
	//moveBy = glm::vec3(0.0f, 0.0f, 0.0f);
	strafeX = 0.0f;
	strafeY = 0.0f;
	zoomFactor = 1.0f;
	camPosition = glm::vec3(0.0f, 0.0f, 0.0f);
	camLookAt = glm::vec3(0.0f, 0.0, -1.0f);
	camUpDirection = glm::vec3(0.0f, 1.0f, 0.0f);
}

void OpenGLCamera::SetRotationPoint(glm::vec3 point)
{
	rotationPoint = point;
	camLookAt = rotationPoint;
}

void OpenGLCamera::UpdateZoom()
{
	if (openGLWin.wheelDelta != 0)
	{
		if (openGLWin.wheelDelta > 0)
		{
			zoomFactor -= 0.2f;
		}
		else if (openGLWin.wheelDelta < 0)
		{
			zoomFactor += 0.2f;
		}
		openGLWin.wheelDelta = 0;
	}
}

void OpenGLCamera::UpdateStrafe()
{


	strafeX += (float)(oldX - pCur.x)*0.002f;
	strafeY += (float)(pCur.y - oldY)*0.002f;
	//cDebug::DbgOut(L"strafe");
	/*if (Keys::GetKeyState('D'))
	{
	//cDebug::DbgOut(L"D");
	strafe += 0.05f;
	}
	if (Keys::GetKeyState('A'))
	{
	//cDebug::DbgOut(L"A");
	strafe -= 0.05f;
	}*/
}

void OpenGLCamera::Orbit()
{

	float offSetX = (float)(oldY - pCur.y)*0.20f;
	float offSetY = (float)(oldX - pCur.x)*0.150f;
	if (offSetX != 0 || offSetY != 0)
	{
		glm::vec4 pos4 = glm::vec4(camPosition, 1.0f);
		camRight = glm::cross(camUpDirection, camDirection);
		glm::vec4 up4 = glm::vec4(camUpDirection, 1.0f);

		glm::mat4 xRotation = glm::rotate(offSetX, camRight);
		glm::mat4 yRotation = glm::rotate(offSetY, camUpDirection);
		pos4 = (xRotation * (glm::vec4(camPosition, 1.0f) - glm::vec4(camLookAt, 1.0f))) + glm::vec4(camLookAt, 1.0f);

		up4 = xRotation * up4;
		camUpDirection = glm::normalize(glm::vec3(up4.x, up4.y, up4.z));
		camPosition = glm::vec3(pos4.x, pos4.y, pos4.z);

		camDirection = glm::normalize(camPosition - camLookAt);
		camRight = glm::normalize(glm::cross(camUpDirection, camDirection));
		pos4 = glm::vec4(camPosition, 1.0f);
		up4 = glm::vec4(camUpDirection, 1.0f);


		pos4 = (yRotation * (glm::vec4(camPosition, 1.0f) - glm::vec4(camLookAt, 1.0f))) + glm::vec4(camLookAt, 1.0f);

		camPosition = glm::vec3(pos4.x, pos4.y, pos4.z);
		up4 = yRotation * up4;
		camUpDirection = glm::normalize(glm::vec3(up4.x, up4.y, up4.z));
	}
}

glm::vec3 OpenGLCamera::GetDirection()
{
	return camDirection;
}

glm::mat4 OpenGLCamera::GetViewMatrix()
{
	if (mode == CAMERA_FREE)
	{
		GetCursorPos(&pCur);

		camDirection = glm::normalize(camPosition - camLookAt);

		UpdateZoom();
		//UpdateStrafe();

		if (Keys::GetKeyState(VK_LBUTTON) &&
			openGLWin.IsMouseInOpenGLWindow())
			Orbit();

		if (Keys::GetKeyState(VK_MBUTTON) &&
			openGLWin.IsMouseInOpenGLWindow())
			UpdateStrafe();

		oldX = pCur.x;
		oldY = pCur.y;

		camLookAt = rotationPoint;



		return glm::lookAt(camPosition + (camDirection * zoomFactor) + (camRight * strafeX) + (camUpDirection * strafeY), camLookAt + (camRight * strafeX) + (camUpDirection * strafeY), camUpDirection);
	}
	else if (mode == CAMERA_SENSOR)
	{
		
		zoomFactor = 1.0f;
		strafeX = 0.0f;
		strafeY = 0.0f;
		glm::mat4 cameraTransform = openGLWin.glControl.GetKinectViewMatrix();

		camPosition = glm::vec3(-cameraTransform[3][0], cameraTransform[3][1], cameraTransform[3][2]);// +moveBy;
		camLookAt = camPosition + glm::vec3(-cameraTransform[2][0], cameraTransform[2][1], -cameraTransform[2][2]);// +moveBy;
		camUpDirection = glm::vec3(cameraTransform[1][0], cameraTransform[1][1], cameraTransform[1][2]);
		camDirection = glm::normalize(camPosition - camLookAt);
		return glm::lookAt(camPosition, camLookAt, camUpDirection);
	}


}

#pragma region
void OpenGLCamera::RotateWithMouse()
{
	GetCursorPos(&pCur);
	RECT rRect; GetWindowRect(openGLWin.glWindowHandle, &rRect);
	int iCentX = (rRect.left + rRect.right) >> 1,
		iCentY = (rRect.top + rRect.bottom) >> 1;

	float deltaX = (float)(iCentX - pCur.x)*rotationSensitivity;
	float deltaY = (float)(iCentY - pCur.y)*rotationSensitivity;

	if (deltaX != 0.0f)
	{


		camLookAt -= camPosition;
		camLookAt = glm::rotate(camLookAt, deltaX, glm::vec3(0.0f, 1.0f, 0.0f));
		camLookAt += camPosition;
	}
	if (deltaY != 0.0f)
	{
		glm::vec3 vAxis = glm::cross(camLookAt - camPosition, camUpDirection);
		vAxis = glm::normalize(vAxis);
		float fAngle = deltaY;
		float fNewAngle = fAngle + GetAngleX();
		if (fNewAngle > -89.80f && fNewAngle < 89.80f)
		{
			camLookAt -= camPosition;
			camLookAt = glm::rotate(camLookAt, deltaY, vAxis);
			camLookAt += camPosition;
		}
	}
	SetCursorPos(iCentX, iCentY);
}

float OpenGLCamera::GetAngleY()
{

	glm::vec3 vDir = camLookAt - camPosition; vDir.y = 0.0f;
	glm::normalize(vDir);
	float fAngle = acos(glm::dot(glm::vec3(0, 0, -1), vDir))*(180.0f / PI);
	if (vDir.x < 0)fAngle = 360.0f - fAngle;
	return fAngle;
}

float OpenGLCamera::GetAngleX()
{
	glm::vec3 vDir = camLookAt - camPosition;
	vDir = glm::normalize(vDir);
	glm::vec3 vDir2 = vDir; vDir2.y = 0.0f;
	vDir2 = glm::normalize(vDir2);
	float fAngle = acos(glm::dot(vDir2, vDir))*(180.0f / PI);
	if (vDir.y < 0)fAngle *= -1.0f;
	return fAngle;
}

void OpenGLCamera::Update()
{
	//if (Keys::GetKeyState('L'))
	//RotateWithMouse();

	glm::vec3 moveTmp = camLookAt - camPosition;
	moveTmp = glm::normalize(moveTmp);
	moveTmp *= moveSpeed;

	glm::vec3 strafeTmp = glm::cross(camLookAt - camPosition, camUpDirection);
	strafeTmp = glm::normalize(strafeTmp);
	strafeTmp *= moveSpeed;

	if (Keys::GetKeyState(fwKey))
		moveBy += moveTmp*openGLWin.SpeedOptimizedFloat(1.0f);
	if (Keys::GetKeyState(bwKey))
		moveBy -= moveTmp*openGLWin.SpeedOptimizedFloat(1.0f);
	if (Keys::GetKeyState(leftKey))
		moveBy -= strafeTmp*openGLWin.SpeedOptimizedFloat(1.0f);
	if (Keys::GetKeyState(rightKey))
		moveBy += strafeTmp*openGLWin.SpeedOptimizedFloat(1.0f);
}

void OpenGLCamera::ResetMouse()
{
	RECT rRect; GetWindowRect(openGLWin.glWindowHandle, &rRect);
	int iCentX = (rRect.left + rRect.right) >> 1,
		iCentY = (rRect.top + rRect.bottom) >> 1;
	SetCursorPos(iCentX, iCentY);
}

glm::vec3 OpenGLCamera::GetUpDirection()
{
	return camUpDirection;
}

glm::vec3 OpenGLCamera::GetPosition()
{
	return camPosition;
}

#pragma endregion not used