#include "common.h"
#include "openGLWin.h"
#include "openGLCamera.h"
#include <gl/glew.h>


const float PI = float(atan(1.0)*4.0);

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

void OpenGLCamera::ResetCameraPosition()
{
	moveBy = glm::vec3(0.0f, 0.0f, 0.0f);
}

void OpenGLCamera::RotateWithMouse()
{
	GetCursorPos(&pCur);
	RECT rRect; GetWindowRect(openGLWin.glWindowHandle, &rRect);
	int iCentX = (rRect.left+rRect.right)>>1,
		iCentY = (rRect.top+rRect.bottom)>>1;

	float deltaX = (float)(iCentX-pCur.x)*rotationSensitivity;
	float deltaY = (float)(iCentY-pCur.y)*rotationSensitivity;

	if(deltaX != 0.0f)
	{
		camLookAt -= camPosition;
		camLookAt = glm::rotate(camLookAt, deltaX, glm::vec3(0.0f, 1.0f, 0.0f));
		camLookAt += camPosition;
	}
	if(deltaY != 0.0f)
	{
		glm::vec3 vAxis = glm::cross(camLookAt - camPosition, camUpDirection);
		vAxis = glm::normalize(vAxis);
		float fAngle = deltaY;
		float fNewAngle = fAngle+GetAngleX();
		if(fNewAngle > -89.80f && fNewAngle < 89.80f)
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

	glm::vec3 vDir = camLookAt-camPosition; vDir.y = 0.0f;
	glm::normalize(vDir);
	float fAngle = acos(glm::dot(glm::vec3(0, 0, -1), vDir))*(180.0f/PI);
	if(vDir.x < 0)fAngle = 360.0f-fAngle;
	return fAngle;
}

float OpenGLCamera::GetAngleX()
{
	glm::vec3 vDir = camLookAt-camPosition;
	vDir = glm::normalize(vDir);
	glm::vec3 vDir2 = vDir; vDir2.y = 0.0f;
	vDir2 = glm::normalize(vDir2);
	float fAngle =  acos(glm::dot(vDir2, vDir))*(180.0f/PI);
	if(vDir.y < 0)fAngle *= -1.0f;
	return fAngle;
}

void OpenGLCamera::Update()
{
	//if (Keys::GetKeyState('L'))
		//RotateWithMouse();

	glm::vec3 moveTmp = camLookAt-camPosition;
	moveTmp = glm::normalize(moveTmp);
	moveTmp *= moveSpeed;

	glm::vec3 strafeTmp = glm::cross(camLookAt-camPosition, camUpDirection);
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
	int iCentX = (rRect.left+rRect.right)>>1,
		iCentY = (rRect.top+rRect.bottom)>>1;
	SetCursorPos(iCentX, iCentY);
}

glm::vec3 OpenGLCamera::GetPosition()
{
	return camPosition;
}

glm::mat4 OpenGLCamera::GetViewMatrix()
{
	if (openGLWin.freeCameraControls)
	{	
		camPosition = camPosition + (moveBy - oldMoveBy);
		camLookAt = camLookAt + (moveBy - oldMoveBy);
		camUpDirection = camUpDirection;
		oldMoveBy = moveBy;

		if (Keys::GetKeyState(VK_LBUTTON) &&
			openGLWin.IsMouseInOpenGLWindow())
				RotateWithMouse();

		return glm::lookAt(camPosition, camLookAt, camUpDirection);
	}
	else
	{
		glm::mat4 cameraTransform = openGLWin.glControl.GetKinectViewMatrix();

		camPosition = glm::vec3(-cameraTransform[3][0], cameraTransform[3][1], cameraTransform[3][2]);// +moveBy;
		camLookAt = camPosition + glm::vec3(-cameraTransform[2][0], cameraTransform[2][1], -cameraTransform[2][2]);// +moveBy;
		camUpDirection = glm::vec3(cameraTransform[1][0], cameraTransform[1][1], cameraTransform[1][2]);

		return glm::lookAt(camPosition, camLookAt, camUpDirection);
	}
	
	
}