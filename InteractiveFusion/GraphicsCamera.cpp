#include "GraphicsCamera.h"

#include "KeyState.h"

#include <glm/gtx/rotate_vector.hpp>
#include <Windows.h>

namespace InteractiveFusion {
	
	POINT currentCursorPosition;
	POINT previousCursorPosition;
	bool rotationPointInitialized = false;
	GraphicsCamera::GraphicsCamera()
	{
		Initialize(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	}

	void GraphicsCamera::Initialize(glm::vec3 _position, glm::vec3 _lookAt, glm::vec3 _upDirection)
	{
		Initialize(_position, _lookAt, _upDirection, 0.05f, 0.15f, 0.2f, 0.002f, 0.002f);
	}

	void GraphicsCamera::Initialize(glm::vec3 _position, glm::vec3 _lookAt, glm::vec3 _upDirection, float _zoomStep, float _orbitXStep, float _orbitYStep, float _strafeXStep, float _strafeYStep)
	{

		camPosition = _position;
		camLookAt = _lookAt;
		camUpDirection = _upDirection;

		zoomFactor = 1.0f;
		zoomStep = _zoomStep;
		orbitXStep = _orbitXStep;
		orbitYStep = _orbitYStep;
		completeStrafeX = 0.0f;
		completeStrafeY = 0.0f;
		strafeXStep = _strafeXStep;
		strafeYStep = _strafeYStep;
		//rotationPoint = glm::vec3(0.0f, 0.0f, -1.0f);
	}

	void GraphicsCamera::ResetCameraPosition()
	{
		zoomFactor = 1.0f;
		zoomStep = 0.2f;
		orbitXStep = 0.2f;
		orbitYStep = 0.15f;
		completeStrafeX = 0.0f;
		completeStrafeY = 0.0f;
		strafeXStep = 0.002f;
		strafeYStep = 0.002f;

		firstTimeLeftMouseClick = false;
		firstTimeMiddleMouseClick = false;

		camPosition = glm::vec3(0.0f, 0.0f, 0.0f);
		camLookAt = glm::vec3(0.0f, 0.0, -1.0f);
		camUpDirection = glm::vec3(0.0f, 1.0f, 0.0f);
	}

	void GraphicsCamera::Update(bool _enableMouseInput, int _mouseWheelDelta)
	{
		enableMouseInput = _enableMouseInput && rotationPointInitialized;
		mouseWheelDelta = _mouseWheelDelta;
	}

	void GraphicsCamera::SetRotationPoint(glm::vec3 point)
	{
		rotationPoint = point;
		camLookAt = rotationPoint;
		rotationPointInitialized = true;
	}

	void GraphicsCamera::Zoom()
	{
		if (mouseWheelDelta != 0)
		{
			if (mouseWheelDelta > 0)
			{
				zoomFactor -= zoomStep;
			}
			else if (mouseWheelDelta < 0)
			{
				zoomFactor += zoomStep;
			}
			mouseWheelDelta = 0;
		}
	}

	void GraphicsCamera::Strafe()
	{
		completeStrafeX += (float)(previousCursorPosition.x - currentCursorPosition.x)*strafeXStep;
		completeStrafeY += (float)(currentCursorPosition.y - previousCursorPosition.y)*strafeYStep;
	}

	void GraphicsCamera::Orbit()
	{
		float cursorOffsetY = (float)(previousCursorPosition.y - currentCursorPosition.y)*orbitYStep;
		float cursorOffsetX = (float)(previousCursorPosition.x - currentCursorPosition.x)*orbitXStep;
		if (cursorOffsetY != 0 || cursorOffsetX != 0)
		{
			glm::vec4 pos4 = glm::vec4(camPosition, 1.0f);
			camRight = glm::cross(camUpDirection, camDirection);
			glm::vec4 up4 = glm::vec4(camUpDirection, 1.0f);

			glm::mat4 yRotation = glm::rotate(cursorOffsetY, camRight);
			glm::mat4 xRotation = glm::rotate(cursorOffsetX, camUpDirection);
			pos4 = (yRotation * (glm::vec4(camPosition, 1.0f) - glm::vec4(camLookAt, 1.0f))) + glm::vec4(camLookAt, 1.0f);

			up4 = yRotation * up4;
			camUpDirection = glm::normalize(glm::vec3(up4.x, up4.y, up4.z));
			camPosition = glm::vec3(pos4.x, pos4.y, pos4.z);

			camDirection = glm::normalize(camPosition - camLookAt);
			camRight = glm::normalize(glm::cross(camUpDirection, camDirection));
			pos4 = glm::vec4(camPosition, 1.0f);
			up4 = glm::vec4(camUpDirection, 1.0f);


			pos4 = (xRotation * (glm::vec4(camPosition, 1.0f) - glm::vec4(camLookAt, 1.0f))) + glm::vec4(camLookAt, 1.0f);

			camPosition = glm::vec3(pos4.x, pos4.y, pos4.z);
			up4 = xRotation * up4;
			camUpDirection = glm::normalize(glm::vec3(up4.x, up4.y, up4.z));
		}
	}

	glm::mat4 GraphicsCamera::GetViewMatrix()
	{
		camDirection = glm::normalize(camPosition - camLookAt);

		if (enableMouseInput)
		{
			Zoom();
			if (KeyState::GetKeyState(VK_LBUTTON))
			{
				GetCursorPos(&currentCursorPosition);
				if (firstTimeLeftMouseClick)
					Orbit();
				previousCursorPosition = currentCursorPosition;
				firstTimeLeftMouseClick = true;
			}
			else
				firstTimeLeftMouseClick = false;

			if (KeyState::GetKeyState(VK_MBUTTON))
			{
				GetCursorPos(&currentCursorPosition);
				if (firstTimeMiddleMouseClick)
					Strafe();
				previousCursorPosition = currentCursorPosition;
				firstTimeMiddleMouseClick = true;
			}
			else
				firstTimeMiddleMouseClick = false;
		}

		camLookAt = rotationPoint;

		return glm::lookAt(CalculateCameraPosition(), CalculateCameraLookAt(), camUpDirection);
	}

	glm::vec3 GraphicsCamera::CalculateCameraPosition()
	{
		return camPosition + (camDirection * zoomFactor) + (camRight * completeStrafeX) + (camUpDirection * completeStrafeY);
	}

	glm::vec3 GraphicsCamera::CalculateCameraLookAt()
	{
		return camLookAt + (camRight * completeStrafeX) + (camUpDirection * completeStrafeY);
	}
}