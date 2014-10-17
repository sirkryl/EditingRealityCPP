#pragma once

class OpenGLCamera
{
public:

	OpenGLCamera();
	OpenGLCamera(glm::vec3 position, glm::vec3 lookAt, glm::vec3 upDirection, float mvSpeed, float sensitivity);


	void RotateWithMouse();

	void Update();
	void ResetCameraPosition();
	glm::mat4 GetViewMatrix();
	glm::vec3 GetPosition();
	void ResetMouse();

	// Functions that get viewing angles
	float GetAngleX(), GetAngleY();

	// Constructors
	

private:
	glm::vec3 camPosition;
	glm::vec3 camLookAt;
	glm::vec3 camUpDirection;
	glm::vec3 moveBy;
	glm::vec3 oldMoveBy;



	float moveSpeed;
	float rotationSensitivity;

	POINT pCur; // For mosue rotation
	int fwKey = 'W';
	int bwKey = 'S';
	int leftKey = 'A';
	int rightKey = 'D';
	float deltaX;
	float deltaY;
	float vAxis;
};

extern OpenGLCamera glCamera;