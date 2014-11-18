#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>

enum CameraMode { CAMERA_FREE, CAMERA_SENSOR };
class OpenGLCamera
{
public:

	CameraMode mode = CAMERA_SENSOR;
	OpenGLCamera();
	OpenGLCamera(glm::vec3 position, glm::vec3 lookAt, glm::vec3 upDirection, float mvSpeed, float sensitivity);

	void SetRotationPoint(glm::vec3 point);

	void Orbit();
	void UpdateZoom();
	void UpdateStrafe();
	void RotateWithMouse();
	void GetRotation(glm::mat4 &rotation);
	glm::vec3 GetDirection();
	void Update();
	void ResetCameraPosition();
	glm::mat4 GetViewMatrix();
	glm::vec3 GetPosition();
	void ResetMouse();
	glm::vec3 GetUpDirection();
	// Functions that get viewing angles
	float GetAngleX(), GetAngleY();

	// Constructors
	

private:
	glm::vec3 camPosition;
	glm::vec3 camLookAt;
	glm::vec3 camUpDirection;
	glm::vec3 moveBy;
	glm::vec3 oldMoveBy;

	glm::vec3 rotationPoint;


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