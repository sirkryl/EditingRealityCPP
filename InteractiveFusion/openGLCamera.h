#pragma once
#include <glm/glm.hpp>

namespace InteractiveFusion {
	
	class OpenGLCamera
	{
	public:

		OpenGLCamera();
		
		void Initialize(glm::vec3 _position, glm::vec3 _lookAt, glm::vec3 _upDirection);
		void Initialize(glm::vec3 _position, glm::vec3 _lookAt, glm::vec3 _upDirection, float _zoomStep, float _orbitXStep, float _orbitYStep, float _strafeXStep, float _strafeYStep);
		void SetRotationPoint(glm::vec3 point);

		void Update(bool _enableMouseInput, int _mouseWheelDelta);

		

		void ResetCameraPosition();

		glm::mat4 GetViewMatrix();



	private:

		glm::vec3 camPosition;
		glm::vec3 camLookAt;
		glm::vec3 camUpDirection;
		glm::vec3 camDirection;
		glm::vec3 camRight;

		glm::vec3 rotationPoint;

		

		
		float zoomFactor = 1.0f;
		float zoomStep = 0.2f;
		float orbitXStep = 0.2f;
		float orbitYStep = 0.15f;
		float completeStrafeX = 0.0f;
		float completeStrafeY = 0.0f;
		float strafeXStep = 0.002f;
		float strafeYStep = 0.002f;

		bool firstTimeLeftMouseClick = false;
		bool firstTimeMiddleMouseClick = false;

		bool enableMouseInput = false;
		int mouseWheelDelta = 0;

		void Orbit();
		void Zoom();
		void Strafe();
	};
}