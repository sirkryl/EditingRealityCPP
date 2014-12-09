#pragma once
#include "common.h"

enum ManipulationMode { MANIPULATION_NONE, MANIPULATION_SCALE, MANIPULATION_ROTATE_X, MANIPULATION_ROTATE_Y, MANIPULATION_ROTATE_Z };
class SelectionHelper
{
public:
	//currently selected object (or -1 if none selected)
	int selectedIndex = -1;
	//objects for helper visualizations
	glm::vec3 hitPoint{ -1, -1, -1 };
	glm::vec3 nearPoint;

	void ProcessObjectManipulation();

	void ProcessSelectedObject();

	void ProcessPicking();

	void ProcessPlacing();

	void SetManipulationMode(ManipulationMode mode);
	ManipulationMode GetManipulationMode();

	void ProcessButtonClicks();
	void UnselectButtons();

	void Unselect();
	void ResetWallObject();
	void SelectWallObject();

	void ResetTransformationBasePoint();
private:

	ManipulationMode manipMode = MANIPULATION_NONE;


	glm::vec3 manipulationBasePoint;
	bool manipulationPointInitialized = false;
	int selectedButton = -1;
	int GetColorUnderCursor();
	bool PlacingPreview();
	bool ColorPlacing(bool preview);
	bool RayCastPlacing(bool preview);
	void GetRayOrientation(glm::vec3 v1, glm::vec3 v2, glm::vec3 normal, std::vector<int> &orientation);
	void RayCast(glm::vec3* v1, glm::vec3* v2);
	
};
extern SelectionHelper glSelector;
