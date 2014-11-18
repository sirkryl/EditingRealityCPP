#pragma once
#include "common.h"

class SelectionHelper
{
public:
	//currently selected object (or -1 if none selected)
	int selectedIndex = -1;
	//objects for helper visualizations
	glm::vec3 hitPoint{ -1, -1, -1 };
	glm::vec3 nearPoint;

	void ProcessSelectedObject();

	void ProcessPicking();

	void ProcessPlacing();

	void ResetWallObject();
	void SelectWallObject();
private:
	int GetColorUnderCursor();
	bool PlacingPreview();
	bool ColorPlacing(bool preview);
	bool RayCastPlacing(bool preview);
	void GetRayOrientation(glm::vec3 v1, glm::vec3 v2, glm::vec3 normal, std::vector<int> &orientation);
	void RayCast(glm::vec3* v1, glm::vec3* v2);
	
};
extern SelectionHelper glSelector;
