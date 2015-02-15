#include "IconData.h"
#include "DebugUtility.h"


using namespace std;

namespace InteractiveFusion {

	IconData::IconData()
	{
	}


	IconData::~IconData()
	{
	}

	bool IconData::LoadFromFile(const char* _fileName, int _colorCode)
	{
		std::unordered_map<int, unique_ptr<MeshContainer2D>>::iterator it = defaultIconData.find(_colorCode);
		if (it == defaultIconData.end())
		{
			defaultIconData[_colorCode] = unique_ptr<MeshContainer2D>(new MeshContainer2D);
			defaultIconData[_colorCode]->LoadFromFile(_fileName);
			defaultIconData[_colorCode]->SetColorCode(_colorCode);
			//defaultIconData[_colorCode]->Clean();
			//defaultIconData[_colorCode]->CopyInternalToVisibleData();
			defaultIconData[_colorCode]->UpdateEssentials();
			defaultIconData[_colorCode]->SetShaderProgram(defaultShaderProgram);
			isHoveredMap[_colorCode] = false;
			return true;
		}
		return false;
	}

	bool IconData::LoadFromFile(const char* _fileNameDefault, const char* _fileNameHover, int _colorCodeDefault)
	{
		if (!LoadFromFile(_fileNameDefault, _colorCodeDefault))
			return false;

		hoveredIconData[_colorCodeDefault] = unique_ptr<MeshContainer2D>(new MeshContainer2D);
		hoveredIconData[_colorCodeDefault]->LoadFromFile(_fileNameHover);
		hoveredIconData[_colorCodeDefault]->SetColorCode(_colorCodeDefault);
		//hoveredIconData[_colorCodeDefault]->Clean();
		//hoveredIconData[_colorCodeDefault]->CopyInternalToVisibleData();
		hoveredIconData[_colorCodeDefault]->UpdateEssentials();
		hoveredIconData[_colorCodeDefault]->SetShaderProgram(defaultShaderProgram);
		return true;
	}

	void IconData::GenerateBuffers()
	{
		for (auto &icon : defaultIconData)
			icon.second->GenerateBuffers();
		for (auto &icon : hoveredIconData)
			icon.second->GenerateBuffers();
	}

	bool IconData::IsReadyForRendering()
	{
		for (auto &icon : defaultIconData)
		{
			if (!icon.second->AreBuffersInitialized())
				return false;
		}
		for (auto &icon : hoveredIconData)
			if (!icon.second->AreBuffersInitialized())
				return false;
		return true;
	}

	void IconData::DrawForColorPicking(int _colorCode, int _viewportWidth, int _viewportHeight)
	{
		if (!IsValidKey(_colorCode))			
			return;
		
		if (defaultIconData[_colorCode]->AreBuffersInitialized())
			defaultIconData[_colorCode]->DrawForColorPicking(_viewportWidth, _viewportHeight);
	}

	void IconData::DrawForColorPicking(int _viewportWidth, int _viewportHeight)
	{
		for (auto &icon : defaultIconData)
		{
			icon.second->DrawForColorPicking(_viewportWidth, _viewportHeight);
		}
	}

	void IconData::Draw(int _colorCode, int _viewportWidth, int _viewportHeight)
	{
		if (!IsValidKey(_colorCode))
			return;
		if (isHoveredMap[_colorCode])
		{
			if (hoveredIconData[_colorCode]->AreBuffersInitialized())
				hoveredIconData[_colorCode]->Draw(_viewportWidth, _viewportHeight);
		}
		else
		{
			if (defaultIconData[_colorCode]->AreBuffersInitialized())
				defaultIconData[_colorCode]->Draw(_viewportWidth, _viewportHeight);
		}
	}

	void IconData::Draw(int _viewportWidth, int _viewportHeight)
	{
		for (auto &icon : defaultIconData)
		{
			if (isHoveredMap[icon.first])
				hoveredIconData[icon.first]->Draw(_viewportWidth, _viewportHeight);
			else
				icon.second->Draw(_viewportWidth, _viewportHeight);
		}
	}

	void IconData::SetHovered(int _colorCode, bool _isHovered)
	{
		if (!IsValidKey(_colorCode))
			return;
		isHoveredMap[_colorCode] = _isHovered;
	}

	bool IconData::IsValidKey(int _colorCode)
	{
		std::unordered_map<int, unique_ptr<MeshContainer2D>>::iterator it = defaultIconData.find(_colorCode);
		if (it != defaultIconData.end())
			return true;
		return false;
	}

	void IconData::SetDefaultShaderProgram(OpenGLShaderProgram _defaultProgram)
	{
		defaultShaderProgram = _defaultProgram;
	}

	void IconData::CleanUp()
	{
		for (auto &icon : defaultIconData)
			icon.second->CleanUp();
		for (auto &icon : hoveredIconData)
			icon.second->CleanUp();
		defaultIconData.clear();
		isHoveredMap.clear();
	}
}
