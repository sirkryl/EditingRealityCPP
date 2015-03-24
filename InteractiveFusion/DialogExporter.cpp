#include "DialogExporter.h"
#include <Shlobj.h>
#include "ModelData.h"

namespace InteractiveFusion {
	using namespace std;
	DialogExporter::DialogExporter()
	{
	}


	DialogExporter::~DialogExporter()
	{
	}

	void DialogExporter::Export(ModelData& _modelData, int _index)
	{
		string fileName = GetFileNameFromSaveFileDialog();
		if (fileName.empty())
			return;

		VCGMesh outputMesh;// = _modelData.GetAlignedVcgMesh(_index, outputMesh);
		if (!_modelData.GetAlignedVcgMesh(_index, outputMesh))
			return;
		SaveMeshToFile(outputMesh, fileName);
	}

	void DialogExporter::Export(ModelData& _modelData)
	{
		string fileName = GetFileNameFromSaveFileDialog();
		if (fileName.empty())
			return;
		VCGMesh combinedMesh;
		_modelData.CombineAndAlignModelData(combinedMesh);
		SaveMeshToFile(combinedMesh, fileName);
	}

	string DialogExporter::GetFileNameFromSaveFileDialog()
	{
		OPENFILENAME ofn;

		char szFileName[MAX_PATH] = "";

		ZeroMemory(&ofn, sizeof(ofn));

		ofn.lStructSize = sizeof(ofn);
		ofn.hwndOwner = NULL;
		ofn.lpstrFilter = (LPCWSTR)L"PLY (*.ply)\0*.ply;\0STL (*.stl)\0*.stl\0OBJ (*.obj)\0*.obj\0OFF (*.off)\0*.off\0All Files (*.*)\0*.*\0";
		ofn.lpstrFile = (LPWSTR)szFileName;
		ofn.nMaxFile = MAX_PATH;
		ofn.Flags = OFN_EXPLORER | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT;
		ofn.lpstrDefExt = (LPCWSTR)L"ply";

		if (!GetSaveFileName(&ofn))
			return "";

		char fileNameBuffer[500];

		// First arg is the pointer to destination char, second arg is
		// the pointer to source wchar_t, last arg is the size of char buffer
		wcstombs(fileNameBuffer, ofn.lpstrFile, 500);

		return string(fileNameBuffer);
		//return ofn.lpstrFile
	}
}