#include "ModelExporter.h"

#include "MeshContainer.h"
#include "ModelData.h"
#include "StringConverter.h"

#include <wrap/io_trimesh/export.h>
#include <glm/gtc/matrix_transform.hpp>
#include "DebugUtility.h"

using namespace std;

namespace InteractiveFusion
{
	ModelExporter::ModelExporter()
	{
	}


	ModelExporter::~ModelExporter()
	{
	}	

	void ModelExporter::SaveMeshToFile(VCGMesh &_mesh, string _fileName)
	{
		DebugUtility::DbgOut(L"ModelData::SaveMeshToFile::VN:", _mesh.vn);
		DebugUtility::DbgOut(L"ModelData::SaveMeshToFile::fileName:" + StringConverter::StringToWString(_fileName));
		if (strstr(_fileName.c_str(), ".ply"))
			vcg::tri::io::ExporterPLY<VCGMesh>::Save(_mesh, _fileName.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR);
		else if (strstr(_fileName.c_str(), ".obj"))
			vcg::tri::io::ExporterOBJ<VCGMesh>::Save(_mesh, _fileName.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR);
		else if (strstr(_fileName.c_str(), ".stl"))
			vcg::tri::io::ExporterSTL<VCGMesh>::Save(_mesh, _fileName.c_str(), true, vcg::tri::io::Mask::IOM_VERTCOLOR);
		else if (strstr(_fileName.c_str(), ".off"))
			vcg::tri::io::ExporterOFF<VCGMesh>::Save(_mesh, _fileName.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR);
		else
			vcg::tri::io::ExporterPLY<VCGMesh>::Save(_mesh, _fileName.c_str(), vcg::tri::io::Mask::IOM_VERTCOLOR);
	}

}