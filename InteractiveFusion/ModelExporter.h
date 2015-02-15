#pragma once

#include <string>
#include "VcgTypes.h"
namespace InteractiveFusion
{
	class MeshContainer;
	class ModelData;
	class ModelExporter
	{
	public:
		ModelExporter();
		~ModelExporter();
		virtual void Export(ModelData* _modelData, int _index) = 0;
		virtual void Export(ModelData* _modelData) = 0;

	protected:
		void SaveMeshToFile(VCGMesh &_mesh, std::string _fileName);
	};
}