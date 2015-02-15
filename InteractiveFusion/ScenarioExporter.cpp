#include "ScenarioExporter.h"
#include "ModelData.h"

#include <boost/filesystem.hpp>
namespace InteractiveFusion {
	ScenarioExporter::ScenarioExporter()
	{
	}


	ScenarioExporter::~ScenarioExporter()
	{
	}

	void ScenarioExporter::Export(ModelData* _modelData)
	{
		namespace fs = boost::filesystem;
		fs::path path_to_remove("data\\output\\");
		for (fs::directory_iterator end_dir_it, it(path_to_remove); it != end_dir_it; ++it) {
			remove_all(it->path());
		}
		std::vector<int> planeIndices = _modelData->GetPlaneIndices();
		std::vector<int> objectIndices = _modelData->GetObjectIndices();
		int objectCount = 0;
		int planeCount = 0;
		for (int index = 0; index < _modelData->GetMeshCount(); index++)
		{
			auto findObjectIndex = std::find(std::begin(objectIndices), std::end(objectIndices), index);
			if (findObjectIndex != std::end(objectIndices))
			{
				VCGMesh outputMesh;
				if (!_modelData->GetAlignedVcgMesh(index, outputMesh))
					continue;
				SaveMeshToFile(outputMesh, "data\\output\\object_" + std::to_string(objectCount) + ".obj");
				objectCount++;
				continue;
			}
			auto findPlaneIndex = std::find(std::begin(planeIndices), std::end(planeIndices), index);
			if (findPlaneIndex != std::end(planeIndices))
			{
				VCGMesh outputMesh;
				if (!_modelData->GetAlignedVcgMesh(index, outputMesh))
					continue;
				SaveMeshToFile(outputMesh, "data\\output\\plane_" + std::to_string(planeCount) + ".obj");
				planeCount++;
				continue;
			}
		}
	}

	void ScenarioExporter::Export(ModelData* _modelData, int _index)
	{
		
	}
}