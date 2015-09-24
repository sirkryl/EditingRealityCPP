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

	void ScenarioExporter::Export(ModelData& _modelData)
	{
		Export(_modelData, ScenarioType::None);
	}

	void ScenarioExporter::Export(ModelData& _modelData, ScenarioType type)
	{
		std::string path = "data\\output\\";

		switch (type)
		{
			case ScenarioType::Basic:
				path += "basic\\";
				break;
			case ScenarioType::Bowling:
				path += "bowling\\";
				break;
			default:
				break;
		}

		namespace fs = boost::filesystem;
		fs::path path_to_remove(path);
		for (fs::directory_iterator end_dir_it, it(path_to_remove); it != end_dir_it; ++it) {
			remove_all(it->path());
		}

		VCGMesh combinedMesh;
		_modelData.CombineAndAlignModelData(combinedMesh);
		SaveMeshToFile(combinedMesh, path + "combinedMesh.ply");

		std::vector<int> planeIndices = _modelData.GetPlaneIndices();
		std::vector<int> objectIndices = _modelData.GetObjectIndices();
		int objectCount = 0;
		int planeCount = 0;
		for (int index = 0; index < _modelData.GetMeshCount(); index++)
		{
			auto findObjectIndex = std::find(std::begin(objectIndices), std::end(objectIndices), index);
			if (findObjectIndex != std::end(objectIndices))
			{
				VCGMesh outputMesh;
				if (!_modelData.GetAlignedVcgMesh(index, outputMesh))
					continue;
				SaveMeshToFile(outputMesh, path + "object_" + std::to_string(objectCount) + ".obj");
				objectCount++;
				continue;
			}
			auto findPlaneIndex = std::find(std::begin(planeIndices), std::end(planeIndices), index);
			if (findPlaneIndex != std::end(planeIndices))
			{
				VCGMesh outputMesh;
				if (!_modelData.GetAlignedVcgMesh(index, outputMesh))
					continue;
				SaveMeshToFile(outputMesh, path + "plane_" + std::to_string(planeCount) + ".obj");
				planeCount++;
				continue;
			}
		}
	}

	void ScenarioExporter::Export(ModelData& _modelData, int _index)
	{
		
	}
}