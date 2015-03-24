#pragma once
#include "ModelExporter.h"
namespace InteractiveFusion {
	class ScenarioExporter :
		public ModelExporter
	{
	public:
		ScenarioExporter();
		~ScenarioExporter();

		virtual void Export(ModelData& _modelData, int _index);
		virtual void Export(ModelData& _modelData);

	protected:
	};

}