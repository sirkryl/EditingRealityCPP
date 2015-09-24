#pragma once
#include "ModelExporter.h"
#include "EnumDeclarations.h"

namespace InteractiveFusion {
	class ScenarioExporter :
		public ModelExporter
	{
	public:
		ScenarioExporter();
		~ScenarioExporter();

		virtual void Export(ModelData& _modelData, int _index);
		virtual void Export(ModelData& _modelData);
		virtual void Export(ModelData& _modelData, ScenarioType type);

	protected:
	};

}