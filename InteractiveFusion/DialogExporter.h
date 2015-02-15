#pragma once
#include "ModelExporter.h"

namespace InteractiveFusion {
	class DialogExporter :
		public ModelExporter
	{
	public:
		DialogExporter();
		~DialogExporter();

		virtual void Export(ModelData* _modelData, int _index);
		virtual void Export(ModelData* _modelData);

	protected:
		std::string GetFileNameFromSaveFileDialog();
	};

}