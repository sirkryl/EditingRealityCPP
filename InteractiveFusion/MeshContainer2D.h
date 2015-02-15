#pragma once
#include "Renderable2D.h"


namespace InteractiveFusion
{ 
	class MeshContainer2D :
		public Renderable2D
	{
	public:
		MeshContainer2D();
		~MeshContainer2D();
		MeshContainer2D(std::vector<Vertex> _vertices);
		MeshContainer2D(std::vector<Vertex> _vertices, std::vector<Triangle> _triangles);

		void LoadFromFile(const char* filename);
	};
}

