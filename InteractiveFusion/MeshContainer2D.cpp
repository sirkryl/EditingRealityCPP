#define NOMINMAX
#include "MeshContainer2D.h"
#include<wrap/io_trimesh/import.h>
#include <glm/gtc/matrix_transform.hpp>
#include "ColorCoder.h"
#include "VcgTypes.h"
#include "VcgException.h"
namespace InteractiveFusion
{
	MeshContainer2D::MeshContainer2D()
	{
		Renderable2D::Renderable2D();
	}


	MeshContainer2D::~MeshContainer2D()
	{

	}

	MeshContainer2D::MeshContainer2D(vector<Vertex> _vertices, vector<Triangle> _triangles) :
		Renderable2D(_vertices, _triangles)
	{
		
	}

	MeshContainer2D::MeshContainer2D(vector<Vertex> _vertices) :
		Renderable2D(_vertices)
	{
		DebugUtility::DbgOut(L"MeshContainer2D::MeshContainer2D()::_vertices size ", (int)_vertices.size());
		//Renderable2D::Renderable2D(_vertices);
	}

	void MeshContainer2D::LoadFromFile(const char* _fileName)
	{
		try
		{
			VCGMesh currentMesh;
			vcg::tri::io::ImporterPLY<VCGMesh>::Open(currentMesh, _fileName);

			scaleWithViewport = true;


			vertices.clear();
			triangles.clear();

			VCGMesh::VertexIterator vi;

			std::vector<int> VertexId((currentMesh).vert.size());
			int numvert = 0;

			for (vi = (currentMesh).vert.begin(); vi != (currentMesh).vert.end(); ++vi) if (!(*vi).IsD())
			{
				VertexId[vi - (currentMesh).vert.begin()] = numvert;

				Vertex vertex;
				vertex.x = (*vi).P()[0];
				vertex.y = (*vi).P()[1];
				vertex.z = (*vi).P()[2];
				vertex.normal_x = (*vi).N()[0];
				vertex.normal_y = (*vi).N()[1];
				vertex.normal_z = (*vi).N()[2];
				vertex.r = (*vi).C()[0] / 255.0f;
				vertex.g = (*vi).C()[1] / 255.0f;
				vertex.b = (*vi).C()[2] / 255.0f;
				vertices.push_back(vertex);

				numvert++;
			}

			for (VCGMesh::FaceIterator fi = (currentMesh).face.begin(); fi != (currentMesh).face.end(); ++fi) if (!(*fi).IsD())
			{
				Triangle triangle;
				triangle.v1 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(0))];
				triangle.v2 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(1))];
				triangle.v3 = VertexId[vcg::tri::Index((currentMesh), (*fi).V(2))];

				triangles.push_back(triangle);
			}
		}
		catch (std::exception& e)
		{
			std::stringstream ss;
			ss << "Exception in MeshContainer::LoadFromFile() with parameter ";
			ss << _fileName;
			ss << ", Exception type: ";
			ss << e.what();
			throw new VcgException(ss.str().c_str());
		}

	}

}