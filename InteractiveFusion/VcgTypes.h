#pragma once
#include<vcg/complex/complex.h>
#include "DebugUtility.h"
namespace InteractiveFusion {
	class VCGFace;
	class VCGVertex;
	class VCGEdge;

	struct VCGUsedTypes : public vcg::UsedTypes < vcg::Use<VCGVertex>::AsVertexType, vcg::Use<VCGEdge>::AsEdgeType, vcg::Use<VCGFace>::AsFaceType > {};

	class VCGVertex : public vcg::Vertex < VCGUsedTypes, vcg::vertex::VEAdj, vcg::vertex::Mark, vcg::vertex::VFAdj, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags > {};
	class VCGEdge : public vcg::Edge < VCGUsedTypes > {};
	class VCGFace : public vcg::Face < VCGUsedTypes, vcg::face::VFAdj, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags, vcg::face::Normal3f, vcg::face::Mark > {};
	class VCGMesh :
		public vcg::tri::TriMesh < std::vector<VCGVertex>, std::vector<VCGFace>, std::vector<VCGEdge> >
	{};
}