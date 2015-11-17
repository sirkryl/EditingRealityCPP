#pragma once

#include <boost/functional/hash.hpp>
#include "CommonStructs.h"

namespace InteractiveFusion {
	struct VertexHasher
	{
		std::size_t operator()(const Vertex& vertex) const
		{
			std::size_t seed = 0;

			boost::hash_combine(seed, boost::hash_value(vertex.x));
			boost::hash_combine(seed, boost::hash_value(vertex.y));
			boost::hash_combine(seed, boost::hash_value(vertex.z));

			return seed;
		}
	};
}