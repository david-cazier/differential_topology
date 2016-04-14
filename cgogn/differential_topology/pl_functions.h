#ifndef DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
#define DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H

#include <array>
#include <cgogn/dijkstra.h>

namespace cgogn
{

//template <typename VEC3_T, typename MAP>
//inline typename VEC3_T::Scalar edge_length(const MAP& map, typename MAP::Edge e, const typename MAP::template VertexAttribute<VEC3_T>& position)
//{
//    using Vertex = typename MAP::Vertex;

//    VEC3_T vec = position[Vertex(map.phi1(e))];
//    vec -= position[Vertex(e)];
//    return vec.norm();
//}


enum CriticalVertexType: unsigned int
{
	REGULAR = 0,
	MAXIMUM = 1,
	MINIMUM = 2,
	SADDLE = 4,
	UNKNOWN = 8
};

struct CriticalVertex
{
	unsigned int n_;
	CriticalVertexType v_;

	inline CriticalVertex(CriticalVertexType v): v_(v), n_(0)
	{}

	inline CriticalVertex(CriticalVertexType v, unsigned int n) : v_(v), n_(n)
	{}
};

// f stands for general piecewise linear function
template <typename T, typename MAP>
CriticalVertex critical_vertex_type(
		MAP& map,
		const typename MAP::Vertex v,
		const typename MAP::template VertexAttribute<T>& scalar_field)
{
	using Vertex = typename MAP::Vertex;
	T center = scalar_field[v];
	int up = 0;
	int down = 0;

	Dart vit = v.dart;
	T previous = scalar_field[Vertex(map.phi2(v.dart))];

	map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex u)
	{
		T current = scalar_field[u];
		if(current < center && previous > center)
			++down;
		if(current > center && previous < center)
			++up;
		previous = current;
	});

	// si on a fait le tour sans aucune variation, c'est un extremum local
	if(up == 0 && down == 0 && previous > center)
		return CriticalVertex(CriticalVertexType::MINIMUM);

	if(up == 0 && down == 0 && previous < center)
		return CriticalVertex(CriticalVertexType::MAXIMUM);

	if(up == 1 && down == 1)
		return CriticalVertex(CriticalVertexType::REGULAR);

	if (up == down)
		return CriticalVertex(CriticalVertexType::SADDLE, up);

	return CriticalVertex(CriticalVertexType::UNKNOWN);
}

template <typename T, typename MAP>
void extract_feature_points(
		MAP& map,
		const typename MAP::template VertexAttribute<T>& scalar_field,
		std::vector<typename MAP::Vertex>& vertices)
{
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex i = critical_vertex_type<T>(map,v,scalar_field);
		if(i.v_ == CriticalVertexType::MAXIMUM || i.v_ == CriticalVertexType::MINIMUM)
			vertices.push_back(v);
	});
}

/**
 * height PL Morse function:
 * \f$f: S \rightarrow R\f$ such that \f$f(v) = y\f$ for each vertex of \f$S\f$.
 */
template <typename T, typename MAP>
void height_pl_function(
		MAP& map,
		const typename MAP::template VertexAttribute<T>& position,
		typename MAP::template VertexAttribute<typename T::Scalar>& scalar_field)
{
	map.foreach_cell([&] (typename MAP::Vertex v)
	{
		scalar_field[v] = position[v][0];
	});
}

template <typename T, typename MAP>
void geodesic_distance_pl_function(
		MAP& map,
		const typename MAP::Vertex v,
		const typename MAP::template EdgeAttribute<T>& weight,
		typename MAP::template VertexAttribute<T>& distance_to_source)
{
	using Vertex = typename MAP::Vertex;
	typename MAP::template VertexAttribute<Vertex> path_to_source = map.template add_attribute<Vertex, Vertex::ORBIT>("path_to_source");

	cgogn::dijkstra_compute_paths<T>(map, weight, {v}, distance_to_source, path_to_source);
	map.remove_attribute(path_to_source);
}

}

#endif // DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
