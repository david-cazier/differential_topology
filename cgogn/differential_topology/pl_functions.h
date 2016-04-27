#ifndef DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
#define DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H

#include <array>
#include <cgogn/dijkstra.h>

namespace cgogn
{

template <typename Scalar, typename MAP>
struct EquivalenceClass
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	Vertex vertex_;
	Scalar value_;
	std::vector<Vertex> vertices_;
	std::vector<Edge> edges_;
};

enum CriticalVertexType: unsigned int
{
	REGULAR = 0,
	MAXIMUM = 1,
	MINIMUM = 2,
	SADDLE  = 4,
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

	Dart next = v.dart;
	Dart prev;
	T center  = scalar_field[v];
	T previous;
	int up = 0;
	int down = 0;

	// Find a vertex whose scalar field value is distinct from the center
	do
	{
		previous = scalar_field[Vertex(map.phi_1(next))];
		prev = next;
		next = map.phi1(map.phi2(next));
	} while (next != v.dart && previous == center);

	if (next == v.dart)
		return CriticalVertex(CriticalVertexType::REGULAR);

	// Count the variation of the scalar field values around the vertex
	// i.e. how many times the value becomes greater or lower than the center
	next = prev;
	do
	{
		T current = scalar_field[Vertex(map.phi2(next))];
		if(current < center && previous > center)
			++down;
		else if(current > center && previous < center)
			++up;
		// Skip the vertex whose value is equal to the center
		// (that alter the detection of variations)
		if (current != center) previous = current;
		next = map.phi1(map.phi2(next));
	} while (next != prev);

	// All values are greater than the center
	if (up == 0 && down == 0 && previous > center)
		return CriticalVertex(CriticalVertexType::MINIMUM);

	// All values are lower than the center
	if (up == 0 && down == 0 && previous < center)
		return CriticalVertex(CriticalVertexType::MAXIMUM);

	// A unique varation in both direction
	if (up == 1 && down == 1)
		return CriticalVertex(CriticalVertexType::REGULAR);

	// More than one varation in both direction
	if (up == down)
		return CriticalVertex(CriticalVertexType::SADDLE, up);

	std::cerr << "Warning: UNKNOW Critical Type" << std::endl;
	return CriticalVertex(CriticalVertexType::UNKNOWN);
}

template <typename T, typename MAP>
void extract_maxima(
		MAP& map,
		const typename MAP::template VertexAttribute<T>& scalar_field,
		std::vector<typename MAP::Vertex>& maxima)
{
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex i = critical_vertex_type<T>(map,v,scalar_field);
		if (i.v_ == CriticalVertexType::MAXIMUM)
			maxima.push_back(v);
	});
}

template <typename T, typename MAP>
void extract_extrema(
		MAP& map,
		const typename MAP::template VertexAttribute<T>& scalar_field,
		std::vector<typename MAP::Vertex>& extema)
{
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex i = critical_vertex_type<T>(map,v,scalar_field);
		if (i.v_ == CriticalVertexType::MAXIMUM || i.v_ == CriticalVertexType::MINIMUM)
			extema.push_back(v);
	});
}

template <typename T, typename MAP>
void extract_critical_points(
		MAP& map,
		const typename MAP::template VertexAttribute<T>& scalar_field,
		std::vector<typename MAP::Vertex>& maxima,
		std::vector<typename MAP::Vertex>& minima,
		std::vector<typename MAP::Vertex>& saddles)
{
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex i = critical_vertex_type<T>(map,v,scalar_field);
		if (i.v_ == CriticalVertexType::MAXIMUM)
			maxima.push_back(v);
		if (i.v_ == CriticalVertexType::MINIMUM)
			minima.push_back(v);
		else if (i.v_ == CriticalVertexType::SADDLE)
			saddles.push_back(v);
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
		const std::vector<typename MAP::Vertex> vertices,
		const typename MAP::template EdgeAttribute<T>& weight,
		typename MAP::template VertexAttribute<T>& distance_to_source)
{
	using Vertex = typename MAP::Vertex;
	typename MAP::template VertexAttribute<Vertex> path_to_source = map.template add_attribute<Vertex, Vertex::ORBIT>("path_to_source");

	cgogn::dijkstra_compute_paths<T>(map, weight, vertices, distance_to_source, path_to_source);
	map.remove_attribute(path_to_source);
}

template <typename T, typename MAP>
void normalized_geodesic_distance_pl_function(
		MAP& map,
		const std::vector<typename MAP::Vertex> vertices,
		const typename MAP::template EdgeAttribute<T>& weight,
		typename MAP::template VertexAttribute<T>& distance_to_source)
{
	using Vertex = typename MAP::Vertex;
	typename MAP::template VertexAttribute<Vertex> path_to_source = map.template add_attribute<Vertex, Vertex::ORBIT>("path_to_source");

	cgogn::dijkstra_compute_normalized_paths<T>(map, weight, vertices, distance_to_source, path_to_source);
	map.remove_attribute(path_to_source);
}

template <typename Scalar, typename MAP>
void extract_level_sets(
		MAP& map,
		typename MAP::template VertexAttribute<Scalar>& scalar_field,
		typename MAP::template VertexAttribute<Scalar>& level_sets)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	typename MAP::template VertexAttribute<uint32> vertex_type =
			map.template add_attribute<uint32, Vertex::ORBIT>("vertex_type");

	for(auto& d : level_sets)
		d = Scalar(0);	// To mark unvisited vertices

	using my_pair = std::pair<Scalar, unsigned int>;
	using my_queue = std::priority_queue<my_pair, std::vector<my_pair>>;

	my_queue level_sets_queue;
	my_queue vertex_queue;

	// Add all maxima as level set sources
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex type = critical_vertex_type<Scalar>(map, v, scalar_field);
		vertex_type[v] = type.v_;
		if (type.v_ == CriticalVertexType::MAXIMUM)
		{
			level_sets_queue.push(std::make_pair(scalar_field[v], v.dart.index));
		}
	});

	// Pour chaque level set
	uint32 current_level_set_id = 1;
	while (!level_sets_queue.empty()) {
		// Set the source vertex of the next level set
		std::cout << "New front : " << current_level_set_id << std::endl;
		my_pair p;
		do {
			p = level_sets_queue.top();
			level_sets_queue.pop();
		} while (!level_sets_queue.empty() && level_sets[Vertex(Dart(p.second))] != Scalar(0));

		if (!level_sets_queue.empty()) {
			vertex_queue.push(p);
			level_sets[Vertex(Dart(p.second))] = Scalar(current_level_set_id);
		}

		// Pour chaque sommet du front du level set
		int nb_vertex = 0;
		while (!vertex_queue.empty())
		{
			Vertex u = Vertex(Dart(vertex_queue.top().second));
			vertex_queue.pop();
			++nb_vertex;

			Scalar current_scalar = scalar_field[u];
			Scalar minimal_scalar = Scalar(0);

			map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
			{
				if (vertex_type[v] == CriticalVertexType::SADDLE || level_sets[v] != Scalar(0)) {
					// Set as visited if not
					if (level_sets[v] != Scalar(0))
						level_sets[v] = current_level_set_id;

					std::cout << "saddle or topological merge" << std::endl;
					// Add the vertices adjacent to the saddle to the level set queue
					minimal_scalar = scalar_field[v];
					map.foreach_adjacent_vertex_through_edge(v, [&](Vertex w)
					{
						if (scalar_field[w] < scalar_field[v]) {
							// Get the maximal scalar lower than current in the 1-ring
							// (this is the lower bound of the current level set)
							if (scalar_field[w] > minimal_scalar)
								minimal_scalar = scalar_field[w];
							// Add potential new sources of level sets
							if (level_sets[w] == Scalar(0) && scalar_field[w] < scalar_field[v])
								vertex_queue.push(std::make_pair(scalar_field[w], w.dart.index));
						}
					});
					// Stop the expansion of this level set (by emptying the queue)
//					while (!vertex_queue.empty()) vertex_queue.pop();
				}
				else if (scalar_field[v] > minimal_scalar)
				{
					level_sets[v] = current_level_set_id;		// Set as visited
					if (scalar_field[v] < current_scalar)
						vertex_queue.push(std::make_pair(scalar_field[v], v.dart.index));
				}
			});
		}
		std::cout << nb_vertex << "in the front" << std::endl;
		++current_level_set_id;
	}
	map.remove_attribute(vertex_type);
}

}

#endif // DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
