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

template <typename T, typename MAP>
typename MAP::Vertex find_one_ring_maxima(
		MAP& map,
		typename MAP::Vertex u,
		const typename MAP::template VertexAttribute<T>& scalar_field)
{
	using Vertex = typename MAP::Vertex;

	Vertex max_vertex = u;
	T max_value = scalar_field[u];

	map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
	{
		if(scalar_field[v] > max_value)
		{
			max_value = scalar_field[v];
			max_vertex = v;
		}
	});
	return max_vertex;
}

template <typename T, typename MAP>
typename MAP::Vertex find_one_ring_minima(
		MAP& map,
		typename MAP::Vertex u,
		const typename MAP::template VertexAttribute<T>& scalar_field)
{
	using Vertex = typename MAP::Vertex;

	Vertex min_vertex = u;
	T min_value = scalar_field[u];

	map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
	{
		if(scalar_field[v] < min_value)
		{
			min_value = scalar_field[v];
			min_vertex = v;
		}
	});
	return min_vertex;
}


template <typename T, typename MAP>
CriticalVertex critical_vertex_type(
		MAP& map,
		const typename MAP::Vertex v,
		const typename MAP::template VertexAttribute<T>& scalar_field)
{
	using Vertex = typename MAP::Vertex;

	Vertex max = find_one_ring_maxima<T, MAP>(map, v, scalar_field);
	std::cout << "max_value : " << max <<
				 " center : " << scalar_field[v] << std::endl;
	if (scalar_field[max] < scalar_field[v]) {
		return CriticalVertex(CriticalVertexType::MAXIMUM);
		std::cout << "MAXIMUM" << std::endl;
	}

	Vertex min = find_one_ring_minima<T, MAP>(map, v, scalar_field);
	std::cout << "max_value : " << max <<
				 " center : " << scalar_field[v] << std::endl;

	if (scalar_field[min] > scalar_field[v]) {
		return CriticalVertex(CriticalVertexType::MINIMUM);
		std::cout << "MINIMUM" << std::endl;
	}

	return CriticalVertex(CriticalVertexType::REGULAR);

	// Ci-dessous le code fonctionnant pour les surfaces
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
		const typename MAP::template VertexAttribute<Scalar>& scalar_field,
		std::vector<typename MAP::Edge>& level_lines)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;
	using FaceMarkerStore = typename cgogn::CellMarkerStore<MAP, Face::ORBIT>;

	VertexMarkerStore vertex_marker(map);
	FaceMarkerStore face_marker(map);
	std::vector<Face> level_set_faces;

	typename MAP::template VertexAttribute<uint32> vertex_type =
			map.template add_attribute<uint32, Vertex::ORBIT>("vertex_type");

	using my_pair = std::pair<Scalar, unsigned int>;
	using my_queue = std::priority_queue<my_pair, std::vector<my_pair>>;

	my_queue level_sets_queue;
	my_queue vertex_queue;

	// Add all local maxima as level set sources
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex type = critical_vertex_type<Scalar>(map, v, scalar_field);
		vertex_type[v] = type.v_;
		if (type.v_ == CriticalVertexType::MAXIMUM)
		{
			level_sets_queue.push(std::make_pair(scalar_field[v], v.dart.index));
		}
	});

	// Tant qu'il reste des maxima locaux => génère un level set
	while (!level_sets_queue.empty()) {
		// Initialise un nouveau front pour calculer le level set suivant
		my_pair p = level_sets_queue.top();
		level_sets_queue.pop();
		vertex_queue.push(p);

		Scalar saddle_value = Scalar(0);

		// Tant qu'il reste des sommets dans le front courrant
		while (!vertex_queue.empty())
		{
			Vertex u = Vertex(Dart(vertex_queue.top().second));
			vertex_queue.pop();

			// We are still in the current level set
			if (!vertex_marker.is_marked(u) && scalar_field[u] > saddle_value) {
				// We reach a saddle (the nearest one)
				if (vertex_type[u] == CriticalVertexType::SADDLE) {
					saddle_value = scalar_field[u];
				}
				else {
					vertex_marker.mark(u);
					// Extend the front of the level set
					map.foreach_adjacent_vertex_through_edge(u, [&] (Vertex v)
					{
						if (!vertex_marker.is_marked(v) && scalar_field[v] > saddle_value)
						{
							vertex_queue.push(std::make_pair(scalar_field[v], v.dart.index));
						}
					});
					map.foreach_incident_face(u, [&] (Face f)
					{
						if (!face_marker.is_marked(f))
						{
							level_set_faces.push_back(f);
							face_marker.mark(f);
						}
					});
				}
			}
		}
		// The marked vertices and faces define the interior and closure of the level set
		for (Face f : level_set_faces)
		{
			map.foreach_incident_edge(f, [&](Edge e)
			{
				std::pair<Vertex, Vertex> p = map.vertices(e);
				if (!vertex_marker.is_marked(p.first) && !vertex_marker.is_marked(p.second))
					level_lines.push_back(e);
			});
		}
		level_set_faces.clear();
		vertex_marker.unmark_all();
		face_marker.unmark_all();
	}
	map.remove_attribute(vertex_type);
}

}

#endif // DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
