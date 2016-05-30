#ifndef DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
#define DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H

#include <array>
#include <cgogn/topology/algos/distance_field.h>

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

// The link vector contains selected vertices in the link of a central vertex C
// For every dart d in this vector, phi2(d) belong to C
template <typename T, typename MAP>
int nb_marked_cc_in_link(
	MAP& map,
	std::vector<Dart> link,
	typename cgogn::CellMarkerStore<MAP, MAP::Vertex::ORBIT>& vertex_marker)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	int nb = 0;
	while (!link.empty())
	{
		// Search a marked vertex in the link (initially selected in the link)
		Dart d;
		do {
			d = link.back();
			link.pop_back();
		} while (!vertex_marker.is_marked(Vertex(d)) && !link.empty());

		// If a marked vertex has been found, its connected component is counted and unmarked
		if (vertex_marker.is_marked(Vertex(d)))
		{
			++nb;
			std::vector<Dart> cc;
			cc.push_back(d);
			while (!cc.empty())
			{
				Dart e = cc.back();
				vertex_marker.unmark(Vertex(e));
				cc.pop_back();

				map.foreach_incident_face(Edge(e), [&](Face f)
				{
					// The vertex of dart adj is adjacent to the vertex of e through an edge and
					// the dart phi2(adj) belongs to the central vertex C
					Dart adj = map.phi2(map.phi_1(map.phi_1(f.dart)));
					if (vertex_marker.is_marked(Vertex(adj)))
						cc.push_back(adj);
				});
			}
		}
	}
	return nb;
}

template <typename T, typename MAP, typename std::enable_if<MAP::DIMENSION == 3>::type* = nullptr>
CriticalVertex critical_vertex_type(
	MAP& map,
	const typename MAP::Vertex v,
	const typename MAP::template VertexAttribute<T>& scalar_field)
{
	using Vertex = typename MAP::Vertex;
	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;

	VertexMarkerStore sup_vertex_marker(map);
	VertexMarkerStore inf_vertex_marker(map);

	std::vector<Dart> sup_link;
	std::vector<Dart> inf_link;

	// Mark and store the vertices that are in the sup_link and inf_link of v
	T center_value = scalar_field[v];
	map.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
	{
		T value = scalar_field[u];
		if (value > center_value)
		{
			sup_vertex_marker.mark(u);
			sup_link.push_back(u.dart);
		}
		else if (value < center_value)
		{
			inf_vertex_marker.mark(u);
			inf_link.push_back(u.dart);
		}
		else
		{
//			std::cout << "Egal " << value << std::endl;
			sup_vertex_marker.mark(u);
			sup_link.push_back(u.dart);
		}
	});

	// Count the number of connected components in the inf and sup links
	int nb_inf = nb_marked_cc_in_link<T, MAP>(map, inf_link, inf_vertex_marker);
	int nb_sup = nb_marked_cc_in_link<T, MAP>(map, sup_link, sup_vertex_marker);

	if (nb_inf == 1 && nb_sup == 0)
		return CriticalVertex(CriticalVertexType::MAXIMUM);

	if (nb_inf == 0 && nb_sup == 1)
		return CriticalVertex(CriticalVertexType::MINIMUM);

	if (nb_inf == 1 && nb_sup == 1)
		return CriticalVertex(CriticalVertexType::REGULAR);

	if (nb_inf == 2 && nb_sup == 1)
		return CriticalVertex(CriticalVertexType::SADDLE, 12);

	if (nb_inf == 3 && nb_sup == 1)
		return CriticalVertex(CriticalVertexType::SADDLE, 13);

	if (nb_inf == 1 && nb_sup == 2)
		return CriticalVertex(CriticalVertexType::SADDLE, 21);

	if (nb_inf == 1 && nb_sup == 3)
		return CriticalVertex(CriticalVertexType::SADDLE, 31);

	if (nb_inf == 2 && nb_sup == 2)
		return CriticalVertex(CriticalVertexType::SADDLE, 22);

	std::cerr << "Warning: UNKNOW Volume Critical Type " << nb_inf << ", " << nb_sup << std::endl;
	return CriticalVertex(CriticalVertexType::UNKNOWN);
}

template <typename T, typename MAP, typename std::enable_if<MAP::DIMENSION == 2>::type* = nullptr>
CriticalVertex critical_vertex_type(
	MAP& map,
	const typename MAP::Vertex v,
	const typename MAP::template VertexAttribute<T>& scalar_field)
{
	using Vertex = typename MAP::Vertex;
	Dart next = v.dart;
	Dart prev;
	T center = scalar_field[v];
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
		if (current < center && previous > center)
			++down;
		else if (current > center && previous < center)
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

//template <typename T, typename MAP>
//CriticalVertex critical_vertex_type(
//	MAP& map,
//	const typename MAP::Vertex v,
//	const typename MAP::template VertexAttribute<T>& scalar_field)
//{
//	critical_vertex_type_concrete<MAP>(map, v, scalar_field);
//}

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

template <typename Scalar, typename MAP>
void extract_ascending_manifold(
		MAP& map,
		const typename MAP::template VertexAttribute<Scalar>& scalar_field,
		std::vector<typename MAP::Edge>& edges_set)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;

	typename MAP::template VertexAttribute<uint32> vertex_type =
			map.template add_attribute<uint32, Vertex::ORBIT>("vertex_type");

	// Search for every 1-saddles the starts of the ascending 1-manifolds
	// that link the saddles to the minima
	// These starts are the minima of the connected components of the inf_link of the saddles
	VertexMarkerStore vertex_marker(map);
	std::vector<Dart> inf_link;
	std::vector<Dart> saddles_to_minima;
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex type = critical_vertex_type<Scalar>(map, v, scalar_field);
		vertex_type[v] = type.v_;
		if (type.v_ == CriticalVertexType::SADDLE &&
				(type.n_ == 12 || type.n_ == 13) )
		{
			// Build the inf_link of v
			Scalar center_value = scalar_field[v];
			map.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
			{
				Scalar value = scalar_field[u];
				if (value < center_value)
				{
					vertex_marker.mark(u);
					inf_link.push_back(u.dart);
				}
			});
			// Foreach connected component of the link, search its local minima
			while (!inf_link.empty())
			{
				// Search a marked vertex in the link (initially selected in the link)
				Dart d;
				do {
					d = inf_link.back();
					inf_link.pop_back();
				} while (!vertex_marker.is_marked(Vertex(d)) && !inf_link.empty());

				// If a marked vertex has been found, its connected component is searched
				// for a minima and unmarked
				if (vertex_marker.is_marked(Vertex(d)))
				{
					std::vector<Dart> cc;
					Vertex min_cc = Vertex(d);
					Scalar min_value = scalar_field[min_cc];
					cc.push_back(d);
					while (!cc.empty())
					{
						Vertex current(cc.back());
						cc.pop_back();
						vertex_marker.unmark(current);
						Scalar current_value = scalar_field[current];
						if (current_value < min_value)
						{
							min_value = current_value;
							min_cc = current;
						}

						map.foreach_incident_face(Edge(current.dart), [&](Face f)
						{
							// The vertex of dart adj is adjacent to the vertex of e through an edge and
							// the dart phi2(adj) belongs to the central vertex C
							Dart adj = map.phi2(map.phi_1(map.phi_1(f.dart)));
							if (vertex_marker.is_marked(Vertex(adj)))
								cc.push_back(adj);
						});
					}
					saddles_to_minima.push_back(min_cc.dart);
				}
			}

		}
	});

	// For each found start descend its ascending 1-manifold
	while (!saddles_to_minima.empty())
	{
		Edge e(saddles_to_minima.back());
		saddles_to_minima.pop_back();
		edges_set.push_back(e);

		// Search for the next vertex in the descending path to the minimum
		Vertex min_vertex = Vertex(e.dart);
		Scalar min_value = scalar_field[min_vertex];
		map.foreach_adjacent_vertex_through_edge(min_vertex, [&](Vertex u)
		{
			Scalar current_value = scalar_field[u];
			if (current_value < min_value)
			{
				min_value = current_value;
				min_vertex = u;
			}
		});
		if (min_vertex.dart != e.dart) saddles_to_minima.push_back(min_vertex.dart);
	}

	map.remove_attribute(vertex_type);
}

template <typename Scalar, typename MAP>
void extract_descending_manifold(
		MAP& map,
		const typename MAP::template VertexAttribute<Scalar>& scalar_field,
		std::vector<typename MAP::Edge>& edges_set)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	using VertexMarkerStore = typename cgogn::CellMarkerStore<MAP, Vertex::ORBIT>;

	typename MAP::template VertexAttribute<uint32> vertex_type =
			map.template add_attribute<uint32, Vertex::ORBIT>("vertex_type");

	// Search for every 1-saddles the starts of the descending 1-manifolds
	// that link the saddles to the maxima
	// These starts are the maxima of the connected components of the sup_link of the saddles
	VertexMarkerStore vertex_marker(map);
	std::vector<Dart> sup_link;
	std::vector<Dart> saddles_to_maxima;
	map.foreach_cell([&](typename MAP::Vertex v)
	{
		CriticalVertex type = critical_vertex_type<Scalar>(map, v, scalar_field);
		vertex_type[v] = type.v_;
		if (type.v_ == CriticalVertexType::SADDLE &&
				(type.n_ == 21 || type.n_ == 31) )
		{
			// Build the inf_link of v
			Scalar center_value = scalar_field[v];
			map.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
			{
				Scalar value = scalar_field[u];
				if (value > center_value)
				{
					vertex_marker.mark(u);
					sup_link.push_back(u.dart);
				}
			});
			// Foreach connected component of the link, search its local minima
			while (!sup_link.empty())
			{
				// Search a marked vertex in the link (initially selected in the link)
				Dart d;
				do {
					d = sup_link.back();
					sup_link.pop_back();
				} while (!vertex_marker.is_marked(Vertex(d)) && !sup_link.empty());

				// If a marked vertex has been found, its connected component is searched
				// for a minima and unmarked
				if (vertex_marker.is_marked(Vertex(d)))
				{
					std::vector<Dart> cc;
					Vertex max_cc = Vertex(d);
					Scalar max_value = scalar_field[max_cc];
					cc.push_back(d);
					while (!cc.empty())
					{
						Vertex current(cc.back());
						cc.pop_back();
						vertex_marker.unmark(current);
						Scalar current_value = scalar_field[current];
						if (current_value > max_value)
						{
							max_value = current_value;
							max_cc = current;
						}

						map.foreach_incident_face(Edge(current.dart), [&](Face f)
						{
							// The vertex of dart adj is adjacent to the vertex of e through an edge and
							// the dart phi2(adj) belongs to the central vertex C
							Dart adj = map.phi2(map.phi_1(map.phi_1(f.dart)));
							if (vertex_marker.is_marked(Vertex(adj)))
								cc.push_back(adj);
						});
					}
					saddles_to_maxima.push_back(max_cc.dart);
				}
			}

		}
	});

	// For each found start descend its ascending 1-manifold
	while (!saddles_to_maxima.empty())
	{
		Edge e(saddles_to_maxima.back());
		saddles_to_maxima.pop_back();
		edges_set.push_back(e);

		// Search for the next vertex in the descending path to the minimum
		Vertex max_vertex = Vertex(e.dart);
		Scalar max_value = scalar_field[max_vertex];
		map.foreach_adjacent_vertex_through_edge(max_vertex, [&](Vertex u)
		{
			Scalar current_value = scalar_field[u];
			if (current_value > max_value)
			{
				max_value = current_value;
				max_vertex = u;
			}
		});
		if (max_vertex.dart != e.dart) saddles_to_maxima.push_back(max_vertex.dart);
	}

	map.remove_attribute(vertex_type);
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
