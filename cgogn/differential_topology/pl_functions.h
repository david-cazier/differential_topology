#ifndef DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
#define DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H

#include <array>
#include <cgogn/topology/types/critical_point.h>
#include <cgogn/topology/algos/distance_field.h>

namespace cgogn
{

namespace topology
{

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
		CriticalPoint type = critical_vertex_type<Scalar>(map, v, scalar_field);
		vertex_type[v] = type.v_;
		if (type.v_ == CriticalPoint::Type::SADDLE &&
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
		CriticalPoint type = critical_vertex_type<Scalar>(map, v, scalar_field);
		vertex_type[v] = type.v_;
		if (type.v_ == CriticalPoint::Type::SADDLE &&
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

} // namespace topology

} // namespace cgogn

#endif // DIFFERENTIAL_TOPOLOGY_PL_FUNCTIONS_H
