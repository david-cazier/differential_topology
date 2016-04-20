/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef DIFFERENTIAL_TOPOLOGY_REEB_GRAPH_H_
#define DIFFERENTIAL_TOPOLOGY_REEB_GRAPH_H_

#include "../directed_graph.h"

namespace cgogn
{

template <typename VEC3 , typename MAP>
class ReebGraph
{
public:

	using Vec3 = VEC3;
	using Scalar = typename Vec3::Scalar;

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	template <typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;
	template <typename T>
	using EdgeAttribute = typename MAP::template EdgeAttribute<T>;

	using EdgeMarker = typename MAP::template CellMarker<Edge::ORBIT>;

	using DGraph = DirectedGraph<cgogn::DefaultMapTraits>;
	using Node = DGraph::Vertex ;
	using Arc = DGraph::Edge;
	template <typename T>
	using NodeAttribute = typename DGraph::template VertexAttribute<T>;
	template <typename T>
	using ArcAttribute = typename DGraph::template EdgeAttribute<T>;

	using Edges = std::vector<Edge>;


public:
	DGraph graph_;

private:
	MAP& map_;

	EdgeAttribute<Arc> highest_arc_;
	VertexAttribute<Node> node_link_;


	NodeAttribute<Vertex> corresponding_vertex_;
	NodeAttribute<Scalar> function_value_;
	NodeAttribute<bool> is_finalized_;
	//	NodeAttribute<VEC3> node_positions_;
	ArcAttribute<Edges> intersecting_edges_;

	Scalar minimum_value_;
	Scalar maximum_value_;

public:

	ReebGraph(MAP& map): map_(map)
	{
		corresponding_vertex_ = graph_.add_attribute<Vertex, Node::ORBIT>("corresponding_vertex");
		function_value_ = graph_.add_attribute<Scalar, Node::ORBIT>("function_value");
		is_finalized_ = graph_.add_attribute<bool, Node::ORBIT>("is_finalized");
		//		node_positions_ = graph_.add_attribute<VEC3, Node::ORBIT>("node_positions");
		intersecting_edges_ = graph_.add_attribute<Edges, Arc::ORBIT>("intersecting_edges");


		highest_arc_ = map_.template add_attribute<Arc, Edge::ORBIT>("highest_arc");
		node_link_ = map_.template add_attribute<Node, Vertex::ORBIT>("node_link");
	}


	void compute(VertexAttribute<Scalar>& scalar_field)
	{
		EdgeMarker em_(map_);

		//addMeshVertex()
		map_.foreach_cell([&] (Vertex v)
		{
			create_node(v, scalar_field[v]);
		});

		//addMeshTriangle
		map_.foreach_cell([&] (Face f)
		{
			map_.foreach_incident_edge(f, [&] (Edge e)
			{
				//e is new
				if(!em_.is_marked(e))
				{
					em_.mark(e);
					create_arc(e);
				}
			});

			//call e0, e1, e2 the edges of f
			//with e0, e1 sharing the maximum
			//and e0, e2 the minimum
			Edge e0(f.dart);
			Edge e1(map_.phi1(f.dart));
			Edge e2(map_.phi_1(f.dart));

			//			marge_paths(e0,e1,e2);

			//			map_.foreach_incident(f, [&] (Edge e)
			//			{
			//				//if all vertices in e are finalized
			//				//remove_edge(e);
			//			});
		});
	}

	Vertex linked_vertex(Node n)
	{
		return node_link_[n];
	}

public:

	/**
	 * \brief Add a new node with index \p i, and function value \p w to \p RG
	 * @param index
	 * @param function_value
	 */
	void create_node(Vertex v, Scalar w)
	{
		Node n = graph_.add_vertex();

		corresponding_vertex_[n] = v;
		function_value_[n] = w;
		is_finalized_.set_value(n, false);

//		highest_arc_[Edge(v.dart)] = n;

		node_link_[v] = n;

		minimum_value_ = std::min(minimum_value_, w);
		maximum_value_ = std::max(maximum_value_, w);

	}

	void create_arc(Edge e)
	{
		std::pair<Vertex,Vertex> v = map_.vertices(e);
		Node n0 = node_link_[v.first];
		Node n1 = node_link_[v.second];

		graph_.connect_vertices(n0,n1);
	}
};

} // namepsace cgogn

#endif // DIFFERENTIAL_TOPOLOGY_REEB_GRAPH_H_
