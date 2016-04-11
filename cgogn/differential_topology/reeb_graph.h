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

#include "../undirected_graph.h"

namespace cgogn
{

template <typename T, typename MAP>
class ReebGraph
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	template <typename V>
	using VertexAttribute = typename MAP::template VertexAttribute<V>;

	using UGraph = UndirectedGraph<cgogn::DefaultMapTraits>;
	using Node = UGraph::Vertex ;
	using Arc = UGraph::Edge;

private:
	// Node structure
	typedef struct
	{
		unsigned int vertex_id_;
		T value_;
		bool  is_finalized_;
		bool is_critical_;
	} ReebNode;

	// Arc structure
	typedef struct
	{
		unsigned int  label_id_0_, label_id_1_;
	} ReebArc;

	// Label structure
	typedef struct
	{
		unsigned int  arc_id_;
		unsigned long long label_;
	} ReebLabel;


	MAP* map_;
	UGraph graph_;

//	VertexAttribute<ReebNode>& nodes;

	//data storage
	std::map<int, int> vertex_stream_;

	struct ReebPath
	{
		T  simplification_value_;
		int  arc_number_;
		unsigned int*  arc_table_;
		int  node_number_;
		unsigned int* node_table_;

		inline bool operator<( struct ReebPath const &E ) const
		{
			return !(
						(simplification_value_ < E.simplification_value_) ||
						(simplification_value_ == E.simplification_value_
						 && arc_number_ < E.arc_number_) ||
						(simplification_value_ == E.simplification_value_
						 && arc_number_ == E.arc_number_
						 && node_table_[node_number_ - 1] < E.node_table_[E.node_number_ - 1]));
			/*      return !((
			(MaximumScalarValue - MinimumScalarValue)
			  < (E.MaximumScalarValue - E.MinimumScalarValue)) ||
			   ((MaximumScalarValue - MinimumScalarValue)
				 == (E.MaximumScalarValue-E.MinimumScalarValue)
				   && ArcNumber < E.ArcNumber) ||
			   ((MaximumScalarValue - MinimumScalarValue)
				 == (E.MaximumScalarValue - E.MinimumScalarValue)
				   && ArcNumber == E.ArcNumber
					 && NodeTable[NodeNumber - 1]<E.NodeTable[E.NodeNumber - 1])
			 );*/
		}
	};

public:

//	ReebGraph(MAP* map):
//		graph_(),
//		map_(map)
//	{}

	ReebGraph():
		graph_()
	{}

	~ReebGraph()
	{}

	void build(VertexAttribute<T>& scalar_field)
	{
		map_->foreach_cell([&] (Face f)
		{
			Vertex v0(f);
			Vertex v1(map_->phi1(f));
			Vertex v2(map_->phi_1(f));

			stream_triangle(v0, scalar_field[v0],
							v1, scalar_field[v1],
							v2, scalar_field[v2]);
		});

		close_stream();
	}

	/**
	 * @brief Streaming reeb graph computation
	 * \details Add to the streaming computation the triangle of the \p MAP surface mesh
	 * @param v0 is the Id of the vertex in the \p MAP structure
	 * @param scalar0 is the corresponding scalar field value
	 * @param v1
	 * @param scalar1
	 * @param v2
	 * @param scalar2
	 */
	void stream_triangle(Vertex v0, T scalar0,
						 Vertex v1, T scalar1,
						 Vertex v2, T scalar2)
	{
		//add vertices to the stream
		create_node(v0, scalar0);
		create_node(v1, scalar1);
		create_node(v2, scalar2);


		std::map<int, int>::iterator s_iter;

		//v0
//		s_iter = vertex_stream.find(v0);
//		if(s_iter == vertex_stream.end())
//		{
//			// this vertex hasn't been streamed yet, let's add it
//			vertex_stream[v0] = this->VertexMapSize;
//			this->VertexMap[this->VertexMapSize]
//					= this->AddMeshVertex(v0, scalar0);
//			this->VertexMapSize++;
//			this->TriangleVertexMapSize++;
//		}

		//v1

		//v2

		add_mesh_triangle(v0, scalar0, v1, scalar1, v2, scalar2);
	}

	void add_mesh_triangle(Vertex v0, T scalar0,
						 Vertex v1, T scalar1,
						 Vertex v2, T scalar2)
	{

	}

	/**
	 * @brief Finalize internal data structure, in the case of streaming computations
	 * \details After this call, no more triangle can be inserted via stream_triangle
	 * This method must be called when the input stream is finished.
	 * If you need to get a snapshot of the reeb graph during the streaming process
	 * (to parse or simplify it) do a deep copy followed by a close_stream on the copy
	 */
	void close_stream()
	{

	}


public:



	/**
	 * \brief Add a new node with index \p i, and function value \p w to \p RG
	 * @param index
	 * @param function_value
	 */
	void create_node(unsigned int index, double w)
	{

	}
};

} // namepsace cgogn

#endif // DIFFERENTIAL_TOPOLOGY_REEB_GRAPH_H_
