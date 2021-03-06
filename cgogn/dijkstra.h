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

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <map>
#include <set>
#include <vector>
#include <algorithm>

#include "directed_graph.h"

namespace cgogn
{

template <typename T, typename MAP>
void dijkstra_compute_paths(
		MAP& map,
		const typename MAP::template EdgeAttribute<T>& weight,
		const std::vector<typename MAP::Vertex> sources,
		typename MAP::template VertexAttribute<T>& distance_to_source,
		typename MAP::template VertexAttribute<typename MAP::Vertex>& path_to_source)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	for(auto& d : distance_to_source)
		d = std::numeric_limits<T>::max();

	for(auto& p : path_to_source)
		p = Vertex();

	using my_pair = std::pair<T, unsigned int>;
	using my_queue = std::priority_queue<my_pair, std::vector<my_pair>, std::greater<my_pair> >;

	my_queue vertex_queue;

	for(auto& source : sources)
	{
		vertex_queue.push(std::make_pair(T(0), source.dart.index));
		distance_to_source[source] = T(0);
		path_to_source[source] = source;
	}

	while(!vertex_queue.empty())
	{
		T dist = vertex_queue.top().first;
		Vertex u = Vertex(Dart(vertex_queue.top().second));

		vertex_queue.pop();

		map.foreach_adjacent_vertex_through_edge(u, [&](Vertex v)
		{
			T distance_through_u = dist + weight[Edge(v.dart)];
			if(distance_through_u < distance_to_source[v])
			{
				vertex_queue.push(std::make_pair(distance_through_u, v.dart.index));
				distance_to_source[v] = distance_through_u;
				path_to_source[v] = u;
			}
		});
	}
}

template <typename T, typename MAP>
void dijkstra_compute_normalized_paths(
		MAP& map,
		const typename MAP::template EdgeAttribute<T>& weight,
		const std::vector<typename MAP::Vertex> sources,
		typename MAP::template VertexAttribute<T>& min_distance,
		typename MAP::template VertexAttribute<typename MAP::Vertex>& min_source)
{
	dijkstra_compute_paths<T>(map, weight, sources, min_distance, min_source);

	//find max of min_distance
	double max_d = std::numeric_limits<double>::min();
	double min_d = std::numeric_limits<double>::max();

	for(auto& d : min_distance)
	{
		max_d = std::max(max_d, d);
		min_d = std::min(min_d, d);
	}

	//normalize
	for(auto& d : min_distance)
		d = (d - min_d) / (max_d - min_d);
}

template <typename Scalar, typename MAP>
typename MAP::Vertex argmin(
		const MAP& map,
		const typename MAP::template VertexAttribute<Scalar>& attribut)
{
	using Vertex = typename MAP::Vertex;

	Scalar min = std::numeric_limits<Scalar>::infinity();
	Vertex d_min;
	map.foreach_cell([&] (Vertex v)
	{
		Scalar cur = attribut[v];
		if(cur < min)
		{
			d_min = v;
			min = cur;
		}
	});

	return d_min;
}

template <typename Scalar, typename MAP>
typename MAP::Vertex argmin(
		const std::map<uint32, typename MAP::Vertex> v,
		const typename MAP::template VertexAttribute<Scalar>& attribut)
{
	using Vertex = typename MAP::Vertex;

	Scalar min = std::numeric_limits<Scalar>::infinity();
	Vertex d_min;

	for (auto it : v)
	{
		Scalar cur = attribut[it.second];
		if(cur < min)
		{
			d_min = it.second;
			min = cur;
		}
	}

	return d_min;
}

//Function value driven dijkstra based function
template <typename T, typename MAP>
void dijkstra_compute_perturbated_function(
		MAP& map,
		const typename MAP::template VertexAttribute<T>& f,
		typename MAP::template VertexAttribute<T>& f_pertubated)
{
	using Vertex = typename MAP::Vertex ;

	unsigned int i = 0;
	std::map<unsigned int, Dart> visited;
	std::map<unsigned int, Dart> candidates;

	Dart dmin = argmin<T>(map, f);
	candidates.insert(std::pair<unsigned int, Dart>(map.template get_embedding(Vertex(dmin)), dmin));

	unsigned int Nv = map.template nb_cells<Vertex::ORBIT>();

	do
	{
		Dart vt = argmin<T,MAP>(candidates, f);

		f_pertubated[Vertex(vt)] = static_cast<T>(i) / static_cast<T>(Nv);

		candidates.erase(map.template get_embedding(Vertex(vt)));

		visited.insert(std::pair<unsigned int, Dart>(map.template get_embedding(Vertex(vt)), vt));

		map.foreach_adjacent_vertex_through_edge(Vertex(vt), [&](Vertex v){
			if(visited.find(map.template get_embedding(Vertex(v))) == visited.end())
				candidates.insert(std::pair<unsigned int, Dart>(map.template get_embedding(Vertex(v)), v));
		});

		++i;
	}
	while(!candidates.empty());
}

template <typename MAP>
void dijkstra_compute_shortest_path_to(
		const typename MAP::Vertex v,
		const typename MAP::template VertexAttribute< typename MAP::Vertex>& previous,
		std::vector<Dart>& path)
{
	using Vertex = typename MAP::Vertex;

	path.clear();
	Dart source = v ;
	Dart end = Dart();

	for( ; source != end; source = previous[Vertex(source)])
	{
		path.push_back(source);
	}

	path.push_back(v);
}


template <typename Scalar, typename  MAP>
void reeb_graph(
		MAP& map,
		const typename MAP::template VertexAttribute<Scalar>& f)
		//DirectedGraph<DefaultMapTraits>& g)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	typename MAP::template VertexAttribute<uint32> vindices = map.template add_attribute<uint32, Vertex::ORBIT>("indices");
	uint32 count = 0;
	map.foreach_cell([&] (Vertex v) { vindices[v] = count++; });

	std::map<uint32,Vertex> min;
	map.foreach_cell([&] (Vertex v)
	{
		Scalar center = f[v];
		bool is_lower = true;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex vn)
		{
			if(f[vn] < center)
				is_lower = false;
		});

		if(is_lower)
			min.insert(std::pair<uint32,Vertex>(vindices[v], v));
	});

	Vertex vm = argmin(map, f);
	min.erase(min.find(vindices[vm]));

	std::map<uint32, Vertex> sub_level_set;
	std::map<uint32, Vertex> level_set;

	level_set.insert(std::pair<uint32,Vertex>(vindices[vm], vm));

	do
	{
		Vertex vt = argmin(level_set, f);
		Vertex vmin = argmin(min, f);

		if(f[vmin] < f[vt])
		{
			vt = vmin;
			min.erase(min.find(vindices[vmin]));
			level_set.insert(std::pair<uint32, Vertex>(vindices[vt], vt));
		}

		// compute discrete contour
		discrete_contour(vt, level_set);

		//

		level_set.erase(std::pair<uint32, Vertex>(vindices[vt], vt));
		sub_level_set.insert(std::pair<uint32, Vertex>(vindices[vt], vt));

		Scalar center = f[vt];
		map.foreach_adjacent_vertex_through_edge(vt, [&] (Vertex vn)
		{
			Scalar current = f[vn];
			if(current > center)
				level_set.insert(std::pair<uint32, Vertex>(vindices[vn], vn));

		});


	}while(!level_set.empty());

}

}
#endif // DIJKSTRA_H
