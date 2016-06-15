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
