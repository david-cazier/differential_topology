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

#ifndef GEOMETRY_ALGOS_AREA2_H_
#define GEOMETRY_ALGOS_AREA2_H_

#include <cmath>

#include <geometry/types/geometry_traits.h>
#include <geometry/algos/area.h>

namespace cgogn
{

namespace geometry
{


template <typename VEC3_T, typename MAP>
inline typename VEC3_T::Scalar incident_faces_area(
		const MAP& map,
		const typename MAP::Edge e,
		const typename MAP::template VertexAttributeHandler<VEC3_T>& position)
{
	using Scalar = typename VEC3_T::Scalar;
	using Face = typename MAP::Face;

	Scalar area(0) ;

	map.foreach_incident_face(e, [&] (Face f)
	{
		area += cgogn::geometry::convex_face_area<VEC3_T, MAP>(map, f, position) / map.codegree(f) ;
	});

	return area ;
}

template <typename VEC3_T, typename MAP>
inline void incident_faces_area(
		const MAP& map,
		const typename MAP::template VertexAttributeHandler<VEC3_T>& position,
		typename MAP::template EdgeAttributeHandler<typename VEC3_T::Scalar>& edge_area)
{
	using Edge = typename MAP::Edge;

	map.foreach_cell([&] (Edge e)
	{
		edge_area[e] = incident_faces_area<VEC3_T, MAP>(map, e, position);
	});
}

}

}

#endif // GEOMETRY_ALGOS_AREA2_H_
